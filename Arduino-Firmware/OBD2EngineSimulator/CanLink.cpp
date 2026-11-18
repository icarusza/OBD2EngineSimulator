#include "CanLink.h"
#include "EngineState.h"
#include "SerialCommands.h"
#include <SPI.h>
#include <EEPROM.h>

static MCP_CAN CAN(SPI_CS_PIN);

// =============================================
// CAN MODES (ISO 15765-4)
// =============================================

static const CanModeInfo CAN_MODES[] =
{
    { 6, false, CAN_500KBPS, 0x7DFUL,      0x7E0UL,      0x7E8UL      },
    { 7, true,  CAN_500KBPS, 0x18DB33F1UL, 0x18DA10F1UL, 0x18DAF110UL },
    { 8, false, CAN_250KBPS, 0x7DFUL,      0x7E0UL,      0x7E8UL      },
    { 9, true,  CAN_250KBPS, 0x18DB33F1UL, 0x18DA10F1UL, 0x18DAF110UL }
};

static const uint8_t CAN_MODE_COUNT = sizeof(CAN_MODES) / sizeof(CAN_MODES[0]);

static const CanModeInfo *activeMode = &CAN_MODES[0];
static uint8_t pendingMode = 0;
static bool canReady = false;
static bool isoTpBusy = false;
static unsigned long lastInitAttemptMs = 0;

static const CanModeInfo *FindMode(uint8_t mode)
{
    for (uint8_t i = 0; i < CAN_MODE_COUNT; i++)
    {
        if (CAN_MODES[i].mode == mode)
            return &CAN_MODES[i];
    }

    return NULL;
}

bool CanIsValidMode(uint8_t mode)
{
    return FindMode(mode) != NULL;
}

uint8_t CanActiveMode() { return activeMode->mode; }
bool CanReady() { return canReady; }
bool IsoTpBusy() { return isoTpBusy; }

// =============================================
// EEPROM
// =============================================

static uint8_t LoadSavedMode()
{
    if (EEPROM.read(EEPROM_ADDR_MAGIC) == EEPROM_MAGIC)
    {
        uint8_t mode = EEPROM.read(EEPROM_ADDR_CAN_MODE);

        if (CanIsValidMode(mode))
            return mode;
    }

    return DEFAULT_CAN_MODE;
}

static void SaveMode(uint8_t mode)
{
    EEPROM.update(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);
    EEPROM.update(EEPROM_ADDR_CAN_MODE, mode);
}

// =============================================
// MCP2515 INITIALISATION
// =============================================

static void PrintModeName(const CanModeInfo *mode)
{
    Serial.print(mode->extended ? F("29-bit") : F("11-bit"));
    Serial.print(mode->speed == CAN_500KBPS ? F(" 500 kbit/s") : F(" 250 kbit/s"));
}

static bool StartCan(const CanModeInfo *mode)
{
    activeMode = mode;
    canReady = false;
    lastInitAttemptMs = millis();

    if (CAN.begin(MCP_STDEXT, mode->speed, MCP_CLOCK) != CAN_OK)
    {
        Serial.println(F("ERROR: MCP2515 initialisation failed, retrying"));
        return false;
    }

    // Accept only the functional and physical request IDs. RXB0 uses mask 0
    // with filters 0-1, RXB1 uses mask 1 with filters 2-5.
    //
    // coryjfowler MCP_CAN quirk: with ext = 0, the standard ID is taken from
    // bits 16..26 of the value (the low 16 bits filter the first two data
    // bytes), so standard masks and filters are shifted left by 16.
    if (mode->extended)
    {
        CAN.init_Mask(0, 1, 0x1FFFFFFFUL);
        CAN.init_Filt(0, 1, mode->functionalId);
        CAN.init_Filt(1, 1, mode->physicalId);

        CAN.init_Mask(1, 1, 0x1FFFFFFFUL);
        CAN.init_Filt(2, 1, mode->functionalId);
        CAN.init_Filt(3, 1, mode->physicalId);
        CAN.init_Filt(4, 1, mode->functionalId);
        CAN.init_Filt(5, 1, mode->physicalId);
    }
    else
    {
        unsigned long functional = mode->functionalId << 16;
        unsigned long physical = mode->physicalId << 16;

        CAN.init_Mask(0, 0, 0x07FF0000UL);
        CAN.init_Filt(0, 0, functional);
        CAN.init_Filt(1, 0, physical);

        CAN.init_Mask(1, 0, 0x07FF0000UL);
        CAN.init_Filt(2, 0, functional);
        CAN.init_Filt(3, 0, physical);
        CAN.init_Filt(4, 0, functional);
        CAN.init_Filt(5, 0, physical);
    }

    CAN.setMode(MCP_NORMAL);
    canReady = true;

    Serial.print(F("CAN READY | MODE "));
    Serial.print(mode->mode);
    Serial.print(F(" | "));
    PrintModeName(mode);
    Serial.print(F(" | REQUEST 0x"));
    Serial.print(mode->functionalId, HEX);
    Serial.print(F("/0x"));
    Serial.print(mode->physicalId, HEX);
    Serial.print(F(" | RESPONSE 0x"));
    Serial.println(mode->responseId, HEX);

    return true;
}

void CanSetup()
{
    StartCan(FindMode(LoadSavedMode()));
}

void CanService()
{
    if (isoTpBusy)
        return;

    if (pendingMode != 0)
    {
        const CanModeInfo *mode = FindMode(pendingMode);
        pendingMode = 0;

        if (mode != NULL && StartCan(mode))
            SaveMode(mode->mode);

        return;
    }

    if (!canReady && millis() - lastInitAttemptMs >= CAN_INIT_RETRY_MS)
    {
        if (StartCan(activeMode))
            SaveMode(activeMode->mode);
    }
}

bool CanRequestMode(uint8_t mode, bool &applied, bool &ok)
{
    const CanModeInfo *info = FindMode(mode);

    applied = false;
    ok = false;

    if (info == NULL)
        return false;

    // Same mode and running: nothing to do (avoids resetting the MCP2515).
    if (info == activeMode && canReady)
    {
        applied = true;
        ok = true;
        pendingMode = 0;
        return true;
    }

    if (isoTpBusy)
    {
        pendingMode = mode;
        return true;
    }

    applied = true;
    ok = StartCan(info);

    if (ok)
        SaveMode(mode);

    return true;
}

// =============================================
// FRAME I/O
// =============================================

static void PrintFrame(const __FlashStringHelper *label, unsigned long id,
                       const uint8_t *data, uint8_t length)
{
    // "CAN RX | ID: 0x18DB33F1 | DATA: " + 8 bytes + CRLF; dropped if it would block.
    if (!LogLineFits(60))
        return;

    Serial.print(label);
    Serial.print(F(" | ID: 0x"));
    Serial.print(id, HEX);
    Serial.print(F(" | DATA: "));

    for (uint8_t i = 0; i < length && i < 8; i++)
    {
        if (data[i] < 0x10)
            Serial.print('0');

        Serial.print(data[i], HEX);
        Serial.print(' ');
    }

    Serial.println();
}

bool CanReceive(CanFrame &frame)
{
    if (!canReady || CAN.checkReceive() != CAN_MSGAVAIL)
        return false;

    unsigned long id = 0;
    uint8_t extended = 0;
    uint8_t length = 0;

    // The DLC field is 4 bits (up to 15); read into a buffer large enough
    // for any DLC in case the library does not clamp it to 8.
    uint8_t buffer[16];
    memset(buffer, 0, sizeof(buffer));

    if (CAN.readMsgBuf(&id, &extended, &length, buffer) != CAN_OK)
        return false;

    frame.id = id;
    frame.extended = extended != 0;
    frame.length = length > 8 ? 8 : length;
    memcpy(frame.data, buffer, 8);

    if (engine.logLevel >= 2)
        PrintFrame(F("CAN RX"), frame.id, frame.data, frame.length);

    return true;
}

static bool SendFrame(uint8_t *data)
{
    byte result = CAN.sendMsgBuf(
        activeMode->responseId,
        activeMode->extended ? 1 : 0,
        8,
        data);

    if (engine.logLevel >= 2)
        PrintFrame(F("CAN TX"), activeMode->responseId, data, 8);

    if (result != CAN_OK)
    {
        Serial.print(F("ERROR: CAN TX failed, code "));
        Serial.println(result);
        return false;
    }

    return true;
}

bool CanIsRequest(const CanFrame &frame, bool &physical)
{
    if (frame.extended != activeMode->extended)
        return false;

    if (frame.id == activeMode->functionalId)
    {
        physical = false;
        return true;
    }

    if (frame.id == activeMode->physicalId)
    {
        physical = true;
        return true;
    }

    return false;
}

// =============================================
// ISO-TP (ISO 15765-2) TRANSMIT
// =============================================

// Waits while still handling serial input, so the UNO's 64-byte UART
// buffer does not overflow during long transfers.
static void WaitWithSerial(unsigned long ms)
{
    unsigned long start = millis();

    while (millis() - start < ms)
        SerialPump();
}

static unsigned long DecodeStMin(uint8_t stMin)
{
    if (stMin <= 0x7F)
        return stMin;

    if (stMin >= 0xF1 && stMin <= 0xF9)
        return 1;  // 100-900 us, rounded up to 1 ms

    return 0x7F;   // reserved values: use the maximum
}

static bool WaitForFlowControl(uint8_t &blockSize, unsigned long &stMinMs)
{
    unsigned long start = millis();
    uint8_t waitFrames = 0;

    while (millis() - start < ISOTP_FLOW_CONTROL_TIMEOUT_MS)
    {
        SerialPump();

        CanFrame frame;

        if (!CanReceive(frame))
            continue;

        if (frame.extended != activeMode->extended ||
            frame.id != activeMode->physicalId ||
            (frame.data[0] & 0xF0) != 0x30)
            continue;  // not a Flow Control frame for us

        uint8_t flowStatus = frame.data[0] & 0x0F;

        if (flowStatus == 0)  // continue to send
        {
            blockSize = frame.data[1];
            stMinMs = DecodeStMin(frame.data[2]);
            return true;
        }

        if (flowStatus == 1 && ++waitFrames <= ISOTP_MAX_WAIT_FRAMES)  // wait
        {
            start = millis();
            continue;
        }

        Serial.println(F("ERROR: ISO-TP transfer aborted by tester"));
        return false;  // overflow, invalid, or too many waits
    }

    Serial.println(F("ERROR: ISO-TP flow control timeout"));
    return false;
}

bool IsoTpSend(const uint8_t *payload, uint8_t length)
{
    uint8_t frame[8];

    if (length <= 7)
    {
        memset(frame, ISOTP_PADDING, sizeof(frame));
        frame[0] = length;
        memcpy(&frame[1], payload, length);
        return SendFrame(frame);
    }

    isoTpBusy = true;

    // First Frame: 12-bit length, first 6 payload bytes.
    frame[0] = 0x10 | ((length >> 8) & 0x0F);
    frame[1] = length & 0xFF;
    memcpy(&frame[2], payload, 6);

    bool ok = SendFrame(frame);
    uint8_t offset = 6;
    uint8_t sequence = 1;

    while (ok && offset < length)
    {
        uint8_t blockSize = 0;
        unsigned long stMinMs = 0;

        if (!WaitForFlowControl(blockSize, stMinMs))
        {
            ok = false;
            break;
        }

        uint8_t sentInBlock = 0;

        while (ok && offset < length &&
               (blockSize == 0 || sentInBlock < blockSize))
        {
            if (sentInBlock > 0)
                WaitWithSerial(stMinMs);

            uint8_t chunk = length - offset;

            if (chunk > 7)
                chunk = 7;

            memset(frame, ISOTP_PADDING, sizeof(frame));
            frame[0] = 0x20 | (sequence & 0x0F);
            memcpy(&frame[1], payload + offset, chunk);

            ok = SendFrame(frame);

            offset += chunk;
            sequence++;
            sentInBlock++;
        }
    }

    isoTpBusy = false;
    return ok;
}
