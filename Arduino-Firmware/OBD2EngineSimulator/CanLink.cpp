#include "CanLink.h"
#include "SerialCommands.h"
#include <SPI.h>
#include <EEPROM.h>

static MCP_CAN CAN(SPI_CS_PIN);

// =============================================
// CAN MODES (ISO 15765-4)
// =============================================

struct CanModeInfo
{
    uint8_t mode;            // ELM327 protocol number 6..9
    bool extended;           // 29-bit identifiers
    uint8_t speed;           // MCP_CAN speed constant
};

static const CanModeInfo CAN_MODES[] =
{
    { 6, false, CAN_500KBPS },
    { 7, true,  CAN_500KBPS },
    { 8, false, CAN_250KBPS },
    { 9, true,  CAN_250KBPS }
};

static const uint8_t CAN_MODE_COUNT = sizeof(CAN_MODES) / sizeof(CAN_MODES[0]);

static const CanModeInfo *activeMode = &CAN_MODES[0];
static uint8_t pendingMode = 0;
static bool canReady = false;
static uint8_t busyDepth = 0;
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

bool CanIsValidMode(uint8_t mode) { return FindMode(mode) != NULL; }
uint8_t CanActiveMode() { return activeMode->mode; }
bool CanReady() { return canReady; }
bool CanBusy() { return busyDepth > 0; }

// =============================================
// ECU ADDRESSES
// =============================================
// 11-bit: physical request 7E0+n, response 7E8+n, functional 7DF.
// 29-bit: physical 18DA<addr>F1, response 18DAF1<addr>, functional 18DB33F1.

static uint8_t Address29(EcuIndex ecu)
{
    return ecu == ECU_TCM ? TCM_ADDRESS_29 : ECM_ADDRESS_29;
}

static unsigned long FunctionalId(const CanModeInfo *mode)
{
    return mode->extended ? 0x18DB33F1UL : 0x7DFUL;
}

static unsigned long PhysicalId(const CanModeInfo *mode, EcuIndex ecu)
{
    return mode->extended
        ? 0x18DA00F1UL | ((unsigned long)Address29(ecu) << 8)
        : 0x7E0UL + ecu;
}

unsigned long CanResponseId(EcuIndex ecu)
{
    return activeMode->extended
        ? 0x18DAF100UL | Address29(ecu)
        : 0x7E8UL + ecu;
}

// =============================================
// EEPROM (CAN mode)
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
// REQUEST QUEUE AND ISO-TP RECEIVE STATE
// =============================================

static CanFrame queue[REQUEST_QUEUE_SIZE];
static uint8_t queueHead = 0;
static uint8_t queueCount = 0;

static uint8_t rxBuffer[ISOTP_RX_BUFFER];
static bool rxActive = false;
static bool rxComplete = false;
static EcuIndex rxEcu = ECU_ECM;
static uint8_t rxTotal = 0;
static unsigned long rxDeadlineMs = 0;
#if FEATURE_UDS
static uint8_t rxPosition = 0;
static uint8_t rxSequence = 0;
#endif

static void ResetQueues()
{
    queueHead = 0;
    queueCount = 0;
    rxActive = false;
    rxComplete = false;
}

static void Enqueue(const CanFrame &frame)
{
    if (queueCount >= REQUEST_QUEUE_SIZE)
    {
        Serial.println(F("ERROR: request queue full, frame dropped"));
        return;
    }

    queue[(queueHead + queueCount) % REQUEST_QUEUE_SIZE] = frame;
    queueCount++;
}

// =============================================
// MCP2515 INITIALISATION
// =============================================

static bool StartCan(const CanModeInfo *mode)
{
    activeMode = mode;
    canReady = false;
    lastInitAttemptMs = millis();
    ResetQueues();

    if (CAN.begin(MCP_STDEXT, mode->speed, MCP_CLOCK) != CAN_OK)
    {
        Serial.println(F("ERROR: MCP2515 init failed, retrying"));
        return false;
    }

    // Accept only our request IDs. RXB0: mask 0, filters 0-1 (functional,
    // ECM physical). RXB1: mask 1, filters 2-5 (TCM physical first, since
    // RXB0 has only two filters).
    //
    // coryjfowler MCP_CAN quirk: with ext = 0, the standard ID is taken from
    // bits 16..26 of the value (the low 16 bits filter the first two data
    // bytes), so standard masks and filters are shifted left by 16.
    uint8_t ext = mode->extended ? 1 : 0;
    uint8_t shift = mode->extended ? 0 : 16;
    unsigned long mask = mode->extended ? 0x1FFFFFFFUL : 0x07FF0000UL;
    unsigned long functional = FunctionalId(mode) << shift;
    unsigned long ecm = PhysicalId(mode, ECU_ECM) << shift;
    unsigned long tcm = PhysicalId(mode, ECU_TCM) << shift;

    CAN.init_Mask(0, ext, mask);
    CAN.init_Filt(0, ext, functional);
    CAN.init_Filt(1, ext, ecm);

    CAN.init_Mask(1, ext, mask);
    CAN.init_Filt(2, ext, tcm);
    CAN.init_Filt(3, ext, functional);
    CAN.init_Filt(4, ext, ecm);
    CAN.init_Filt(5, ext, tcm);

    CAN.setMode(MCP_NORMAL);
    canReady = true;

    Serial.print(F("CAN READY | MODE "));
    Serial.print(mode->mode);
    Serial.print(mode->extended ? F(" | 29-bit") : F(" | 11-bit"));
    Serial.println(mode->speed == CAN_500KBPS ? F(" 500 kbit/s") : F(" 250 kbit/s"));

    return true;
}

void CanSetup()
{
    StartCan(FindMode(LoadSavedMode()));
}

void CanService()
{
    if (rxActive && (long)(millis() - rxDeadlineMs) > 0)
    {
        rxActive = false;
        Serial.println(F("ERROR: ISO-TP receive timeout"));
    }

    if (CanBusy())
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

    if (CanBusy())
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

static bool CanReceive(CanFrame &frame)
{
    if (!canReady || CAN.checkReceive() != CAN_MSGAVAIL)
        return false;

    unsigned long id = 0;
    uint8_t extended = 0;
    uint8_t length = 0;

    // The DLC field is 4 bits (up to 15) and the library copies that many
    // bytes, so read into a buffer large enough for any DLC.
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

bool CanNextFrame(CanFrame &frame)
{
    if (queueCount > 0)
    {
        frame = queue[queueHead];
        queueHead = (queueHead + 1) % REQUEST_QUEUE_SIZE;
        queueCount--;
        return true;
    }

    return CanReceive(frame);
}

static bool SendFrame(EcuIndex ecu, uint8_t *data)
{
    unsigned long id = CanResponseId(ecu);
    byte result = CAN.sendMsgBuf(id, activeMode->extended ? 1 : 0, 8, data);

    if (engine.logLevel >= 2)
        PrintFrame(F("CAN TX"), id, data, 8);

    if (result != CAN_OK)
    {
        Serial.print(F("ERROR: CAN TX failed, code "));
        Serial.println(result);
        return false;
    }

    return true;
}

uint8_t CanRequestTarget(const CanFrame &frame)
{
    if (frame.extended != activeMode->extended)
        return TARGET_NONE;

    if (frame.id == FunctionalId(activeMode))
        return TARGET_FUNCTIONAL;

    if (frame.id == PhysicalId(activeMode, ECU_ECM))
        return ECU_ECM;

#if FEATURE_TCM
    if (frame.id == PhysicalId(activeMode, ECU_TCM))
        return ECU_TCM;
#endif

    return TARGET_NONE;
}

// =============================================
// ISO-TP (ISO 15765-2) RECEIVE
// =============================================

bool IsoTpReceive(const CanFrame &frame, uint8_t target)
{
#if FEATURE_UDS
    uint8_t type = frame.data[0] & 0xF0;

    if (type == 0x10)  // First Frame
    {
        // Multi-frame requests are physical only (ISO 15765-4), and an absent
        // TCM must not answer with Flow Control.
        if (target == TARGET_FUNCTIONAL || target == TARGET_NONE || frame.length < 8)
            return true;

        if (target == ECU_TCM && !engine.p.tcmEnabled)
            return true;

        uint16_t total = ((uint16_t)(frame.data[0] & 0x0F) << 8) | frame.data[1];
        uint8_t flowControl[8] = { 0x30, 0x00, 0x00, ISOTP_PADDING, ISOTP_PADDING,
                                   ISOTP_PADDING, ISOTP_PADDING, ISOTP_PADDING };

        if (total <= 7)
            return true;  // invalid First Frame: ignore

        if (total > ISOTP_RX_BUFFER)
        {
            flowControl[0] = 0x32;  // overflow
            SendFrame((EcuIndex)target, flowControl);
            rxActive = false;
            return true;
        }

        rxActive = true;
        rxComplete = false;
        rxEcu = (EcuIndex)target;
        rxTotal = (uint8_t)total;
        memcpy(rxBuffer, &frame.data[2], 6);
        rxPosition = 6;
        rxSequence = 1;
        rxDeadlineMs = millis() + ISOTP_CONSECUTIVE_TIMEOUT_MS;

        SendFrame(rxEcu, flowControl);
        return true;
    }

    if (type == 0x20)  // Consecutive Frame
    {
        if (!rxActive || target != rxEcu)
            return true;  // not expected: ignore

        if ((frame.data[0] & 0x0F) != (rxSequence & 0x0F))
        {
            Serial.println(F("ERROR: ISO-TP sequence error"));
            rxActive = false;
            return true;
        }

        uint8_t chunk = rxTotal - rxPosition;

        if (chunk > 7)
            chunk = 7;

        memcpy(&rxBuffer[rxPosition], &frame.data[1], chunk);
        rxPosition += chunk;
        rxSequence++;
        rxDeadlineMs = millis() + ISOTP_CONSECUTIVE_TIMEOUT_MS;

        if (rxPosition >= rxTotal)
        {
            rxActive = false;
            rxComplete = true;
        }

        return true;
    }
#else
    (void)frame;
    (void)target;
#endif

    return false;
}

bool IsoTpTakeRequest(EcuIndex &ecu, uint8_t *buffer, uint8_t &length)
{
    if (!rxComplete)
        return false;

    rxComplete = false;
    ecu = rxEcu;
    length = rxTotal;
    memcpy(buffer, rxBuffer, rxTotal);
    return true;
}

// =============================================
// WAITING WITHOUT DROPPING REQUESTS
// =============================================

// Handles a frame that arrives while we are busy: multi-frame request data
// is collected, other requests are queued for later.
static void HandleFrameWhileBusy(const CanFrame &frame)
{
    uint8_t target = CanRequestTarget(frame);

    if (target == TARGET_NONE)
        return;

    uint8_t type = frame.data[0] & 0xF0;

    // Multi-frame request data is handled at once: the First Frame needs its
    // Flow Control within the tester's N_Bs, not after our delay. The
    // completed request is taken by ObdService afterwards.
    if (type == 0x10 || type == 0x20)
    {
        IsoTpReceive(frame, target);
        return;
    }

    if (type == 0x00)
        Enqueue(frame);
}

void CanWait(unsigned long ms)
{
    unsigned long start = millis();
    busyDepth++;

    while (millis() - start < ms)
    {
        SerialPump();

        CanFrame frame;

        if (CanReceive(frame))
            HandleFrameWhileBusy(frame);
    }

    busyDepth--;
}

// =============================================
// ISO-TP TRANSMIT
// =============================================

static unsigned long DecodeStMin(uint8_t stMin)
{
    if (stMin <= 0x7F)
        return stMin;

    if (stMin >= 0xF1 && stMin <= 0xF9)
        return 1;  // 100-900 us, rounded up to 1 ms

    return 0x7F;   // reserved values: use the maximum
}

static bool WaitForFlowControl(EcuIndex ecu, uint8_t &blockSize, unsigned long &stMinMs)
{
    unsigned long start = millis();
    uint8_t waitFrames = 0;
    unsigned long physical = PhysicalId(activeMode, ecu);

    while (millis() - start < ISOTP_FLOW_CONTROL_TIMEOUT_MS)
    {
        SerialPump();

        CanFrame frame;

        if (!CanReceive(frame))
            continue;

        if (frame.extended != activeMode->extended ||
            frame.id != physical ||
            (frame.data[0] & 0xF0) != 0x30)
        {
            HandleFrameWhileBusy(frame);
            continue;
        }

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

bool IsoTpSend(EcuIndex ecu, const uint8_t *payload, uint8_t length)
{
    uint8_t frame[8];

    if (length <= 7)
    {
        memset(frame, ISOTP_PADDING, sizeof(frame));
        frame[0] = length;
        memcpy(&frame[1], payload, length);
        return SendFrame(ecu, frame);
    }

    busyDepth++;

    // First Frame: 12-bit length, first 6 payload bytes.
    frame[0] = 0x10 | ((length >> 8) & 0x0F);
    frame[1] = length & 0xFF;
    memcpy(&frame[2], payload, 6);

    bool ok = SendFrame(ecu, frame);
    uint8_t offset = 6;
    uint8_t sequence = 1;

    while (ok && offset < length)
    {
        uint8_t blockSize = 0;
        unsigned long stMinMs = 0;

        if (!WaitForFlowControl(ecu, blockSize, stMinMs))
        {
            ok = false;
            break;
        }

        uint8_t sentInBlock = 0;

        while (ok && offset < length &&
               (blockSize == 0 || sentInBlock < blockSize))
        {
            if (sentInBlock > 0 && stMinMs > 0)
                CanWait(stMinMs);

            uint8_t chunk = length - offset;

            if (chunk > 7)
                chunk = 7;

            memset(frame, ISOTP_PADDING, sizeof(frame));
            frame[0] = 0x20 | (sequence & 0x0F);
            memcpy(&frame[1], payload + offset, chunk);

            ok = SendFrame(ecu, frame);

            offset += chunk;
            sequence++;
            sentInBlock++;
        }
    }

    busyDepth--;
    return ok;
}
