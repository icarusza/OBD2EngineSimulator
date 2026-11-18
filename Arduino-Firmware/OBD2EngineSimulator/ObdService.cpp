#include "ObdService.h"
#include "EngineState.h"
#include "SerialCommands.h"
#include <math.h>
#include <string.h>

// =============================================
// ENCODING HELPERS
// =============================================

static uint8_t U8(float value)
{
    if (!(value > 0)) return 0;  // also catches NaN
    if (value >= 255) return 255;
    return (uint8_t)round(value);
}

static uint16_t U16(float value)
{
    if (!(value > 0)) return 0;
    if (value >= 65535) return 65535;
    return (uint16_t)round(value);
}

static void Put16(uint8_t *out, uint16_t value)
{
    out[0] = highByte(value);
    out[1] = lowByte(value);
}

static bool EncodePid(uint8_t pid, const float *v, bool freezeFrame,
                      uint8_t *out, uint8_t &length);

// =============================================
// SUPPORTED-PID BITMAPS
// =============================================
// Bitmaps (PIDs 00, 20, 40, 60) are generated from EncodePid(), so adding a
// case there is all that is needed to advertise a new PID.

static bool IsSupported(uint8_t pid, bool freezeFrame);

static bool HasSupportedAbove(uint8_t rangePid, bool freezeFrame)
{
    for (uint16_t pid = rangePid + 1; pid <= 0xFF; pid++)
    {
        if ((pid % 0x20) != 0 && IsSupported((uint8_t)pid, freezeFrame))
            return true;
    }

    return false;
}

static bool IsSupported(uint8_t pid, bool freezeFrame)
{
    if (pid % 0x20 == 0)
        return pid == 0 || HasSupportedAbove(pid, freezeFrame);

    uint8_t scratch[4];
    uint8_t length;

    return EncodePid(pid, engine.values, freezeFrame, scratch, length);
}

static void EncodeBitmap(uint8_t basePid, bool freezeFrame, uint8_t *out)
{
    memset(out, 0, 4);

    for (uint8_t i = 0; i < 32; i++)
    {
        uint16_t pid = basePid + 1 + i;

        if (pid <= 0xFF && IsSupported((uint8_t)pid, freezeFrame))
            out[i / 8] |= 0x80 >> (i % 8);
    }
}

// =============================================
// PID ENCODERS (Mode 01 and Mode 02)
// =============================================
// v: live values (Mode 01) or freeze-frame snapshot (Mode 02).
// Returns false if the PID is not supported in that mode.

#define LIVE_ONLY() if (freezeFrame) return false

static bool EncodePid(uint8_t pid, const float *v, bool freezeFrame,
                      uint8_t *out, uint8_t &length)
{
    if (pid % 0x20 == 0)
    {
        if (!IsSupported(pid, freezeFrame))
            return false;

        EncodeBitmap(pid, freezeFrame, out);
        length = 4;
        return true;
    }

    // Mode 02 only answers PID 02 until a freeze frame has been captured.
    if (freezeFrame && !engine.freezeFrame.valid && pid != 0x02)
        return false;

    length = 1;

    switch (pid)
    {
        // MONITOR STATUS SINCE DTCS CLEARED
        case 0x01:
            LIVE_ONLY();
            out[0] = (MilOn() ? 0x80 : 0x00) |
                     (engine.dtcs[DTC_STORED].count & 0x7F);
            out[1] = 0x07;  // spark ignition; misfire, fuel, components supported and complete
            out[2] = 0x65;  // catalyst, EVAP, O2 sensor, O2 heater supported
            // After a clear, these monitors report "not complete" until the
            // engine has run 5 minutes, like a real ECU re-running its tests.
            out[3] = (engine.codesClearedSinceBoot && engine.secondsSinceClear < 300)
                ? 0x65 : 0x00;
            length = 4;
            return true;

        // DTC THAT CAUSED THE FREEZE FRAME (Mode 02 only)
        case 0x02:
            if (!freezeFrame)
                return false;
            Put16(out, engine.freezeFrame.valid ? engine.freezeFrame.dtc : 0);
            length = 2;
            return true;

        // FUEL SYSTEM STATUS
        case 0x03:
            out[0] = U8(v[P_FUELSYS]);
            out[1] = 0x00;  // no bank 2
            length = 2;
            return true;

        // ENGINE LOAD
        case 0x04:
            out[0] = U8(v[P_LOAD] * 255.0 / 100.0);
            return true;

        // COOLANT TEMPERATURE
        case 0x05:
            out[0] = U8(v[P_ECT] + 40);
            return true;

        // SHORT TERM FUEL TRIM
        case 0x06:
            out[0] = U8((v[P_STFT] + 100.0) * 128.0 / 100.0);
            return true;

        // LONG TERM FUEL TRIM
        case 0x07:
            out[0] = U8((v[P_LTFT] + 100.0) * 128.0 / 100.0);
            return true;

        // MANIFOLD PRESSURE
        case 0x0B:
            out[0] = U8(v[P_MAP]);
            return true;

        // ENGINE RPM
        case 0x0C:
            Put16(out, U16(v[P_RPM] * 4.0));
            length = 2;
            return true;

        // VEHICLE SPEED
        case 0x0D:
            out[0] = U8(v[P_SPEED]);
            return true;

        // IGNITION TIMING
        case 0x0E:
            out[0] = U8((v[P_TIMING] + 64.0) * 2.0);
            return true;

        // INTAKE AIR TEMPERATURE
        case 0x0F:
            out[0] = U8(v[P_IAT] + 40);
            return true;

        // MASS AIR FLOW
        case 0x10:
            Put16(out, U16(v[P_MAF] * 100.0));
            length = 2;
            return true;

        // THROTTLE POSITION
        case 0x11:
            out[0] = U8(v[P_TPS] * 255.0 / 100.0);
            return true;

        // OXYGEN SENSOR LOCATIONS (bank 1 sensor 1)
        case 0x13:
            LIVE_ONLY();
            out[0] = 0x01;
            return true;

        // OBD STANDARD
        case 0x1C:
            LIVE_ONLY();
            out[0] = OBD_STANDARD;
            return true;

        // RUN TIME SINCE ENGINE START (s)
        case 0x1F:
            LIVE_ONLY();
            Put16(out, engine.engineRunning
                ? U16((millis() - engine.engineStartMs) / 1000.0)
                : 0);
            length = 2;
            return true;

        // DISTANCE TRAVELED WITH MIL ON (km)
        case 0x21:
            LIVE_ONLY();
            Put16(out, U16(engine.distanceMilOnKm));
            length = 2;
            return true;

        // WIDE-RANGE OXYGEN SENSOR 1: LAMBDA + VOLTAGE
        case 0x24:
        {
            float lambda = v[P_MEASURED_AFR] / 14.7;
            Put16(out, U16(lambda * 65535.0 / 2.0));
            Put16(out + 2, U16(v[P_O2V] * 65535.0 / 8.0));
            length = 4;
            return true;
        }

        // FUEL TANK LEVEL
        case 0x2F:
            out[0] = U8(v[P_FUEL] * 255.0 / 100.0);
            return true;

        // WARM-UPS SINCE CODES CLEARED
        case 0x30:
            LIVE_ONLY();
            out[0] = engine.warmUpsSinceClear;
            return true;

        // DISTANCE SINCE CODES CLEARED (km)
        case 0x31:
            LIVE_ONLY();
            Put16(out, U16(engine.distanceSinceClearKm));
            length = 2;
            return true;

        // BAROMETRIC PRESSURE
        case 0x33:
            out[0] = U8(v[P_BARO]);
            return true;

        // CONTROL MODULE VOLTAGE
        case 0x42:
            Put16(out, U16(v[P_VBATT] * 1000.0));
            length = 2;
            return true;

        // COMMANDED EQUIVALENCE RATIO
        case 0x44:
            Put16(out, U16(14.7 / v[P_AFR] * 32768.0));
            length = 2;
            return true;

        // AMBIENT AIR TEMPERATURE
        case 0x46:
            out[0] = U8(v[P_AMBIENT] + 40);
            return true;

        // TIME RUN WITH MIL ON (min)
        case 0x4D:
            LIVE_ONLY();
            Put16(out, U16(engine.secondsMilOn / 60.0));
            length = 2;
            return true;

        // TIME SINCE TROUBLE CODES CLEARED (min)
        case 0x4E:
            LIVE_ONLY();
            Put16(out, U16(engine.secondsSinceClear / 60.0));
            length = 2;
            return true;

        // ENGINE OIL TEMPERATURE
        case 0x5C:
            out[0] = U8(v[P_OIL] + 40);
            return true;

        default:
            return false;
    }
}

// =============================================
// SERVICES
// =============================================

// Mode 01: current data. Up to 6 PIDs per request.
static bool Mode01(const uint8_t *params, uint8_t count,
                   uint8_t *response, uint8_t &length)
{
    response[0] = 0x41;
    length = 1;

    for (uint8_t i = 0; i < count && i < 6; i++)
    {
        uint8_t data[4];
        uint8_t dataLength;

        if (!EncodePid(params[i], engine.values, false, data, dataLength))
            continue;

        if (length + 1 + dataLength > ISOTP_MAX_PAYLOAD)
            break;

        response[length++] = params[i];
        memcpy(&response[length], data, dataLength);
        length += dataLength;
    }

    return length > 1;
}

// Mode 02: freeze frame data. Parameters are (PID, frame number) pairs.
static bool Mode02(const uint8_t *params, uint8_t count,
                   uint8_t *response, uint8_t &length)
{
    response[0] = 0x42;
    length = 1;

    for (uint8_t i = 0; i + 1 < count; i += 2)
    {
        uint8_t pid = params[i];
        uint8_t frameNumber = params[i + 1];

        if (frameNumber != 0)
            continue;  // only frame 0 is stored

        uint8_t data[4];
        uint8_t dataLength;

        if (!EncodePid(pid, engine.freezeFrame.values, true, data, dataLength))
            continue;

        if (length + 2 + dataLength > ISOTP_MAX_PAYLOAD)
            break;

        response[length++] = pid;
        response[length++] = frameNumber;
        memcpy(&response[length], data, dataLength);
        length += dataLength;
    }

    return length > 1;
}

// Modes 03, 07, 0A: stored, pending and permanent DTCs.
static bool ReportDtcs(uint8_t service, DtcListType type,
                       uint8_t *response, uint8_t &length)
{
    const DtcList &list = engine.dtcs[type];

    response[0] = service + 0x40;
    response[1] = list.count;
    length = 2;

    for (uint8_t i = 0; i < list.count; i++)
    {
        Put16(&response[length], list.codes[i]);
        length += 2;
    }

    return true;
}

// Mode 06: on-board monitoring test results.
static void PutTestRecord(uint8_t *out, uint8_t mid, uint8_t tid,
                          float value, float minimum, float maximum)
{
    const float voltsPerBit = 0.000122;  // UASID 0x0A

    out[0] = mid;
    out[1] = tid;
    out[2] = 0x0A;
    Put16(out + 3, U16(value / voltsPerBit));
    Put16(out + 5, U16(minimum / voltsPerBit));
    Put16(out + 7, U16(maximum / voltsPerBit));
}

static bool Mode06(const uint8_t *params, uint8_t count,
                   uint8_t *response, uint8_t &length)
{
    if (count < 1)
        return false;

    uint8_t mid = params[0];

    response[0] = 0x46;

    if (mid == 0x00)
    {
        response[1] = 0x00;
        response[2] = 0x80;  // MID 01 supported
        response[3] = 0x00;
        response[4] = 0x00;
        response[5] = 0x00;
        length = 6;
        return true;
    }

    if (mid == 0x01)  // O2 sensor monitor bank 1 sensor 1
    {
        float o2v = engine.values[P_O2V];

        PutTestRecord(&response[1], 0x01, 0x01, o2v, 0.3, 0.6);   // rich-to-lean threshold
        PutTestRecord(&response[10], 0x01, 0x02, o2v, 0.3, 0.6);  // lean-to-rich threshold
        length = 19;
        return true;
    }

    return false;
}

// Mode 09: vehicle information.
static uint8_t PutText(uint8_t *out, const char *text, uint8_t size)
{
    uint8_t textLength = strlen(text);

    for (uint8_t i = 0; i < size; i++)
        out[i] = i < textLength ? (uint8_t)text[i] : 0x00;

    return size;
}

// J1979 ECU name: bytes 1-4 acronym (0x00 padded), byte 5 '-', bytes 6-20
// name (0x00 padded). "ECM-EngineControl" is split at the first '-' when it
// comes within the first 5 characters; other text is sent as is.
static uint8_t PutEcuName(uint8_t *out, const char *text)
{
    const char *dash = strchr(text, '-');

    if (dash == NULL || dash - text > 4)
        return PutText(out, text, 20);

    uint8_t acronymLength = dash - text;

    for (uint8_t i = 0; i < 4; i++)
        out[i] = i < acronymLength ? (uint8_t)text[i] : 0x00;

    out[4] = '-';
    PutText(&out[5], dash + 1, 15);
    return 20;
}

static bool Mode09(const uint8_t *params, uint8_t count,
                   uint8_t *response, uint8_t &length)
{
    if (count < 1)
        return false;

    uint8_t infoType = params[0];

    response[0] = 0x49;
    response[1] = infoType;
    length = 2;

    switch (infoType)
    {
        case 0x00:  // supported InfoTypes 01-20: 01, 02, 03, 04, 09, 0A
            response[2] = 0xF0;
            response[3] = 0xC0;
            response[4] = 0x00;
            response[5] = 0x00;
            length = 6;
            return true;

        case 0x01:  // VIN message count
        case 0x03:  // calibration ID message count
        case 0x09:  // ECU name message count
            response[2] = 0x01;
            length = 3;
            return true;

        case 0x02:  // VIN
            response[2] = 0x01;
            length = 3 + PutText(&response[3], engine.vin, 17);
            return true;

        case 0x04:  // calibration ID
            response[2] = 0x01;
            length = 3 + PutText(&response[3], engine.calibrationId, 16);
            return true;

        case 0x0A:  // ECU name
            response[2] = 0x01;
            length = 3 + PutEcuName(&response[3], engine.ecuName);
            return true;

        default:
            return false;
    }
}

// =============================================
// REQUEST DISPATCH
// =============================================

// Prints at most 12 bytes ("... +N" for the rest) and only if the line fits
// in the TX buffer without blocking.
static void PrintBytes(const __FlashStringHelper *label, const uint8_t *data, uint8_t length)
{
    const uint8_t maxShown = 12;
    uint8_t shown = length > maxShown ? maxShown : length;

    if (!LogLineFits(strlen_P((const char *)label) + 3 * shown + 8))
        return;

    Serial.print(label);

    for (uint8_t i = 0; i < shown; i++)
    {
        Serial.print(' ');

        if (data[i] < 0x10)
            Serial.print('0');

        Serial.print(data[i], HEX);
    }

    if (length > shown)
    {
        Serial.print(F(" +"));
        Serial.print(length - shown);
    }

    Serial.println();
}

static void HandleRequest(const CanFrame &frame, bool physical)
{
    uint8_t pci = frame.data[0];

    if ((pci & 0xF0) != 0)
        return;  // only single-frame requests are used by OBD

    uint8_t requestLength = pci & 0x0F;

    if (requestLength < 1 || requestLength > 7 ||
        requestLength + 1 > frame.length)
        return;

    const uint8_t *request = &frame.data[1];
    uint8_t service = request[0];
    const uint8_t *params = request + 1;
    uint8_t paramCount = requestLength - 1;

    if (engine.logLevel >= 1)
        PrintBytes(physical ? F("OBD REQ PHYS |") : F("OBD REQ FUNC |"),
                   request, requestLength);

    uint8_t response[ISOTP_MAX_PAYLOAD];
    uint8_t length = 0;
    bool handled = false;
    bool knownService = true;

    switch (service)
    {
        case 0x01: handled = Mode01(params, paramCount, response, length); break;
        case 0x02: handled = Mode02(params, paramCount, response, length); break;
        case 0x03: handled = ReportDtcs(0x03, DTC_STORED, response, length); break;
        case 0x07: handled = ReportDtcs(0x07, DTC_PENDING, response, length); break;
        case 0x0A: handled = ReportDtcs(0x0A, DTC_PERMANENT, response, length); break;
        case 0x06: handled = Mode06(params, paramCount, response, length); break;
        case 0x09: handled = Mode09(params, paramCount, response, length); break;

        case 0x04:
            ClearDiagnosticInformation();
            response[0] = 0x44;
            length = 1;
            handled = true;
            Serial.println(F("EVENT DTC CLEARED BY TESTER"));
            break;

        default:
            knownService = false;
            break;
    }

    if (!handled)
    {
        // ISO 15765-4: no reply to unsupported functional requests.
        if (!physical)
            return;

        response[0] = 0x7F;
        response[1] = service;
        // 0x11 service not supported; 0x12 PID / InfoType not supported
        response[2] = knownService ? 0x12 : 0x11;
        length = 3;
    }

    if (engine.logLevel >= 1)
        PrintBytes(F("OBD RSP |"), response, length);

    IsoTpSend(response, length);
}

void ObdService()
{
    CanFrame frame;

    while (CanReceive(frame))
    {
        bool physical;

        if (!CanIsRequest(frame, physical))
            continue;

        if (!engine.ignitionOn)
            continue;

        HandleRequest(frame, physical);
    }
}
