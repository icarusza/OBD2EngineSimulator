#include "ObdService.h"
#include "SerialCommands.h"
#include "UdsService.h"
#include <string.h>

// =============================================
// INTEGER HELPERS
// =============================================

static int32_t DivRound(int32_t numerator, int32_t denominator)
{
    return numerator >= 0
        ? (numerator + denominator / 2) / denominator
        : -((-numerator + denominator / 2) / denominator);
}

static uint8_t U8(int32_t value)
{
    return value <= 0 ? 0 : value >= 255 ? 255 : (uint8_t)value;
}

static uint16_t U16(int32_t value)
{
    return value <= 0 ? 0 : value >= 65535 ? 65535 : (uint16_t)value;
}

static void Put16(uint8_t *out, uint16_t value)
{
    out[0] = value >> 8;
    out[1] = value & 0xFF;
}

static void Put32(uint8_t *out, uint32_t value)
{
    Put16(out, value >> 16);
    Put16(out + 2, value & 0xFFFF);
}

// Percent with 1 decimal (0..1000) to A*100/255.
static uint8_t PercentRaw(int32_t percent10)
{
    return U8(DivRound(percent10 * 255, 1000));
}

// Fuel trim with 1 decimal to (A-128)*100/128.
static uint8_t TrimRaw(int32_t trim10)
{
    return U8(DivRound((trim10 + 1000) * 128, 1000));
}

// AFR with 1 decimal to lambda * 32768.
static uint16_t LambdaRaw(int32_t afr10)
{
    return U16(DivRound(afr10 * 32768, 147));
}

// =============================================
// SUPPORT BITMAPS
// =============================================
// One generator for every "supported IDs" bitmap (PIDs per ECU, freeze
// frame PIDs, MIDs, InfoTypes, Mode 08 TIDs): an ID 20/40/... is supported
// when anything above it is.

typedef bool (*SupportFunction)(uint8_t id, uint8_t context);

static bool RangeSupported(uint8_t range, SupportFunction supported, uint8_t context)
{
    if (range == 0)
        return true;

    for (uint16_t id = range + 1; id <= 0xFF; id++)
    {
        if ((id % 0x20) != 0 && supported((uint8_t)id, context))
            return true;
    }

    return false;
}

static bool IdSupported(uint8_t id, SupportFunction supported, uint8_t context)
{
    return id % 0x20 == 0 ? RangeSupported(id, supported, context) : supported(id, context);
}

// Writes the 4-byte bitmap for 'range' if that range is supported.
static bool EncodeBitmap(uint8_t range, SupportFunction supported, uint8_t context,
                         uint8_t *out, uint8_t &length)
{
    if (!RangeSupported(range, supported, context))
        return false;

    memset(out, 0, 4);

    for (uint8_t i = 0; i < 32; i++)
    {
        uint16_t id = range + 1 + i;

        if (id <= 0xFF && IdSupported((uint8_t)id, supported, context))
            out[i / 8] |= 0x80 >> (i % 8);
    }

    length = 4;
    return true;
}

// =============================================
// MODE 01 PIDS: ENGINE (ECM)
// =============================================

// PID 01/41 byte C/D, spark ignition: catalyst (bit 0), EVAP (2),
// O2 sensor (5), O2 heater (6), EGR (7).
const uint8_t SPARK_MONITORS = 0xE5;

static bool EncodeEcmPid(uint8_t pid, uint8_t *out, uint8_t &length)
{
    length = 1;

    switch (pid)
    {
        case 0x01:  // monitor status since DTCs cleared
            out[0] = (MilOn(ECU_ECM) ? 0x80 : 0x00) |
                     (CountDtcs(ECU_ECM, DTC_LIST_STORED) & 0x7F);
            out[1] = 0x07;  // spark ignition; misfire, fuel, components supported and complete
            out[2] = SPARK_MONITORS;  // catalyst, EVAP, O2 sensor, O2 heater, EGR supported
            out[3] = engine.p.readinessIncomplete ? SPARK_MONITORS : 0x00;
            length = 4;
            return true;

        case 0x03:  // fuel system status
            out[0] = U8(V(P_FUELSYS));
            out[1] = 0x00;
            length = 2;
            return true;

        case 0x04: out[0] = PercentRaw(V(P_LOAD)); return true;
        case 0x05: out[0] = U8(V(P_ECT) + 40); return true;
        case 0x06: out[0] = TrimRaw(V(P_STFT)); return true;
        case 0x07: out[0] = TrimRaw(V(P_LTFT)); return true;
#if DUAL_BANK
        case 0x08: out[0] = TrimRaw(V(P_STFT2)); return true;
        case 0x09: out[0] = TrimRaw(V(P_LTFT2)); return true;
#endif
        case 0x0B: out[0] = U8(V(P_MAP)); return true;

        case 0x0C:
            Put16(out, U16(V(P_RPM) * 4));
            length = 2;
            return true;

        case 0x0D: out[0] = U8(V(P_SPEED)); return true;
        case 0x0E: out[0] = U8(DivRound(V(P_TIMING) + 640, 5)); return true;  // (deg + 64) * 2
        case 0x0F: out[0] = U8(V(P_IAT) + 40); return true;

        case 0x10:
            Put16(out, U16(V(P_MAF) * 10));  // g/s * 100
            length = 2;
            return true;

        case 0x11: out[0] = PercentRaw(V(P_TPS)); return true;

        case 0x13:  // O2 sensor locations
            out[0] = DUAL_BANK ? 0x33 : 0x03;  // B1S1, B1S2 (+ B2S1, B2S2)
            return true;

        // Downstream narrowband sensors: about 0.65 V with a good catalyst;
        // they follow the upstream sensor when P0420 (catalyst) is active.
        case 0x15:
#if DUAL_BANK
        case 0x19:
#endif
        {
            int32_t millivolts = FaultActive(0x0420) ? V(P_O2V) : 650 + (V(P_O2V) - 450) / 10;
            out[0] = U8(millivolts / 5);
            out[1] = 0xFF;  // no trim from this sensor
            length = 2;
            return true;
        }

        case 0x1C: out[0] = OBD_STANDARD; return true;

        case 0x1F:  // run time since engine start (s)
            Put16(out, engine.engineRunning
                ? U16((millis() - engine.engineStartMs) / 1000)
                : 0);
            length = 2;
            return true;

        case 0x21:  // distance with MIL on (km)
            Put16(out, U16(engine.p.distanceMilOnM / 1000));
            length = 2;
            return true;

        case 0x23:  // fuel rail gauge pressure (10 kPa)
            Put16(out, U16(DivRound(V(P_RAILP), 10)));
            length = 2;
            return true;

        // Upstream wideband sensors: lambda + voltage.
        case 0x24:
#if DUAL_BANK
        case 0x28:
#endif
            Put16(out, LambdaRaw(V(P_MEASURED_AFR)));
            Put16(out + 2, U16(DivRound(V(P_O2V) * 8192, 1000)));  // mV -> V * 65536 / 8
            length = 4;
            return true;

        case 0x2F: out[0] = PercentRaw(V(P_FUEL)); return true;
        case 0x30: out[0] = engine.p.warmUpsSinceClear; return true;

        case 0x31:  // distance since codes cleared (km)
            Put16(out, U16(engine.p.distanceSinceClearM / 1000));
            length = 2;
            return true;

        case 0x33: out[0] = U8(V(P_BARO)); return true;

        case 0x41:  // monitor status this drive cycle
            out[0] = 0x00;
            out[1] = 0x07;
            out[2] = SPARK_MONITORS;
            out[3] = engine.cycleRunMs < CYCLE_MIN_RUN_S * 1000UL ? SPARK_MONITORS : 0x00;
            length = 4;
            return true;

        case 0x42:  // control module voltage (mV)
            Put16(out, U16(V(P_VBATT) * 10));
            length = 2;
            return true;

        case 0x43:  // absolute load (here equal to calculated load)
            Put16(out, U16(DivRound(V(P_LOAD) * 255, 1000)));
            length = 2;
            return true;

        case 0x44:  // commanded equivalence ratio
            Put16(out, U16(DivRound(147L * 32768, V(P_AFR))));
            length = 2;
            return true;

        case 0x45: out[0] = PercentRaw(max(0L, V(P_TPS) - 20)); return true;     // relative throttle
        case 0x46: out[0] = U8(V(P_AMBIENT) + 40); return true;
        case 0x47: out[0] = PercentRaw(min(1000L, V(P_TPS) + 40)); return true;  // throttle B

        case 0x49:  // accelerator pedal D
        case 0x4A:  // accelerator pedal E (second track, half range)
        {
            int32_t pedal = 150 + V(P_TPS) * 80 / 100;
            out[0] = PercentRaw(pid == 0x49 ? pedal : pedal / 2);
            return true;
        }

        case 0x4C: out[0] = PercentRaw(V(P_TPS)); return true;  // commanded throttle

        case 0x4D:  // time run with MIL on (min)
            Put16(out, U16(engine.p.secondsMilOn / 60));
            length = 2;
            return true;

        case 0x4E:  // time since codes cleared (min)
            Put16(out, U16(engine.p.secondsSinceClear / 60));
            length = 2;
            return true;

        case 0x51: out[0] = FUEL_TYPE; return true;
        case 0x52: out[0] = U8(DivRound(ETHANOL_PERCENT * 255L, 100)); return true;

        case 0x59:  // fuel rail absolute pressure (10 kPa)
            Put16(out, U16(DivRound(V(P_RAILP) + V(P_BARO), 10)));
            length = 2;
            return true;

        case 0x5C: out[0] = U8(V(P_OIL) + 40); return true;

        case 0x5E:  // fuel rate (0.05 L/h) = MAF / AFR * 3600 / 740 g/L
            Put16(out, U16(DivRound(V(P_MAF) * 72000L, V(P_MEASURED_AFR) * 740L)));
            length = 2;
            return true;

        case 0xA6:  // odometer (0.1 km)
            Put32(out, engine.p.odometerM / 100);
            length = 4;
            return true;

#if FEATURE_EXTRA_PIDS
        case 0x2C:  // commanded EGR: active at part load
        case 0x2D:  // EGR error: -25 % with P0401 (insufficient flow)
        {
            bool egrActive = V(P_LOAD) >= 200 && V(P_LOAD) <= 600 &&
                             V(P_RPM) >= 1200 && V(P_RPM) <= 3500;

            if (pid == 0x2C)
                out[0] = PercentRaw(egrActive ? 100 : 0);
            else
                out[0] = (egrActive && FaultActive(0x0401)) ? 96 : 128;

            return true;
        }

        case 0x2E:  // commanded EVAP purge: closed loop and warm
            out[0] = PercentRaw(V(P_FUELSYS) == 2 && V(P_ECT) >= 70 ? 150 : 0);
            return true;

        case 0x32:  // EVAP vapour pressure (0.25 Pa, signed)
            Put16(out, (uint16_t)(int16_t)(engine.evapPressurePa * 4));
            length = 2;
            return true;

        case 0x3C:  // catalyst temperature B1S1
        case 0x3E:  // B1S2
#if DUAL_BANK
        case 0x3D:  // B2S1
        case 0x3F:  // B2S2
#endif
        {
            int32_t temperature = engine.catalystTempC -
                ((pid == 0x3D || pid == 0x3F) ? 5 : 0) -
                ((pid == 0x3E || pid == 0x3F) ? 60 : 0);
            Put16(out, U16((temperature + 40) * 10));
            length = 2;
            return true;
        }

        case 0x61:  // driver's demand torque (A-125 %)
            out[0] = U8(V(P_LOAD) * 9 / 100 + 2 + 125);
            return true;

        case 0x62:  // actual torque (A-125 %)
            out[0] = U8(V(P_LOAD) * 9 / 100 + 125);
            return true;

        case 0x63:  // reference torque (Nm)
            Put16(out, REFERENCE_TORQUE_NM);
            length = 2;
            return true;

        // Temperature sensor PIDs: A bit 0/1 = sensor 1/2 supported,
        // B = sensor 1 - 40, C = sensor 2 - 40. Only sensor 1 is fitted.
        case 0x67:  // engine coolant temperature sensors
        case 0x68:  // intake air temperature sensors
            out[0] = 0x01;
            out[1] = U8((pid == 0x67 ? V(P_ECT) : V(P_IAT)) + 40);
            out[2] = 0x00;
            length = 3;
            return true;
#endif

        default:
            return false;
    }
}

// =============================================
// MODE 01 PIDS: TRANSMISSION (TCM)
// =============================================

#if FEATURE_TCM
// Gear ratios for PID A4 (x1000), index = gear.
static const uint16_t GEAR_RATIOS[9] PROGMEM =
{
    0, 4700, 3130, 2100, 1670, 1290, 1000, 840, 670
};

static bool EncodeTcmPid(uint8_t pid, uint8_t *out, uint8_t &length)
{
    length = 1;

    switch (pid)
    {
        case 0x01:
            out[0] = (MilOn(ECU_TCM) ? 0x80 : 0x00) |
                     (CountDtcs(ECU_TCM, DTC_LIST_STORED) & 0x7F);
            out[1] = 0x00;
            out[2] = 0x00;
            out[3] = 0x00;
            length = 4;
            return true;

        case 0x0D: out[0] = U8(V(P_SPEED)); return true;
        case 0x1C: out[0] = OBD_STANDARD; return true;

        case 0xA4:  // transmission actual gear: supported flag, gear, ratio (0.001)
        {
            uint8_t gear = U8(V(P_GEAR));

            if (gear > 8)
                gear = 8;

            out[0] = 0x02;
            out[1] = gear << 4;
            Put16(out + 2, pgm_read_word(&GEAR_RATIOS[gear]));
            length = 4;
            return true;
        }

        default:
            return false;
    }
}
#endif

static bool DataPidSupported(uint8_t pid, uint8_t ecu)
{
    uint8_t scratch[4];
    uint8_t length;

#if FEATURE_TCM
    if (ecu == ECU_TCM)
        return EncodeTcmPid(pid, scratch, length);
#else
    (void)ecu;
#endif

    return EncodeEcmPid(pid, scratch, length);
}

bool EncodePid(EcuIndex ecu, uint8_t pid, uint8_t *out, uint8_t &length)
{
    if (pid % 0x20 == 0)
        return EncodeBitmap(pid, DataPidSupported, ecu, out, length);

#if FEATURE_TCM
    if (ecu == ECU_TCM)
        return EncodeTcmPid(pid, out, length);
#endif

    return EncodeEcmPid(pid, out, length);
}

// =============================================
// FREEZE FRAMES
// =============================================
// A snapshot stores the encoded bytes of these PIDs, in this order.

struct FreezeFramePid
{
    uint8_t pid;
    uint8_t length;
};

static const FreezeFramePid FREEZE_FRAME_PIDS[] PROGMEM =
{
    { 0x03, 2 }, { 0x04, 1 }, { 0x05, 1 }, { 0x06, 1 }, { 0x07, 1 },
#if DUAL_BANK
    { 0x08, 1 }, { 0x09, 1 },
#endif
    { 0x0B, 1 }, { 0x0C, 2 }, { 0x0D, 1 }, { 0x0E, 1 }, { 0x0F, 1 },
    { 0x10, 2 }, { 0x11, 1 }, { 0x24, 4 }, { 0x2F, 1 }, { 0x33, 1 },
    { 0x42, 2 }, { 0x44, 2 }, { 0x46, 1 }, { 0x5C, 1 }
};

static const uint8_t FREEZE_FRAME_PID_COUNT =
    sizeof(FREEZE_FRAME_PIDS) / sizeof(FREEZE_FRAME_PIDS[0]);

void ObdCaptureSnapshot(uint8_t *data)
{
    uint8_t offset = 0;

    memset(data, 0, SNAPSHOT_DATA_SIZE);

    for (uint8_t i = 0; i < FREEZE_FRAME_PID_COUNT; i++)
    {
        uint8_t pid = pgm_read_byte(&FREEZE_FRAME_PIDS[i].pid);
        uint8_t expected = pgm_read_byte(&FREEZE_FRAME_PIDS[i].length);
        uint8_t length;

        if (offset + expected > SNAPSHOT_DATA_SIZE)
            break;

        EncodeEcmPid(pid, data + offset, length);
        offset += expected;
    }
}

bool SnapshotPidData(const Snapshot *snapshot, uint8_t pid,
                     const uint8_t *&data, uint8_t &length)
{
    if (snapshot == NULL)
        return false;

    uint8_t offset = 0;

    for (uint8_t i = 0; i < FREEZE_FRAME_PID_COUNT; i++)
    {
        uint8_t entryLength = pgm_read_byte(&FREEZE_FRAME_PIDS[i].length);

        if (pgm_read_byte(&FREEZE_FRAME_PIDS[i].pid) == pid)
        {
            data = snapshot->data + offset;
            length = entryLength;
            return true;
        }

        offset += entryLength;
    }

    return false;
}

static bool FreezeFramePidSupported(uint8_t pid, uint8_t)
{
    if (pid == 0x02)
        return true;

    const uint8_t *data;
    uint8_t length;
    return SnapshotPidData(PrimarySnapshot(), pid, data, length);
}

// =============================================
// MODE 06 MIDS
// =============================================

// One test record: MID, TID, UASID, value, minimum, maximum.
static uint8_t PutTest(uint8_t *out, uint8_t mid, uint8_t tid, uint8_t uasid,
                       uint16_t value, uint16_t minimum, uint16_t maximum)
{
    out[0] = mid;
    out[1] = tid;
    out[2] = uasid;
    Put16(out + 3, value);
    Put16(out + 5, minimum);
    Put16(out + 7, maximum);
    return 9;
}

#if FEATURE_MODE06_EXT
const uint8_t CYLINDERS = 6;

// Misfire counts per cylinder (0 = all): P0300 spreads misfires over all
// cylinders, P030n concentrates them on cylinder n.
static uint16_t MisfireCount(uint8_t cylinder)
{
    uint16_t total = 0;

    for (uint8_t c = 1; c <= CYLINDERS; c++)
    {
        if (cylinder != 0 && c != cylinder)
            continue;

        if (FaultActive(0x0300))
            total += 20;

        if (FaultActive(0x0300 + c))
            total += 120;
    }

    return total;
}
#endif

static bool MidSupported(uint8_t mid, uint8_t)
{
    uint8_t scratch[18];
    uint8_t length;
    return EncodeMid(mid, scratch, length);
}

bool EncodeMid(uint8_t mid, uint8_t *out, uint8_t &length)
{
    if (mid % 0x20 == 0)
        return EncodeBitmap(mid, MidSupported, 0, out, length);

    // UASID 0x0A: 0.122 mV per bit.
    const int32_t thresholdLow = DivRound(300L * 1000, 122);
    const int32_t thresholdHigh = DivRound(600L * 1000, 122);
    uint16_t o2 = U16(DivRound(V(P_O2V) * 1000, 122));

    switch (mid)
    {
        case 0x01:  // O2 sensor monitor B1S1: rich-to-lean and lean-to-rich thresholds
            length = PutTest(out, mid, 0x01, 0x0A, o2, thresholdLow, thresholdHigh);
            length += PutTest(out + length, mid, 0x02, 0x0A, o2, thresholdLow, thresholdHigh);
            return true;

#if FEATURE_MODE06_EXT
        case 0x21:  // catalyst monitor B1 (manufacturer TID, raw ratio x1000)
            length = PutTest(out, mid, 0x80, 0x01,
                             FaultActive(0x0420) ? 900 : 300, 0, 700);
            return true;

        case 0x31:  // EGR monitor B1 (manufacturer TID, raw flow)
            length = PutTest(out, mid, 0x80, 0x01,
                             FaultActive(0x0401) ? 100 : 600, 300, 0xFFFF);
            return true;

        case 0x3C:  // EVAP 0.020" leak (Mode 08 test); all zero until it has run
            length = PutTest(out, mid, 0x80, 0x01,
                             engine.p.evapLeakResult, 0,
                             engine.p.evapTestDone ? 20 : 0);
            return true;

        default:
            // A1 = misfire all cylinders, A2.. = cylinder 1..: EWMA counts
            // over 10 cycles (TID 0B) and counts this cycle (TID 0C).
            if (mid >= 0xA1 && mid <= 0xA1 + CYLINDERS)
            {
                uint16_t count = MisfireCount(mid - 0xA1);
                length = PutTest(out, mid, 0x0B, 0x24, count / 2, 0, 0xFFFF);
                length += PutTest(out + length, mid, 0x0C, 0x24, count, 0, 0xFFFF);
                return true;
            }

            return false;
#else
        default:
            return false;
#endif
    }
}

// =============================================
// MODE 09 INFOTYPES
// =============================================

static void PutText(uint8_t *out, const char *text, uint8_t size)
{
    uint8_t textLength = strlen(text);

    for (uint8_t i = 0; i < size; i++)
        out[i] = i < textLength ? (uint8_t)text[i] : 0x00;
}

// J1979 ECU name: bytes 1-4 acronym (0x00 padded), byte 5 '-', bytes 6-20
// name (0x00 padded). "ECM-EngineControl" is split at the first '-' when it
// comes within the first 5 characters; other text is sent as is.
static void PutEcuName(uint8_t *out, const char *text)
{
    const char *dash = strchr(text, '-');

    if (dash == NULL || dash - text > 4)
    {
        PutText(out, text, 20);
        return;
    }

    uint8_t acronymLength = dash - text;

    for (uint8_t i = 0; i < 4; i++)
        out[i] = i < acronymLength ? (uint8_t)text[i] : 0x00;

    out[4] = '-';
    PutText(&out[5], dash + 1, 15);
}

static uint32_t Crc32(const uint8_t *data, uint8_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;

    for (uint8_t i = 0; i < length; i++)
    {
        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8; bit++)
            crc = (crc >> 1) ^ (0xEDB88320UL & (0 - (crc & 1)));
    }

    return ~crc;
}

static bool InfoTypeSupported(uint8_t infoType, uint8_t ecu)
{
    uint8_t scratch[32];
    uint8_t length;
    uint8_t items;
    return EncodeInfoType((EcuIndex)ecu, infoType, scratch, length, items);
}

bool EncodeInfoType(EcuIndex ecu, uint8_t infoType, uint8_t *out,
                    uint8_t &length, uint8_t &items)
{
    items = 0;

    if (infoType % 0x20 == 0)
        return EncodeBitmap(infoType, InfoTypeSupported, ecu, out, length);

    items = 1;

    switch (infoType)
    {
        case 0x01:  // message counts (legacy tools): VIN, ECU name
        case 0x09:
            items = 0;
            out[0] = 0x01;
            length = 1;
            return true;

        case 0x02:
            PutText(out, engine.p.vin, 17);
            length = 17;
            return true;

        case 0x0A:
            PutEcuName(out, ecu == ECU_TCM ? TCM_ECU_NAME : engine.p.ecuName);
            length = 20;
            return true;
    }

    if (ecu != ECU_ECM)
        return false;

    switch (infoType)
    {
        case 0x03:  // calibration ID message count (legacy tools)
            items = 0;
            out[0] = 0x01;
            length = 1;
            return true;

        case 0x04:
            PutText(out, engine.p.calibrationId, 16);
            length = 16;
            return true;

        case 0x06:  // CVN: CRC32 of the calibration ID
        {
            uint8_t calibration[16];
            PutText(calibration, engine.p.calibrationId, 16);
            Put32(out, Crc32(calibration, 16));
            length = 4;
            return true;
        }

#if FEATURE_IPT
        case 0x08:  // in-use performance tracking, spark ignition: 16 counters
            for (uint8_t i = 0; i < IPT_COUNT; i++)
                Put16(out + 2 * i, engine.p.ipt[i]);

            items = IPT_COUNT;
            length = 2 * IPT_COUNT;
            return true;
#endif

        case 0x0D:  // engine serial number
            PutText(out, engine.p.esn, 16);
            length = 16;
            return true;

        default:
            return false;
    }
}

// =============================================
// MODE 08
// =============================================

#if FEATURE_MODE08
void StartEvapTest()
{
    engine.evapTestState = 1;
    engine.evapTestStartMs = millis();
    engine.evapPressurePa = 0;
}

static bool TestIdSupported(uint8_t tid, uint8_t)
{
    return tid == 0x01;
}
#endif

// =============================================
// SERVICES
// =============================================

static uint8_t Mode01(EcuIndex ecu, const uint8_t *params, uint8_t count,
                      uint8_t *response, uint8_t &length)
{
    response[0] = 0x41;
    length = 1;

    for (uint8_t i = 0; i < count && i < 6; i++)
    {
        uint8_t data[4];
        uint8_t dataLength;

        if (!EncodePid(ecu, params[i], data, dataLength))
            continue;

        if (length + 1 + dataLength > ISOTP_MAX_PAYLOAD)
            break;

        response[length++] = params[i];
        memcpy(&response[length], data, dataLength);
        length += dataLength;
    }

    return length > 1 ? RESULT_POSITIVE : NRC_SUBFUNCTION_NOT_SUPPORTED;
}

// Mode 02: freeze frame 0. Parameters are (PID, frame) pairs.
static uint8_t Mode02(const uint8_t *params, uint8_t count,
                      uint8_t *response, uint8_t &length)
{
    const Snapshot *snapshot = PrimarySnapshot();

    response[0] = 0x42;
    length = 1;

    for (uint8_t i = 0; i + 1 < count; i += 2)
    {
        uint8_t pid = params[i];
        uint8_t frame = params[i + 1];
        uint8_t buffer[4];
        const uint8_t *data = buffer;
        uint8_t dataLength;

        if (frame != 0)
            continue;

        if (pid % 0x20 == 0)
        {
            if (!EncodeBitmap(pid, FreezeFramePidSupported, 0, buffer, dataLength))
                continue;
        }
        else if (pid == 0x02)
        {
            Put16(buffer, snapshot != NULL ? snapshot->dtc : 0);
            dataLength = 2;
        }
        else if (!SnapshotPidData(snapshot, pid, data, dataLength))
        {
            continue;
        }

        if (length + 2 + dataLength > ISOTP_MAX_PAYLOAD)
            break;

        response[length++] = pid;
        response[length++] = frame;
        memcpy(&response[length], data, dataLength);
        length += dataLength;
    }

    return length > 1 ? RESULT_POSITIVE : NRC_SUBFUNCTION_NOT_SUPPORTED;
}

// Modes 03, 07, 0A: stored, pending and permanent DTCs of one ECU.
static uint8_t ReportDtcs(EcuIndex ecu, uint8_t service, DtcListType type,
                          uint8_t *response, uint8_t &length)
{
    response[0] = service + 0x40;
    response[1] = 0;
    length = 2;

    for (uint8_t i = 0; i < MAX_DTCS; i++)
    {
        const DtcRecord &record = engine.p.dtcs[i];
        EcuIndex owner = (record.flags & DTC_FLAG_TCM) ? ECU_TCM : ECU_ECM;

        if (!DtcInList(record, type) || owner != ecu)
            continue;

        Put16(&response[length], record.code);
        length += 2;
        response[1]++;
    }

    return RESULT_POSITIVE;
}

static uint8_t Mode04(EcuIndex ecu, uint8_t *response, uint8_t &length)
{
    // Real ECUs only clear with the engine off.
    if (EngineRunning())
        return NRC_CONDITIONS_NOT_CORRECT;

    ClearDiagnosticInformation(1 << ecu);
    Serial.println(F("EVENT DTC CLEARED BY TESTER"));

    response[0] = 0x44;
    length = 1;
    return RESULT_POSITIVE;
}

static uint8_t Mode06(const uint8_t *params, uint8_t count,
                      uint8_t *response, uint8_t &length)
{
    if (count < 1)
        return NRC_INCORRECT_LENGTH;

    response[0] = 0x46;
    length = 1;

    uint8_t dataLength;

    if (params[0] % 0x20 != 0)
    {
        // One test MID: its records already contain the MID.
        if (EncodeMid(params[0], &response[1], dataLength))
            length += dataLength;
    }
    else
    {
        // Several range MIDs (00, 20, ...) may be requested together.
        for (uint8_t i = 0; i < count && params[i] % 0x20 == 0; i++)
        {
            if (length + 5 > ISOTP_MAX_PAYLOAD)
                break;

            response[length] = params[i];

            if (EncodeMid(params[i], &response[length + 1], dataLength))
                length += 1 + dataLength;
        }
    }

    return length > 1 ? RESULT_POSITIVE : NRC_SUBFUNCTION_NOT_SUPPORTED;
}

#if FEATURE_MODE08
static uint8_t Mode08(const uint8_t *params, uint8_t count,
                      uint8_t *response, uint8_t &length)
{
    if (count < 1)
        return NRC_INCORRECT_LENGTH;

    uint8_t tid = params[0];

    response[0] = 0x48;
    response[1] = tid;

    if (tid % 0x20 == 0)
    {
        uint8_t dataLength;

        if (!EncodeBitmap(tid, TestIdSupported, 0, &response[2], dataLength))
            return NRC_SUBFUNCTION_NOT_SUPPORTED;

        length = 2 + dataLength;
        return RESULT_POSITIVE;
    }

    if (tid != 0x01)
        return NRC_SUBFUNCTION_NOT_SUPPORTED;

    // EVAP leak test: vehicle must be stationary.
    if (V(P_SPEED) > 0)
        return NRC_CONDITIONS_NOT_CORRECT;

    StartEvapTest();
    memset(&response[2], 0, 5);
    length = 7;
    return RESULT_POSITIVE;
}
#endif

static uint8_t Mode09(EcuIndex ecu, const uint8_t *params, uint8_t count,
                      uint8_t *response, uint8_t &length)
{
    if (count < 1)
        return NRC_INCORRECT_LENGTH;

    uint8_t infoType = params[0];
    uint8_t items;
    uint8_t dataLength;

    // Reserve the item-count byte; drop it again if not used.
    if (!EncodeInfoType(ecu, infoType, &response[3], dataLength, items))
        return NRC_SUBFUNCTION_NOT_SUPPORTED;

    response[0] = 0x49;
    response[1] = infoType;

    if (items > 0)
    {
        response[2] = items;
        length = 3 + dataLength;
    }
    else
    {
        memmove(&response[2], &response[3], dataLength);
        length = 2 + dataLength;
    }

    return RESULT_POSITIVE;
}

static uint8_t HandleService(EcuIndex ecu, const uint8_t *request, uint8_t requestLength,
                             uint8_t *response, uint8_t &length)
{
    uint8_t service = request[0];
    const uint8_t *params = request + 1;
    uint8_t count = requestLength - 1;

    switch (service)
    {
        case 0x01: return Mode01(ecu, params, count, response, length);
        case 0x02: return ecu == ECU_ECM ? Mode02(params, count, response, length)
                                         : NRC_SUBFUNCTION_NOT_SUPPORTED;
        case 0x03: return ReportDtcs(ecu, 0x03, DTC_LIST_STORED, response, length);
        case 0x04: return Mode04(ecu, response, length);
        case 0x06: return ecu == ECU_ECM ? Mode06(params, count, response, length)
                                         : NRC_SUBFUNCTION_NOT_SUPPORTED;
        case 0x07: return ReportDtcs(ecu, 0x07, DTC_LIST_PENDING, response, length);
#if FEATURE_MODE08
        case 0x08: return ecu == ECU_ECM ? Mode08(params, count, response, length)
                                         : NRC_SUBFUNCTION_NOT_SUPPORTED;
#endif
        case 0x09: return Mode09(ecu, params, count, response, length);
        case 0x0A: return ReportDtcs(ecu, 0x0A, DTC_LIST_PERMANENT, response, length);
    }

#if FEATURE_UDS
    if (UdsHandlesService(service))
        return UdsHandle(ecu, request, requestLength, response, length);
#endif

    return NRC_SERVICE_NOT_SUPPORTED;
}

// =============================================
// REQUEST DISPATCH
// =============================================

// Prints at most 12 bytes ("+N" for the rest) and only if the line fits
// in the TX buffer without blocking.
static void PrintBytes(const __FlashStringHelper *label, EcuIndex ecu,
                       const uint8_t *data, uint8_t length)
{
    const uint8_t maxShown = 12;
    uint8_t shown = length > maxShown ? maxShown : length;

    if (!LogLineFits(strlen_P((const char *)label) + 3 * shown + 12))
        return;

    Serial.print(label);
    Serial.print(ecu == ECU_TCM ? F(" TCM |") : F(" ECM |"));

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

static void SendNegative(EcuIndex ecu, uint8_t service, uint8_t code)
{
    uint8_t negative[3] = { 0x7F, service, code };

    if (engine.logLevel >= 1)
        PrintBytes(F("OBD RSP"), ecu, negative, 3);

    IsoTpSend(ecu, negative, 3);
}

// A response prepared by one ECU before it is sent.
struct PreparedResponse
{
    bool send;
    uint8_t result;
    uint8_t length;
    uint8_t data[ISOTP_MAX_PAYLOAD];
};

static bool EcuPresent(uint8_t ecu)
{
#if FEATURE_TCM
    return ecu == ECU_ECM || (ecu == ECU_TCM && engine.p.tcmEnabled);
#else
    return ecu == ECU_ECM;
#endif
}

// Handles the request for one ECU and decides whether it answers.
static void Prepare(EcuIndex ecu, bool physical, const uint8_t *request, uint8_t requestLength,
                    PreparedResponse &prepared)
{
    if (engine.logLevel >= 1)
        PrintBytes(physical ? F("OBD REQ PHYS") : F("OBD REQ FUNC"), ecu, request, requestLength);

    prepared.length = 0;
    prepared.result = HandleService(ecu, request, requestLength, prepared.data, prepared.length);

    // ISO 15765-4 / ISO 14229-2: no negative response to functional requests
    // for unsupported services, subfunctions or out-of-range data.
    bool suppressed = !physical &&
        (prepared.result == NRC_SERVICE_NOT_SUPPORTED ||
         prepared.result == NRC_SUBFUNCTION_NOT_SUPPORTED ||
         prepared.result == NRC_REQUEST_OUT_OF_RANGE);

    prepared.send = prepared.result != RESULT_SILENT && !suppressed;
}

// Waits for the simulated response delay. Longer than P2 (or a clear, which
// real ECUs often answer late) sends "response pending" from every answering
// ECU first, repeated within P2*. All ECUs share one wait, so on a
// functional request the second ECU is not delayed past P2 by the first.
static void SimulateProcessingTime(const PreparedResponse *prepared, uint8_t service)
{
    bool isClear = service == 0x04 || service == 0x14;
    unsigned long remaining = engine.responseDelayMs;

    if (remaining <= RESPONSE_PENDING_AFTER_MS && !isClear)
    {
        if (remaining > 0)
            CanWait(remaining);

        return;
    }

    while (true)
    {
        for (uint8_t ecu = 0; ecu < ECU_COUNT; ecu++)
        {
            if (prepared[ecu].send)
                SendNegative((EcuIndex)ecu, service, NRC_RESPONSE_PENDING);
        }

        unsigned long chunk = remaining > RESPONSE_PENDING_REPEAT_MS
            ? RESPONSE_PENDING_REPEAT_MS : remaining;

        if (chunk > 0)
            CanWait(chunk);

        remaining -= chunk;

        if (remaining == 0)
            return;
    }
}

// target: one ECU (physical request) or TARGET_FUNCTIONAL (every ECU,
// engine first).
static void DispatchToTarget(uint8_t target, const uint8_t *request, uint8_t requestLength)
{
    bool physical = target != TARGET_FUNCTIONAL;
    PreparedResponse prepared[ECU_COUNT];
    bool anySend = false;

    for (uint8_t ecu = 0; ecu < ECU_COUNT; ecu++)
    {
        prepared[ecu].send = false;

        if ((physical && ecu != target) || !EcuPresent(ecu))
            continue;

        Prepare((EcuIndex)ecu, physical, request, requestLength, prepared[ecu]);
        anySend |= prepared[ecu].send;
    }

    if (!anySend)
        return;

    SimulateProcessingTime(prepared, request[0]);

    for (uint8_t ecu = 0; ecu < ECU_COUNT; ecu++)
    {
        PreparedResponse &response = prepared[ecu];

        if (!response.send)
            continue;

        if (response.result != RESULT_POSITIVE)
        {
            SendNegative((EcuIndex)ecu, request[0], response.result);
            continue;
        }

        if (engine.logLevel >= 1)
            PrintBytes(F("OBD RSP"), (EcuIndex)ecu, response.data, response.length);

        IsoTpSend((EcuIndex)ecu, response.data, response.length);
    }
}

void ObdService()
{
    uint8_t buffer[ISOTP_RX_BUFFER];
    uint8_t length;
    EcuIndex ecu;

    if (IsoTpTakeRequest(ecu, buffer, length))
    {
        if (engine.ignitionOn)
            DispatchToTarget(ecu, buffer, length);
    }

    CanFrame frame;

    while (CanNextFrame(frame))
    {
        uint8_t target = CanRequestTarget(frame);

        if (target == TARGET_NONE || !engine.ignitionOn)
            continue;

        if (IsoTpReceive(frame, target))
        {
            if (IsoTpTakeRequest(ecu, buffer, length))
                DispatchToTarget(ecu, buffer, length);

            continue;
        }

        uint8_t pci = frame.data[0];

        if ((pci & 0xF0) != 0)
            continue;  // Flow Control or other non-request frame

        uint8_t requestLength = pci & 0x0F;

        if (requestLength < 1 || requestLength > 7 || requestLength + 1 > frame.length)
            continue;

        DispatchToTarget(target, &frame.data[1], requestLength);
    }
}
