#include "EngineState.h"
#include "Persist.h"
#include "ObdService.h"
#include <string.h>

EngineState engine;

// =============================================
// PARAMETER DEFINITIONS
// =============================================
// name, minimum, maximum, default (all scaled by 10^decimals), decimals.
// Decimals must match DecimalPlaces in the Windows app
// (ParameterDefinitions.cs) so echoed values compare exactly.

const ParamDef PARAM_DEFS[PARAM_COUNT] PROGMEM =
{
    { "RPM",          0,     12000, 850,  0 },
    { "MAP",          0,     170,   85,   0 },
    { "STFT",         -1000, 992,   31,   1 },
    { "LTFT",         -1000, 992,   -23,  1 },
    { "AFR",          80,    200,   147,  1 },
    { "MEASURED_AFR", 80,    200,   147,  1 },
    { "ECT",          -40,   215,   90,   0 },
    { "IAT",          -40,   215,   25,   0 },
    { "TPS",          0,     1000,  0,    1 },
    { "SPEED",        0,     255,   0,    0 },
    { "LOAD",         0,     1000,  200,  1 },
    { "MAF",          0,     2540,  35,   1 },
    { "TIMING",       -640,  635,   100,  1 },
    { "FUEL",         0,     1000,  750,  1 },
    { "BARO",         0,     255,   101,  0 },
    { "VBATT",        0,     6550,  1420, 2 },
    { "AMBIENT",      -40,   215,   20,   0 },
    { "OIL",          -40,   215,   90,   0 },
    { "O2V",          0,     5000,  450,  3 },
    { "FUELSYS",      0,     16,    2,    0 },
    { "STFT2",        -1000, 992,   25,   1 },
    { "LTFT2",        -1000, 992,   -18,  1 },
    { "RAILP",        0,     60000, 5000, 0 },
    { "GEAR",         0,     8,     0,    0 }
};

void ReadParamDef(uint8_t index, ParamDef &out)
{
    memcpy_P(&out, &PARAM_DEFS[index], sizeof(ParamDef));
}

int8_t FindParam(const char *name)
{
    for (uint8_t i = 0; i < PARAM_COUNT; i++)
    {
        if (strcmp_P(name, PARAM_DEFS[i].name) == 0)
            return (int8_t)i;
    }

    return -1;
}

// =============================================
// DTC TABLE HELPERS
// =============================================

static DtcRecord *FindRecord(uint16_t code)
{
    for (uint8_t i = 0; i < MAX_DTCS; i++)
    {
        if (engine.p.dtcs[i].code == code)
            return &engine.p.dtcs[i];
    }

    return NULL;
}

static DtcRecord *FindOrCreateRecord(uint16_t code)
{
    DtcRecord *record = FindRecord(code);

    if (record != NULL)
        return record;

    record = FindRecord(0);  // free slot

    if (record == NULL)
        return NULL;

    memset(record, 0, sizeof(DtcRecord));
    record->code = code;
    record->status = DTC_NOT_COMPLETED_THIS_CYCLE;

    if (DtcOwner(code) == ECU_TCM)
        record->flags = DTC_FLAG_TCM;

    return record;
}

static EcuIndex RecordOwner(const DtcRecord &record)
{
    return (record.flags & DTC_FLAG_TCM) ? ECU_TCM : ECU_ECM;
}

EcuIndex DtcOwner(uint16_t code)
{
#if FEATURE_TCM
    // Powertrain codes P0700-P0999 are transmission codes.
    uint16_t number = code & 0x3FFF;

    if ((code >> 14) == 0 && number >= 0x0700 && number <= 0x0999)
        return ECU_TCM;
#else
    (void)code;
#endif

    return ECU_ECM;
}

// Misfire (P0300-P030F) and fuel system (P0171-P0175) codes have freeze
// frame priority (SAE J1979).
static bool IsPriorityCode(uint16_t code)
{
    return (code & 0xFFF0) == 0x0300 || (code >= 0x0171 && code <= 0x0175);
}

// =============================================
// FREEZE FRAMES
// =============================================

const Snapshot *FindSnapshot(uint16_t code)
{
    for (uint8_t i = 0; i < MAX_SNAPSHOTS; i++)
    {
        if (engine.p.snapshots[i].dtc == code && code != 0)
            return &engine.p.snapshots[i];
    }

    return NULL;
}

static void FreeSnapshot(uint16_t code)
{
    for (uint8_t i = 0; i < MAX_SNAPSHOTS; i++)
    {
        if (engine.p.snapshots[i].dtc == code)
            engine.p.snapshots[i].dtc = 0;
    }
}

static void CaptureSnapshot(uint16_t code)
{
    if (DtcOwner(code) != ECU_ECM || FindSnapshot(code) != NULL)
        return;

    Snapshot *slot = NULL;

    for (uint8_t i = 0; i < MAX_SNAPSHOTS && slot == NULL; i++)
    {
        if (engine.p.snapshots[i].dtc == 0)
            slot = &engine.p.snapshots[i];
    }

    // Full: a priority code may replace a non-priority one.
    if (slot == NULL && IsPriorityCode(code))
    {
        for (uint8_t i = 0; i < MAX_SNAPSHOTS && slot == NULL; i++)
        {
            if (!IsPriorityCode(engine.p.snapshots[i].dtc))
                slot = &engine.p.snapshots[i];
        }
    }

    if (slot == NULL)
        return;

    slot->dtc = code;
    slot->order = ++engine.p.snapshotCounter;
    ObdCaptureSnapshot(slot->data);
}

const Snapshot *PrimarySnapshot()
{
    const Snapshot *best = NULL;

    for (uint8_t i = 0; i < MAX_SNAPSHOTS; i++)
    {
        const Snapshot *s = &engine.p.snapshots[i];

        if (s->dtc == 0)
            continue;

        if (best == NULL)
        {
            best = s;
            continue;
        }

        bool sPriority = IsPriorityCode(s->dtc);
        bool bestPriority = IsPriorityCode(best->dtc);

        if ((sPriority && !bestPriority) ||
            (sPriority == bestPriority && (int8_t)(s->order - best->order) < 0))  // wrap-safe "older"
            best = s;
    }

    return best;
}

static void EraseIfEmpty(DtcRecord &record)
{
    bool alive = (record.status & (DTC_TEST_FAILED | DTC_PENDING | DTC_CONFIRMED)) != 0 ||
                 (record.flags & DTC_FLAG_PERMANENT) != 0;

    if (alive)
        return;

    FreeSnapshot(record.code);
    memset(&record, 0, sizeof(DtcRecord));
}

// =============================================
// DTC LIFECYCLE
// =============================================

bool ReportFault(uint16_t code, uint8_t severity)
{
    DtcRecord *record = severity > 0 ? FindOrCreateRecord(code) : FindRecord(code);

    if (record == NULL)
        return severity == 0;  // passing an unknown code is fine; failing needs a slot

    uint8_t oldStatus = record->status;
    uint8_t oldFlags = record->flags;

    record->status &= ~(DTC_NOT_COMPLETED_THIS_CYCLE | DTC_NOT_COMPLETED_SINCE_CLEAR);

    if (severity > 0)
    {
        bool wasPending = (record->status & DTC_PENDING) != 0;

        record->status |= DTC_TEST_FAILED | DTC_FAILED_THIS_CYCLE |
                          DTC_PENDING | DTC_FAILED_SINCE_CLEAR;

        if (severity >= 2)
            record->flags |= DTC_FLAG_SEVERE;
        else
            record->flags &= ~DTC_FLAG_SEVERE;

        if (!wasPending)
            CaptureSnapshot(code);
    }
    else
    {
        record->status &= ~DTC_TEST_FAILED;
        record->flags &= ~DTC_FLAG_SEVERE;
        EraseIfEmpty(*record);
    }

    // The app repeats fault reports every 2 s: only real changes are saved,
    // otherwise each repeat would rewrite the EEPROM.
    if (record->code != code || record->status != oldStatus || record->flags != oldFlags)
        PersistMarkDirty(true);

    return true;
}

static void StartDriveCycle()
{
    engine.cycleRunMs = 0;
    engine.cycleOpen = true;

    for (uint8_t i = 0; i < MAX_DTCS; i++)
    {
        DtcRecord &record = engine.p.dtcs[i];

        if (record.code == 0)
            continue;

        record.status &= ~DTC_FAILED_THIS_CYCLE;
        record.status |= DTC_NOT_COMPLETED_THIS_CYCLE;

        // A fault that is still present fails again in the new cycle.
        if (record.status & DTC_TEST_FAILED)
        {
            record.status |= DTC_FAILED_THIS_CYCLE | DTC_PENDING;
            record.status &= ~DTC_NOT_COMPLETED_THIS_CYCLE;
        }
    }
}

bool EndDriveCycle()
{
    // A cycle is counted once: not again after the ignition already ended it.
    if (!engine.cycleOpen)
        return false;

    engine.cycleOpen = false;

    for (uint8_t i = 0; i < MAX_DTCS; i++)
    {
        DtcRecord &record = engine.p.dtcs[i];

        if (record.code == 0)
            continue;

        if (record.status & DTC_FAILED_THIS_CYCLE)
        {
            if (record.failCycles < 255)
                record.failCycles++;

            record.passCycles = 0;
            record.aging = 0;

            if (record.failCycles >= CYCLES_TO_CONFIRM || (record.status & DTC_CONFIRMED))
            {
                record.status |= DTC_CONFIRMED | DTC_WARNING_INDICATOR;
                record.flags |= DTC_FLAG_PERMANENT;
                CaptureSnapshot(record.code);
            }
        }
        else
        {
            record.failCycles = 0;
            record.status &= ~DTC_PENDING;

            if (record.status & DTC_WARNING_INDICATOR)
            {
                if (++record.passCycles >= CYCLES_TO_MIL_OFF)
                {
                    record.status &= ~DTC_WARNING_INDICATOR;
                    record.passCycles = 0;
                }
            }

            // A permanent DTC is erased once the MIL is off and the monitor passed.
            if (!(record.status & DTC_WARNING_INDICATOR))
                record.flags &= ~DTC_FLAG_PERMANENT;
        }

        EraseIfEmpty(record);
    }

    engine.p.readinessIncomplete = 0;
    engine.cyclesSinceBoot++;

    IptIncrement(IPT_OBDCOND);
    IptIncrement(IPT_CATCOMP1);
    IptIncrement(IPT_CATCOND1);
    IptIncrement(IPT_O2SCOMP1);
    IptIncrement(IPT_O2SCOND1);
    IptIncrement(IPT_EGRCOMP);
    IptIncrement(IPT_EGRCOND);
#if DUAL_BANK
    IptIncrement(IPT_CATCOMP2);
    IptIncrement(IPT_CATCOND2);
    IptIncrement(IPT_O2SCOMP2);
    IptIncrement(IPT_O2SCOND2);
#endif

    PersistMarkDirty(true);

    if (engine.ignitionOn)
        StartDriveCycle();

    return true;
}

static void AgeDtcsOnWarmUp()
{
    for (uint8_t i = 0; i < MAX_DTCS; i++)
    {
        DtcRecord &record = engine.p.dtcs[i];

        if (record.code == 0 ||
            !(record.status & DTC_CONFIRMED) ||
            (record.status & (DTC_WARNING_INDICATOR | DTC_FAILED_THIS_CYCLE)))
            continue;

        if (++record.aging >= WARMUPS_TO_ERASE)
        {
            record.status &= ~DTC_CONFIRMED;
            EraseIfEmpty(record);
        }
    }

    PersistMarkDirty(false);
}

// =============================================
// MANUAL DTC CONTROL (serial DTC:ADD/DEL/SET)
// =============================================

bool DtcInList(const DtcRecord &record, DtcListType type)
{
    if (record.code == 0)
        return false;

    switch (type)
    {
        case DTC_LIST_STORED: return (record.status & DTC_CONFIRMED) != 0;
        case DTC_LIST_PENDING: return (record.status & DTC_PENDING) != 0;
        default: return (record.flags & DTC_FLAG_PERMANENT) != 0;
    }
}

bool DtcManualAdd(DtcListType type, uint16_t code)
{
    DtcRecord *record = FindOrCreateRecord(code);

    if (record == NULL)
        return false;

    switch (type)
    {
        case DTC_LIST_STORED:
            record->status |= DTC_CONFIRMED | DTC_WARNING_INDICATOR | DTC_FAILED_SINCE_CLEAR;
            record->status &= ~DTC_NOT_COMPLETED_SINCE_CLEAR;
            record->passCycles = 0;
            record->aging = 0;
            CaptureSnapshot(code);
            break;

        case DTC_LIST_PENDING:
            record->status |= DTC_PENDING | DTC_FAILED_SINCE_CLEAR;
            record->status &= ~DTC_NOT_COMPLETED_SINCE_CLEAR;
            CaptureSnapshot(code);
            break;

        default:
            record->flags |= DTC_FLAG_PERMANENT;
            break;
    }

    PersistMarkDirty(true);
    return true;
}

void DtcManualRemove(DtcListType type, uint16_t code)
{
    DtcRecord *record = FindRecord(code);

    if (record == NULL)
        return;

    switch (type)
    {
        case DTC_LIST_STORED:
            record->status &= ~(DTC_CONFIRMED | DTC_WARNING_INDICATOR);
            record->passCycles = 0;
            record->aging = 0;
            break;

        case DTC_LIST_PENDING:
            record->status &= ~(DTC_PENDING | DTC_FAILED_THIS_CYCLE);
            record->failCycles = 0;
            break;

        default:
            record->flags &= ~DTC_FLAG_PERMANENT;
            break;
    }

    EraseIfEmpty(*record);
    PersistMarkDirty(true);
}

static void ResetSinceClearCounters()
{
    engine.p.distanceMilOnM = 0;
    engine.p.distanceSinceClearM = 0;
    engine.p.secondsMilOn = 0;
    engine.p.secondsSinceClear = 0;
    engine.p.warmUpsSinceClear = 0;
    engine.p.readinessIncomplete = 1;
    engine.p.evapTestDone = 0;
    engine.p.evapLeakResult = 0;
}

void ClearDiagnosticInformation(uint8_t ecuMask)
{
    for (uint8_t i = 0; i < MAX_DTCS; i++)
    {
        DtcRecord &record = engine.p.dtcs[i];

        if (record.code == 0 || !(ecuMask & (1 << RecordOwner(record))))
            continue;

        FreeSnapshot(record.code);

        // Permanent DTCs and still-present faults survive a clear.
        record.status = (record.status & DTC_TEST_FAILED) |
                        DTC_NOT_COMPLETED_SINCE_CLEAR | DTC_NOT_COMPLETED_THIS_CYCLE;
        record.failCycles = 0;
        record.passCycles = 0;
        record.aging = 0;
        record.flags &= DTC_FLAG_PERMANENT | DTC_FLAG_TCM;

        EraseIfEmpty(record);
    }

    if (ecuMask & (1 << ECU_ECM))
        ResetSinceClearCounters();

    PersistMarkDirty(true);
}

void ClearAllDtcs()
{
    memset(engine.p.dtcs, 0, sizeof(engine.p.dtcs));
    memset(engine.p.snapshots, 0, sizeof(engine.p.snapshots));
    ResetSinceClearCounters();
    PersistMarkDirty(true);
}

uint8_t CountDtcs(EcuIndex ecu, DtcListType type)
{
    uint8_t count = 0;

    for (uint8_t i = 0; i < MAX_DTCS; i++)
    {
        const DtcRecord &record = engine.p.dtcs[i];

        if (DtcInList(record, type) && RecordOwner(record) == ecu)
            count++;
    }

    return count;
}

bool MilOn(EcuIndex ecu)
{
    for (uint8_t i = 0; i < MAX_DTCS; i++)
    {
        const DtcRecord &record = engine.p.dtcs[i];

        // On, or flashing for an active severe misfire (even while only pending).
        if (record.code != 0 && RecordOwner(record) == ecu &&
            ((record.status & DTC_WARNING_INDICATOR) ||
             ((record.status & DTC_TEST_FAILED) && (record.flags & DTC_FLAG_SEVERE))))
            return true;
    }

    return false;
}

uint8_t MilState()
{
    bool on = false;

    for (uint8_t i = 0; i < MAX_DTCS; i++)
    {
        const DtcRecord &record = engine.p.dtcs[i];

        if (record.code == 0)
            continue;

        if ((record.status & DTC_TEST_FAILED) && (record.flags & DTC_FLAG_SEVERE))
            return 2;  // severe misfire: flashing

        if (record.status & DTC_WARNING_INDICATOR)
            on = true;
    }

    return on ? 1 : 0;
}

// =============================================
// DTC TEXT
// =============================================

static int8_t HexDigit(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

bool ParseDtc(const char *text, uint16_t &code)
{
    if (strlen(text) != 5)
        return false;

    uint16_t system;

    switch (text[0])
    {
        case 'P': case 'p': system = 0; break;
        case 'C': case 'c': system = 1; break;
        case 'B': case 'b': system = 2; break;
        case 'U': case 'u': system = 3; break;
        default: return false;
    }

    int8_t first = HexDigit(text[1]);

    if (first < 0 || first > 3)
        return false;

    uint16_t result = (system << 14) | ((uint16_t)first << 12);

    for (uint8_t i = 2; i < 5; i++)
    {
        int8_t digit = HexDigit(text[i]);

        if (digit < 0)
            return false;

        result |= (uint16_t)digit << (4 * (4 - i));
    }

    // 0 marks an unused slot, so P0000 cannot be stored.
    if (result == 0)
        return false;

    code = result;
    return true;
}

void FormatDtc(uint16_t code, char *out)
{
    static const char systems[] = "PCBU";
    static const char hex[] = "0123456789ABCDEF";

    out[0] = systems[(code >> 14) & 0x03];
    out[1] = hex[(code >> 12) & 0x03];
    out[2] = hex[(code >> 8) & 0x0F];
    out[3] = hex[(code >> 4) & 0x0F];
    out[4] = hex[code & 0x0F];
    out[5] = '\0';
}

// =============================================
// IN-USE PERFORMANCE TRACKING
// =============================================

void IptIncrement(IptCounter counter)
{
#if FEATURE_IPT
    if (engine.p.ipt[counter] < 0xFFFF)
        engine.p.ipt[counter]++;

    PersistMarkDirty(false);
#else
    (void)counter;
#endif
}

// =============================================
// INITIALISATION AND IGNITION
// =============================================

void EngineStateInit()
{
    memset(&engine, 0, sizeof(engine));

    ParamDef def;

    for (uint8_t i = 0; i < PARAM_COUNT; i++)
    {
        ReadParamDef(i, def);
        engine.values[i] = def.defaultValue;
    }

    engine.logLevel = 1;
    engine.catalystTempC = 20;

    PersistLoad();

    // Live test results are not kept across power-up: the app re-reports
    // present faults. Otherwise a fault whose "gone" report was lost would
    // stay failing forever.
    for (uint8_t i = 0; i < MAX_DTCS; i++)
    {
        DtcRecord &record = engine.p.dtcs[i];
        record.status &= ~DTC_TEST_FAILED;
        record.flags &= ~DTC_FLAG_SEVERE;

        if (record.code != 0)
            EraseIfEmpty(record);
    }

    // Power-on is an ignition-on event.
    engine.ignitionOn = true;
    StartDriveCycle();
    IptIncrement(IPT_IGNCNTR);
}

bool EngineRunning()
{
    return engine.ignitionOn && engine.values[P_RPM] >= 300;
}

void SetIgnition(bool on)
{
    if (on == engine.ignitionOn)
        return;

    engine.ignitionOn = on;

    if (on)
    {
        StartDriveCycle();
        IptIncrement(IPT_IGNCNTR);
        return;
    }

    engine.engineRunning = false;

    // A short ignition cycle is not a drive cycle; either way the cycle is
    // closed, so a later CYCLE:1 cannot count it.
    if (engine.cycleRunMs >= CYCLE_MIN_RUN_S * 1000UL)
        EndDriveCycle();
    else
        engine.cycleOpen = false;

    // Save counters and odometer now rather than at the next interval.
    PersistMarkDirty(true);
}

// =============================================
// FAULT LOOKUP FOR DERIVED VALUES
// =============================================

bool FaultActive(uint16_t code)
{
    const DtcRecord *record = FindRecord(code);
    return record != NULL && (record->status & DTC_TEST_FAILED);
}

// =============================================
// PERIODIC UPDATE
// =============================================

void EngineStateService()
{
    static unsigned long lastUpdateMs = 0;
    static uint32_t distanceAccumulator = 0;  // km/h * ms (3600 per metre)
    static uint16_t runMsAccumulator = 0;

    unsigned long now = millis();
    unsigned long elapsed = now - lastUpdateMs;

    if (elapsed < 100)
        return;

    lastUpdateMs = now;

    if (!engine.ignitionOn)
    {
        engine.engineRunning = false;
        return;
    }

    bool running = EngineRunning();

    if (running && !engine.engineRunning)
        engine.engineStartMs = now;

    engine.engineRunning = running;
    bool mil = MilState() > 0;

    // Run time (PIDs 4D, 4E) and drive-cycle length.
    if (running)
    {
        engine.cycleRunMs += elapsed;
        runMsAccumulator += elapsed;

        while (runMsAccumulator >= 1000)
        {
            runMsAccumulator -= 1000;
            engine.p.secondsSinceClear++;

            if (mil)
                engine.p.secondsMilOn++;
        }
    }

    // Distance (PIDs 21, 31, A6).
    distanceAccumulator += (uint32_t)V(P_SPEED) * elapsed;
    uint32_t metres = distanceAccumulator / 3600;

    if (metres > 0)
    {
        distanceAccumulator -= metres * 3600;
        engine.p.odometerM += metres;
        engine.p.distanceSinceClearM += metres;

        if (mil)
            engine.p.distanceMilOnM += metres;

        PersistMarkDirty(false);
    }

    // Warm-up cycle (PID 30, DTC aging): coolant was cold, then reached 70 °C.
    int32_t ect = V(P_ECT);

    if (ect < 50)
        engine.coldSinceLastWarmUp = true;

    if (ect >= 70 && engine.coldSinceLastWarmUp)
    {
        engine.coldSinceLastWarmUp = false;

        if (engine.p.warmUpsSinceClear < 255)
            engine.p.warmUpsSinceClear++;

        AgeDtcsOnWarmUp();
    }

    // Catalyst temperature: first-order lag (time constant 20 s) toward
    // 350 °C + 4.5 °C per % load while running, ambient otherwise.
    int32_t target = running ? 350 + V(P_LOAD) * 45 / 100 : V(P_AMBIENT);
    int32_t step = (target - engine.catalystTempC) * (int32_t)elapsed / 20000;

    if (step == 0 && target != engine.catalystTempC)
        step = target > engine.catalystTempC ? 1 : -1;

    engine.catalystTempC += step;

#if FEATURE_MODE08
    // Mode 08 EVAP leak test: pull vacuum for 3 s, hold, report at 10 s.
    if (engine.evapTestState == 1)
    {
        unsigned long t = now - engine.evapTestStartMs;
        uint16_t leakPaPerS = FaultActive(0x0442) ? 400 : FaultActive(0x0456) ? 150 : 10;

        if (t < 3000)
        {
            engine.evapPressurePa = -(int16_t)(t / 2);  // to -1500 Pa
        }
        else if (t < 10000)
        {
            int32_t pressure = -1500 + (int32_t)(t - 3000) * leakPaPerS / 1000;
            engine.evapPressurePa = pressure > 0 ? 0 : (int16_t)pressure;
        }
        else
        {
            engine.evapTestState = 0;
            engine.evapPressurePa = 0;
            engine.p.evapTestDone = 1;
            engine.p.evapLeakResult = FaultActive(0x0442) ? 60 : FaultActive(0x0456) ? 30 : 5;
            IptIncrement(IPT_EVAPCOMP);
            IptIncrement(IPT_EVAPCOND);
            PersistMarkDirty(true);
        }
    }
#endif
}
