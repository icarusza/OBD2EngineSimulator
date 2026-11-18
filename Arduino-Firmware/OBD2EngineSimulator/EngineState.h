#ifndef OBD2SIM_ENGINE_STATE_H
#define OBD2SIM_ENGINE_STATE_H

#include <Arduino.h>
#include "Config.h"

// =============================================
// ENGINE PARAMETERS (fixed point)
// =============================================
// Values are stored as integers scaled by 10^decimals (e.g. STFT 3.1 % is
// stored as 31). This avoids the AVR floating-point library (~4 KB flash).

enum ParamIndex : uint8_t
{
    P_RPM = 0,
    P_MAP,
    P_STFT,
    P_LTFT,
    P_AFR,
    P_MEASURED_AFR,
    P_ECT,
    P_IAT,
    P_TPS,
    P_SPEED,
    P_LOAD,
    P_MAF,
    P_TIMING,
    P_FUEL,
    P_BARO,
    P_VBATT,
    P_AMBIENT,
    P_OIL,
    P_O2V,
    P_FUELSYS,
    P_STFT2,
    P_LTFT2,
    P_RAILP,
    P_GEAR,
    PARAM_COUNT
};

struct ParamDef
{
    char name[13];
    int32_t minimum;       // scaled
    int32_t maximum;       // scaled
    int32_t defaultValue;  // scaled
    uint8_t decimals;
};

// Stored in flash; read with ReadParamDef().
extern const ParamDef PARAM_DEFS[PARAM_COUNT] PROGMEM;

void ReadParamDef(uint8_t index, ParamDef &out);
int8_t FindParam(const char *name);

// =============================================
// ECUS
// =============================================

enum EcuIndex : uint8_t
{
    ECU_ECM = 0,
    ECU_TCM = 1,
    ECU_COUNT = 2
};

// =============================================
// DIAGNOSTIC TROUBLE CODES
// =============================================

// ISO 14229 DTC status byte (also used for the OBD lists).
const uint8_t DTC_TEST_FAILED = 0x01;
const uint8_t DTC_FAILED_THIS_CYCLE = 0x02;
const uint8_t DTC_PENDING = 0x04;
const uint8_t DTC_CONFIRMED = 0x08;
const uint8_t DTC_NOT_COMPLETED_SINCE_CLEAR = 0x10;
const uint8_t DTC_FAILED_SINCE_CLEAR = 0x20;
const uint8_t DTC_NOT_COMPLETED_THIS_CYCLE = 0x40;
const uint8_t DTC_WARNING_INDICATOR = 0x80;  // MIL on

// DtcRecord::flags
const uint8_t DTC_FLAG_PERMANENT = 0x01;
const uint8_t DTC_FLAG_SEVERE = 0x02;   // severe misfire: flashing MIL
const uint8_t DTC_FLAG_TCM = 0x04;      // owned by the transmission ECU

struct DtcRecord
{
    uint16_t code;         // 0 = unused slot; OBD 2-byte encoding (P0171 = 0x0171)
    uint8_t status;
    uint8_t failCycles;    // consecutive drive cycles with a failure
    uint8_t passCycles;    // consecutive passing cycles while the MIL is on
    uint8_t aging;         // warm-ups since the MIL went off
    uint8_t flags;
};

// List views used by Mode 03/07/0A and the serial DTC commands.
enum DtcListType : uint8_t
{
    DTC_LIST_STORED = 0,    // confirmed
    DTC_LIST_PENDING,
    DTC_LIST_PERMANENT,
    DTC_LIST_COUNT
};

// =============================================
// FREEZE FRAMES
// =============================================

// Encoded bytes of the freeze-frame PIDs (see FREEZE_FRAME_PIDS in ObdService).
const uint8_t SNAPSHOT_DATA_SIZE = 32;

struct Snapshot
{
    uint16_t dtc;          // 0 = unused
    uint8_t order;         // capture order, for "oldest first"
    uint8_t data[SNAPSHOT_DATA_SIZE];
};

// =============================================
// IN-USE PERFORMANCE TRACKING (Mode 09 InfoType 08, spark ignition)
// =============================================

enum IptCounter : uint8_t
{
    IPT_OBDCOND = 0, IPT_IGNCNTR,
    IPT_CATCOMP1, IPT_CATCOND1, IPT_CATCOMP2, IPT_CATCOND2,
    IPT_O2SCOMP1, IPT_O2SCOND1, IPT_O2SCOMP2, IPT_O2SCOND2,
    IPT_EGRCOMP, IPT_EGRCOND, IPT_AIRCOMP, IPT_AIRCOND,
    IPT_EVAPCOMP, IPT_EVAPCOND,
    IPT_COUNT
};

// =============================================
// PERSISTED STATE (EEPROM)
// =============================================

const uint8_t PERSIST_VERSION = 2;  // change when PersistentState changes

struct PersistentState
{
    uint8_t version;

    char vin[18];
    char calibrationId[17];
    char ecuName[21];
    char esn[17];

    DtcRecord dtcs[MAX_DTCS];
    Snapshot snapshots[MAX_SNAPSHOTS];
    uint8_t snapshotCounter;

    uint32_t odometerM;
    uint32_t distanceMilOnM;
    uint32_t distanceSinceClearM;
    uint32_t secondsMilOn;
    uint32_t secondsSinceClear;
    uint8_t warmUpsSinceClear;
    uint8_t readinessIncomplete;  // monitors not complete since the last clear
    uint8_t tcmEnabled;

    uint16_t ipt[IPT_COUNT];

    uint8_t evapTestDone;         // Mode 08 EVAP test completed since clear
    uint8_t evapLeakResult;       // Mode 06 MID 3C value (raw)
};

// =============================================
// COMPLETE SIMULATOR STATE
// =============================================

struct EngineState
{
    int32_t values[PARAM_COUNT];

    bool ignitionOn;
    uint8_t logLevel;            // 0 = errors, 1 = OBD requests/replies, 2 = raw frames
    uint16_t responseDelayMs;    // simulated slow ECU

    PersistentState p;

    bool engineRunning;
    unsigned long engineStartMs;
    unsigned long cycleRunMs;    // engine run time in the current drive cycle
    bool cycleOpen;              // a drive cycle is in progress (ignition on)
    uint16_t cyclesSinceBoot;
    bool coldSinceLastWarmUp;

    int16_t catalystTempC;       // bank 1 sensor 1
    int16_t evapPressurePa;      // PID 32
    uint8_t evapTestState;       // 0 idle, 1 running
    unsigned long evapTestStartMs;

    uint16_t lastSequence;
};

extern EngineState engine;

// Scaled value of a parameter.
inline int32_t V(ParamIndex index) { return engine.values[index]; }

void EngineStateInit();
void EngineStateService();       // call every loop: counters, odometer, warm-ups

void SetIgnition(bool on);
bool EndDriveCycle();            // also used by CYCLE:1; false if no cycle is open
bool EngineRunning();

// =============================================
// DTC API
// =============================================

bool ParseDtc(const char *text, uint16_t &code);
void FormatDtc(uint16_t code, char *out);  // out needs 6 bytes
EcuIndex DtcOwner(uint16_t code);           // P07xx-P09xx belong to the TCM

// Live test result from the app: severity 0 = passed, 1 = failed, 2 = severe misfire.
bool ReportFault(uint16_t code, uint8_t severity);

bool FaultActive(uint16_t code);   // a FAULT report for this code is currently failing

bool DtcInList(const DtcRecord &record, DtcListType type);
bool DtcManualAdd(DtcListType type, uint16_t code);
void DtcManualRemove(DtcListType type, uint16_t code);
void ClearDiagnosticInformation(uint8_t ecuMask);  // Mode 04 / UDS 14 (bit per ECU)
void ClearAllDtcs();                                // DTC:CLEAR:ALL

uint8_t CountDtcs(EcuIndex ecu, DtcListType type);
bool MilOn(EcuIndex ecu);
uint8_t MilState();              // 0 off, 1 on, 2 flashing

const Snapshot *FindSnapshot(uint16_t code);
const Snapshot *PrimarySnapshot();   // Mode 02 frame 0

void IptIncrement(IptCounter counter);

#endif
