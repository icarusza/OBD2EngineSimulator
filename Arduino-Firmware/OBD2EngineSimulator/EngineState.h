#ifndef OBD2SIM_ENGINE_STATE_H
#define OBD2SIM_ENGINE_STATE_H

#include <Arduino.h>
#include "Config.h"

// =============================================
// ENGINE PARAMETERS
// =============================================

// Index into EngineState::values. Order matches PARAM_DEFS in
// EngineState.cpp and the GET reply.
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
    PARAM_COUNT
};

struct ParamDef
{
    char name[13];
    float minimum;
    float maximum;
    float defaultValue;
    uint8_t decimals;
};

// Stored in flash; read with ReadParamDef().
extern const ParamDef PARAM_DEFS[PARAM_COUNT] PROGMEM;

void ReadParamDef(uint8_t index, ParamDef &out);
int8_t FindParam(const char *name);

// =============================================
// DIAGNOSTIC TROUBLE CODES
// =============================================

enum DtcListType : uint8_t
{
    DTC_STORED = 0,
    DTC_PENDING,
    DTC_PERMANENT,
    DTC_LIST_COUNT
};

struct DtcList
{
    uint16_t codes[MAX_DTCS_PER_LIST];
    uint8_t count;
};

// =============================================
// FREEZE FRAME
// =============================================

struct FreezeFrame
{
    bool valid;
    uint16_t dtc;
    float values[PARAM_COUNT];
};

// =============================================
// COMPLETE SIMULATOR STATE
// =============================================

struct EngineState
{
    float values[PARAM_COUNT];

    bool ignitionOn;
    uint8_t logLevel;  // 0 = errors, 1 = OBD requests/replies, 2 = raw frames

    DtcList dtcs[DTC_LIST_COUNT];
    FreezeFrame freezeFrame;

    // Counters for PIDs 1F, 21, 30, 31, 4D, 4E.
    bool engineRunning;
    unsigned long engineStartMs;
    float distanceMilOnKm;
    float distanceSinceClearKm;
    float secondsMilOn;
    float secondsSinceClear;
    uint8_t warmUpsSinceClear;
    bool coldSinceLastWarmUp;
    bool codesClearedSinceBoot;  // PID 01 readiness: monitors incomplete after a clear

    char vin[18];
    char calibrationId[17];
    char ecuName[21];

    uint16_t lastSequence;
};

extern EngineState engine;

void EngineStateInit();
void UpdateCounters();

bool MilOn();

// DTC helpers. Codes use the 2-byte OBD encoding (e.g. P0171 = 0x0171).
bool ParseDtc(const char *text, uint16_t &code);
void FormatDtc(uint16_t code, char *out);  // out needs 6 bytes
bool AddDtc(DtcListType type, uint16_t code);
bool RemoveDtc(DtcListType type, uint16_t code);
void ClearDtcList(DtcListType type);
void ClearDiagnosticInformation();  // Mode 04 behaviour

#endif
