#include "EngineState.h"
#include <string.h>

EngineState engine;

// =============================================
// PARAMETER DEFINITIONS
// =============================================
// name, minimum, maximum, default, decimals
// Decimals must match DecimalPlaces in the Windows app
// (ParameterDefinitions.cs) so echoed values compare exactly.

const ParamDef PARAM_DEFS[PARAM_COUNT] PROGMEM =
{
    { "RPM",          0,     12000,  850,   0 },
    { "MAP",          0,     170,    85,    0 },
    { "STFT",         -100,  99.2,   3.1,   1 },
    { "LTFT",         -100,  99.2,   -2.3,  1 },
    { "AFR",          8,     20,     14.7,  1 },
    { "MEASURED_AFR", 8,     20,     14.7,  1 },
    { "ECT",          -40,   215,    90,    0 },
    { "IAT",          -40,   215,    25,    0 },
    { "TPS",          0,     100,    0,     1 },
    { "SPEED",        0,     255,    0,     0 },
    { "LOAD",         0,     100,    20,    1 },
    { "MAF",          0,     254,    3.5,   1 },
    { "TIMING",       -64,   63.5,   10,    1 },
    { "FUEL",         0,     100,    75,    1 },
    { "BARO",         0,     255,    101,   0 },
    { "VBATT",        0,     65.5,   14.2,  2 },
    { "AMBIENT",      -40,   215,    20,    0 },
    { "OIL",          -40,   215,    90,    0 },
    { "O2V",          0,     5,      0.45,  3 },
    { "FUELSYS",      0,     16,     2,     0 }
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
// INITIALISATION
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

    engine.ignitionOn = true;
    engine.logLevel = 1;

    strncpy(engine.vin, DEFAULT_VIN, sizeof(engine.vin) - 1);
    strncpy(engine.calibrationId, DEFAULT_CALID, sizeof(engine.calibrationId) - 1);
    strncpy(engine.ecuName, DEFAULT_ECU_NAME, sizeof(engine.ecuName) - 1);
}

// =============================================
// COUNTERS (PIDs 1F, 21, 30, 31, 4D, 4E)
// =============================================

void UpdateCounters()
{
    static unsigned long lastUpdateMs = 0;

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

    float seconds = elapsed / 1000.0;
    bool running = engine.values[P_RPM] >= 300;

    if (running && !engine.engineRunning)
        engine.engineStartMs = now;

    engine.engineRunning = running;

    float km = engine.values[P_SPEED] * seconds / 3600.0;

    engine.distanceSinceClearKm += km;

    if (running)
        engine.secondsSinceClear += seconds;

    if (MilOn())
    {
        engine.distanceMilOnKm += km;

        if (running)
            engine.secondsMilOn += seconds;
    }

    // Warm-up cycle: coolant was cold, then reached operating temperature.
    float ect = engine.values[P_ECT];

    if (ect < 50)
        engine.coldSinceLastWarmUp = true;

    if (ect >= 70 && engine.coldSinceLastWarmUp)
    {
        engine.coldSinceLastWarmUp = false;

        if (engine.warmUpsSinceClear < 255)
            engine.warmUpsSinceClear++;
    }
}

// =============================================
// DTC HELPERS
// =============================================

bool MilOn()
{
    return engine.dtcs[DTC_STORED].count > 0;
}

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

bool AddDtc(DtcListType type, uint16_t code)
{
    DtcList &list = engine.dtcs[type];

    for (uint8_t i = 0; i < list.count; i++)
    {
        if (list.codes[i] == code)
            return true;
    }

    if (list.count >= MAX_DTCS_PER_LIST)
        return false;

    list.codes[list.count++] = code;

    // The first confirmed DTC captures the freeze frame.
    if (type == DTC_STORED && !engine.freezeFrame.valid)
    {
        engine.freezeFrame.valid = true;
        engine.freezeFrame.dtc = code;
        memcpy(engine.freezeFrame.values, engine.values, sizeof(engine.values));
    }

    return true;
}

bool RemoveDtc(DtcListType type, uint16_t code)
{
    DtcList &list = engine.dtcs[type];

    for (uint8_t i = 0; i < list.count; i++)
    {
        if (list.codes[i] == code)
        {
            for (uint8_t j = i + 1; j < list.count; j++)
                list.codes[j - 1] = list.codes[j];

            list.count--;
            return true;
        }
    }

    return false;
}

void ClearDtcList(DtcListType type)
{
    engine.dtcs[type].count = 0;
}

void ClearDiagnosticInformation()
{
    // Mode 04: permanent DTCs are not erased.
    ClearDtcList(DTC_STORED);
    ClearDtcList(DTC_PENDING);

    engine.freezeFrame.valid = false;
    engine.freezeFrame.dtc = 0;

    engine.distanceMilOnKm = 0;
    engine.distanceSinceClearKm = 0;
    engine.secondsMilOn = 0;
    engine.secondsSinceClear = 0;
    engine.warmUpsSinceClear = 0;
    engine.codesClearedSinceBoot = true;
}
