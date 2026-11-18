#include "SerialCommands.h"
#include "Config.h"
#include "EngineState.h"
#include "CanLink.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

// =============================================
// SERIAL PROTOCOL
// =============================================
//
// Request line:  [#<seq> ]<body>\n
//   body = INFO | GET | DTC? | NAME:VALUE[;NAME:VALUE...]
//
// Reply: zero or more  ERR [#<seq>] <REASON> <NAME>   (one per rejected pair)
//        then exactly  OK [#<seq>] [ITEM;ITEM...]      (applied values echoed)
//        or, if the whole line is unusable, a single  ERR [#<seq>] <REASON>
//
// Echoed values are the values actually applied after clamping and rounding.

static char lineBuffer[SERIAL_LINE_LENGTH + 1];
static uint8_t linePosition = 0;
static bool lineOverflow = false;

static bool hasSequence = false;
static uint16_t sequence = 0;

static unsigned long droppedLogLines = 0;

bool LogLineFits(uint8_t length)
{
    if (Serial.availableForWrite() >= length)
        return true;

    droppedLogLines++;
    return false;
}

// Echo item codes: 0..PARAM_COUNT-1 are parameters.
enum EchoItem : uint8_t
{
    ECHO_CAN = 100,
    ECHO_IGN,
    ECHO_LOG,
    ECHO_VIN,
    ECHO_CALID,
    ECHO_ECUNAME,
    ECHO_DTC
};

const uint8_t MAX_ECHO_ITEMS = 24;
static uint8_t echoItems[MAX_ECHO_ITEMS];
static uint8_t echoCount = 0;

// =============================================
// OUTPUT HELPERS
// =============================================

static void PrintSequence()
{
    if (hasSequence)
    {
        Serial.print(F(" #"));
        Serial.print(sequence);
    }
}

static void PrintError(const __FlashStringHelper *reason, const char *name)
{
    Serial.print(F("ERR"));
    PrintSequence();
    Serial.print(' ');
    Serial.print(reason);

    if (name != NULL && name[0] != '\0')
    {
        Serial.print(' ');
        Serial.print(name);
    }

    Serial.println();
}

static void PrintParam(uint8_t index)
{
    ParamDef def;
    ReadParamDef(index, def);

    char number[16];
    dtostrf(engine.values[index], 1, def.decimals, number);

    Serial.print(def.name);
    Serial.print('=');
    Serial.print(number);
}

static void PrintDtcList(DtcListType type)
{
    const DtcList &list = engine.dtcs[type];
    char text[6];

    for (uint8_t i = 0; i < list.count; i++)
    {
        if (i > 0)
            Serial.print(',');

        FormatDtc(list.codes[i], text);
        Serial.print(text);
    }
}

static void PrintDtcState()
{
    Serial.print(F("DTC=S:"));
    PrintDtcList(DTC_STORED);
    Serial.print(F("|P:"));
    PrintDtcList(DTC_PENDING);
    Serial.print(F("|X:"));
    PrintDtcList(DTC_PERMANENT);
}

static void PrintEchoItem(uint8_t item)
{
    if (item < PARAM_COUNT)
    {
        PrintParam(item);
        return;
    }

    switch (item)
    {
        case ECHO_CAN:     Serial.print(F("CAN="));     Serial.print(CanActiveMode()); break;
        case ECHO_IGN:     Serial.print(F("IGN="));     Serial.print(engine.ignitionOn ? 1 : 0); break;
        case ECHO_LOG:     Serial.print(F("LOG="));     Serial.print(engine.logLevel); break;
        case ECHO_VIN:     Serial.print(F("VIN="));     Serial.print(engine.vin); break;
        case ECHO_CALID:   Serial.print(F("CALID="));   Serial.print(engine.calibrationId); break;
        case ECHO_ECUNAME: Serial.print(F("ECUNAME=")); Serial.print(engine.ecuName); break;
        case ECHO_DTC:     PrintDtcState(); break;
    }
}

static void AddEcho(uint8_t item)
{
    for (uint8_t i = 0; i < echoCount; i++)
    {
        if (echoItems[i] == item)
            return;
    }

    if (echoCount < MAX_ECHO_ITEMS)
        echoItems[echoCount++] = item;
}

static void PrintOk()
{
    Serial.print(F("OK"));
    PrintSequence();

    for (uint8_t i = 0; i < echoCount; i++)
    {
        Serial.print(i == 0 ? ' ' : ';');
        PrintEchoItem(echoItems[i]);
    }

    Serial.println();
}

// =============================================
// QUERIES
// =============================================

static void ReplyInfo()
{
    Serial.print(F("OK"));
    PrintSequence();
    Serial.print(F(" FW=" FIRMWARE_VERSION ";CAN="));
    Serial.print(CanActiveMode());
    Serial.print(F(";CANOK="));
    Serial.print(CanReady() ? 1 : 0);
    Serial.print(F(";IGN="));
    Serial.print(engine.ignitionOn ? 1 : 0);
    Serial.print(F(";LOG="));
    Serial.print(engine.logLevel);
    Serial.print(F(";UP="));
    Serial.print(millis());
    Serial.print(F(";SEQ="));
    Serial.print(engine.lastSequence);
    Serial.print(F(";MIL="));
    Serial.print(MilOn() ? 1 : 0);
    Serial.print(F(";LOGDROP="));
    Serial.print(droppedLogLines);
    Serial.print(F(";VIN="));
    Serial.print(engine.vin);
    Serial.print(F(";CALID="));
    Serial.print(engine.calibrationId);
    Serial.print(F(";ECUNAME="));
    Serial.println(engine.ecuName);
}

static void ReplyGet()
{
    Serial.print(F("OK"));
    PrintSequence();

    for (uint8_t i = 0; i < PARAM_COUNT; i++)
    {
        Serial.print(i == 0 ? ' ' : ';');
        PrintParam(i);
    }

    Serial.println();
}

static void ReplyDtcs()
{
    Serial.print(F("OK"));
    PrintSequence();
    Serial.print(' ');
    PrintDtcState();
    Serial.print(F(";FF="));

    if (engine.freezeFrame.valid)
    {
        char text[6];
        FormatDtc(engine.freezeFrame.dtc, text);
        Serial.println(text);
    }
    else
    {
        Serial.println('-');
    }
}

// =============================================
// VALUE PARSING
// =============================================

static bool ParseFloat(const char *text, float &value)
{
    if (*text == '\0')
        return false;

    char *end;
    double parsed = strtod(text, &end);

    if (end == text || *end != '\0' || !isfinite(parsed))
        return false;

    value = (float)parsed;
    return true;
}

static bool ParseInteger(const char *text, long minimum, long maximum, long &value)
{
    if (*text == '\0')
        return false;

    char *end;
    long parsed = strtol(text, &end, 10);

    if (*end != '\0' || parsed < minimum || parsed > maximum)
        return false;

    value = parsed;
    return true;
}

static bool IsValidVin(const char *text)
{
    if (strlen(text) != 17)
        return false;

    for (uint8_t i = 0; i < 17; i++)
    {
        char c = text[i];
        bool digit = c >= '0' && c <= '9';
        bool letter = c >= 'A' && c <= 'Z' && c != 'I' && c != 'O' && c != 'Q';

        if (!digit && !letter)
            return false;
    }

    return true;
}

static bool IsValidText(const char *text, uint8_t maxLength)
{
    uint8_t length = strlen(text);

    if (length > maxLength)
        return false;

    for (uint8_t i = 0; i < length; i++)
    {
        char c = text[i];

        if (c < 0x20 || c > 0x7E || c == ';' || c == '#' || c == '|' || c == '=')
            return false;
    }

    return true;
}

static bool ParseDtcType(char letter, DtcListType &type)
{
    switch (letter)
    {
        case 'S': type = DTC_STORED; return true;
        case 'P': type = DTC_PENDING; return true;
        case 'X': type = DTC_PERMANENT; return true;
        default: return false;
    }
}

static void InvalidateFreezeFrameIfNoStoredDtcs()
{
    if (engine.dtcs[DTC_STORED].count == 0)
        engine.freezeFrame.valid = false;
}

// DTC:ADD:<S|P|X>:<code>   DTC:DEL:<S|P|X>:<code>
// DTC:SET:<S|P|X>:<code>,<code>...  (replaces the list; empty clears it)
// DTC:CLEAR                (Mode 04 behaviour, keeps permanent DTCs)
// DTC:CLEAR:ALL            (also clears permanent DTCs)
static bool ApplyDtcCommand(char *value, const __FlashStringHelper *&reason)
{
    reason = F("BAD_VALUE");

    if (strcmp_P(value, PSTR("CLEAR")) == 0)
    {
        ClearDiagnosticInformation();
        return true;
    }

    if (strcmp_P(value, PSTR("CLEAR:ALL")) == 0)
    {
        ClearDiagnosticInformation();
        ClearDtcList(DTC_PERMANENT);
        return true;
    }

    // <ACTION>:<TYPE>:<ARGUMENT>
    if (strlen(value) < 6 || value[3] != ':' || value[5] != ':')
        return false;

    DtcListType type;

    if (!ParseDtcType(value[4], type))
        return false;

    char *argument = value + 6;

    if (strncmp_P(value, PSTR("SET"), 3) == 0)
    {
        uint16_t codes[MAX_DTCS_PER_LIST];
        uint8_t count = 0;

        // Validate the whole list before changing anything.
        char *token = argument;

        while (*token != '\0')
        {
            char *comma = strchr(token, ',');

            if (comma != NULL)
                *comma = '\0';

            if (count >= MAX_DTCS_PER_LIST)
            {
                reason = F("LIST_FULL");
                return false;
            }

            if (!ParseDtc(token, codes[count]))
                return false;

            count++;

            if (comma == NULL)
                break;

            token = comma + 1;
        }

        DtcList &list = engine.dtcs[type];

        // Remove codes that are no longer wanted, then add new ones. Codes
        // already present stay, so the freeze frame is not re-captured.
        for (int8_t i = list.count - 1; i >= 0; i--)
        {
            bool keep = false;

            for (uint8_t j = 0; j < count; j++)
            {
                if (codes[j] == list.codes[i])
                    keep = true;
            }

            if (!keep)
                RemoveDtc(type, list.codes[i]);
        }

        for (uint8_t j = 0; j < count; j++)
            AddDtc(type, codes[j]);

        InvalidateFreezeFrameIfNoStoredDtcs();
        return true;
    }

    uint16_t code;

    if (!ParseDtc(argument, code))
        return false;

    if (strncmp_P(value, PSTR("ADD"), 3) == 0)
    {
        if (!AddDtc(type, code))
        {
            reason = F("LIST_FULL");
            return false;
        }

        return true;
    }

    if (strncmp_P(value, PSTR("DEL"), 3) == 0)
    {
        RemoveDtc(type, code);
        InvalidateFreezeFrameIfNoStoredDtcs();
        return true;
    }

    return false;
}

// =============================================
// NAME:VALUE PAIRS
// =============================================

static void ApplyPair(char *pair)
{
    char *separator = strchr(pair, ':');

    if (separator == NULL)
    {
        PrintError(F("BAD_FORMAT"), pair);
        return;
    }

    *separator = '\0';

    char *name = pair;
    char *value = separator + 1;

    // Engine parameter.
    int8_t index = FindParam(name);

    if (index >= 0)
    {
        float parsed;

        if (!ParseFloat(value, parsed))
        {
            PrintError(F("BAD_VALUE"), name);
            return;
        }

        // PID 03 fuel system status must have exactly one bit set (or 0).
        if (index == P_FUELSYS &&
            parsed != 0 && parsed != 1 && parsed != 2 &&
            parsed != 4 && parsed != 8 && parsed != 16)
        {
            PrintError(F("BAD_VALUE"), name);
            return;
        }

        ParamDef def;
        ReadParamDef(index, def);

        parsed = constrain(parsed, def.minimum, def.maximum);

        float scale = pow(10, def.decimals);
        engine.values[index] = round(parsed * scale) / scale;

        AddEcho(index);
        return;
    }

    long number;

    if (strcmp_P(name, PSTR("CAN")) == 0)
    {
        bool applied;
        bool ok;

        if (!ParseInteger(value, 0, 255, number) ||
            !CanRequestMode((uint8_t)number, applied, ok))
        {
            PrintError(F("BAD_VALUE"), name);
            return;
        }

        if (applied && !ok)
        {
            PrintError(F("CAN_INIT_FAILED"), name);
            return;
        }

        AddEcho(ECHO_CAN);
        return;
    }

    if (strcmp_P(name, PSTR("IGN")) == 0)
    {
        if (!ParseInteger(value, 0, 1, number))
        {
            PrintError(F("BAD_VALUE"), name);
            return;
        }

        engine.ignitionOn = number == 1;
        AddEcho(ECHO_IGN);
        return;
    }

    if (strcmp_P(name, PSTR("LOG")) == 0)
    {
        if (!ParseInteger(value, 0, 2, number))
        {
            PrintError(F("BAD_VALUE"), name);
            return;
        }

        engine.logLevel = (uint8_t)number;
        AddEcho(ECHO_LOG);
        return;
    }

    if (strcmp_P(name, PSTR("VIN")) == 0)
    {
        if (!IsValidVin(value))
        {
            PrintError(F("BAD_VALUE"), name);
            return;
        }

        strncpy(engine.vin, value, sizeof(engine.vin) - 1);
        AddEcho(ECHO_VIN);
        return;
    }

    if (strcmp_P(name, PSTR("CALID")) == 0)
    {
        if (!IsValidText(value, sizeof(engine.calibrationId) - 1))
        {
            PrintError(F("BAD_VALUE"), name);
            return;
        }

        memset(engine.calibrationId, 0, sizeof(engine.calibrationId));
        strncpy(engine.calibrationId, value, sizeof(engine.calibrationId) - 1);
        AddEcho(ECHO_CALID);
        return;
    }

    if (strcmp_P(name, PSTR("ECUNAME")) == 0)
    {
        if (!IsValidText(value, sizeof(engine.ecuName) - 1))
        {
            PrintError(F("BAD_VALUE"), name);
            return;
        }

        memset(engine.ecuName, 0, sizeof(engine.ecuName));
        strncpy(engine.ecuName, value, sizeof(engine.ecuName) - 1);
        AddEcho(ECHO_ECUNAME);
        return;
    }

    if (strcmp_P(name, PSTR("DTC")) == 0)
    {
        const __FlashStringHelper *reason;

        if (!ApplyDtcCommand(value, reason))
        {
            PrintError(reason, name);
            return;
        }

        AddEcho(ECHO_DTC);
        return;
    }

    PrintError(F("UNKNOWN"), name);
}

// =============================================
// LINE PROCESSING
// =============================================

static void ProcessLine(char *line)
{
    hasSequence = false;
    echoCount = 0;

    while (*line == ' ')
        line++;

    // Optional sequence number.
    if (*line == '#')
    {
        char *end;
        unsigned long parsed = strtoul(line + 1, &end, 10);

        if (end == line + 1 || parsed > 65535 || (*end != ' ' && *end != '\0'))
        {
            PrintError(F("BAD_SEQUENCE"), NULL);
            return;
        }

        hasSequence = true;
        sequence = (uint16_t)parsed;
        engine.lastSequence = sequence;

        line = end;

        while (*line == ' ')
            line++;
    }

    if (*line == '\0')
    {
        PrintError(F("EMPTY"), NULL);
        return;
    }

    if (strcmp_P(line, PSTR("INFO")) == 0) { ReplyInfo(); return; }
    if (strcmp_P(line, PSTR("GET")) == 0)  { ReplyGet(); return; }
    if (strcmp_P(line, PSTR("DTC?")) == 0) { ReplyDtcs(); return; }

    // One or more NAME:VALUE pairs separated by ';'.
    char *pair = line;

    while (pair != NULL)
    {
        char *next = strchr(pair, ';');

        if (next != NULL)
            *next++ = '\0';

        while (*pair == ' ')
            pair++;

        if (*pair != '\0')
            ApplyPair(pair);

        pair = next;
    }

    PrintOk();
}

void SerialPump()
{
    while (Serial.available())
    {
        char incoming = Serial.read();

        if (incoming == '\n')
        {
            if (lineOverflow)
            {
                hasSequence = false;
                PrintError(F("LINE_TOO_LONG"), NULL);
            }
            else
            {
                lineBuffer[linePosition] = '\0';
                ProcessLine(lineBuffer);
            }

            linePosition = 0;
            lineOverflow = false;
        }
        else if (incoming != '\r')
        {
            if (linePosition < SERIAL_LINE_LENGTH)
                lineBuffer[linePosition++] = incoming;
            else
                lineOverflow = true;
        }
    }
}
