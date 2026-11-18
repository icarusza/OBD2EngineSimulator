#include "SerialCommands.h"
#include "Config.h"
#include "EngineState.h"
#include "CanLink.h"
#include "Persist.h"
#include <string.h>

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
    ECHO_ESN,
    ECHO_DTC,
    ECHO_TCM,
    ECHO_RESPDELAY,
    ECHO_ODO,
    ECHO_CYCLE,
    ECHO_FACTORY
};

const uint8_t MAX_ECHO_ITEMS = 32;
static uint8_t echoItems[MAX_ECHO_ITEMS];
static uint8_t echoCount = 0;

// =============================================
// NUMBERS (fixed point, no floating-point library)
// =============================================

static const int32_t POWERS_OF_TEN[] = { 1, 10, 100, 1000 };

// Parses "-12.34" into a value scaled by 10^decimals, rounding extra digits.
static bool ParseScaled(const char *text, uint8_t decimals, int32_t &value)
{
    bool negative = false;

    if (*text == '-' || *text == '+')
        negative = *text++ == '-';

    int32_t result = 0;
    uint8_t digits = 0;
    uint8_t fraction = 0;   // fraction digits used
    bool seenPoint = false;
    bool roundUp = false;

    for (; *text != '\0'; text++)
    {
        if (*text == '.' && !seenPoint)
        {
            seenPoint = true;
            continue;
        }

        if (*text < '0' || *text > '9')
            return false;

        digits++;

        if (seenPoint && fraction >= decimals)
        {
            if (fraction == decimals)
                roundUp = *text >= '5';

            fraction = decimals + 1;  // further digits are ignored
            continue;
        }

        if (result > 99999999L)
            return false;  // too large

        result = result * 10 + (*text - '0');

        if (seenPoint)
            fraction++;
    }

    if (digits == 0)
        return false;

    if (fraction > decimals)
        fraction = decimals;

    result *= POWERS_OF_TEN[decimals - fraction];

    if (roundUp)
        result++;

    value = negative ? -result : result;
    return true;
}

static bool ParseUnsigned(const char *text, uint32_t maximum, uint32_t &value)
{
    if (*text == '\0')
        return false;

    uint32_t result = 0;

    for (; *text != '\0'; text++)
    {
        if (*text < '0' || *text > '9')
            return false;

        result = result * 10 + (*text - '0');

        if (result > maximum)
            return false;
    }

    value = result;
    return true;
}

static void PrintScaled(int32_t value, uint8_t decimals)
{
    if (value < 0)
    {
        Serial.print('-');
        value = -value;
    }

    int32_t scale = POWERS_OF_TEN[decimals];
    Serial.print(value / scale);

    if (decimals == 0)
        return;

    Serial.print('.');

    int32_t fraction = value % scale;

    for (int32_t digit = scale / 10; digit > 0; digit /= 10)
    {
        Serial.print((char)('0' + fraction / digit));
        fraction %= digit;
    }
}

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

    Serial.print(def.name);
    Serial.print('=');
    PrintScaled(engine.values[index], def.decimals);
}

static void PrintCode(uint16_t code)
{
    char text[6];
    FormatDtc(code, text);
    Serial.print(text);
}

static void PrintDtcList(DtcListType type)
{
    bool first = true;

    for (uint8_t i = 0; i < MAX_DTCS; i++)
    {
        const DtcRecord &record = engine.p.dtcs[i];

        if (!DtcInList(record, type))
            continue;

        if (!first)
            Serial.print(',');

        PrintCode(record.code);
        first = false;
    }
}

static void PrintDtcState()
{
    Serial.print(F("DTC=S:"));
    PrintDtcList(DTC_LIST_STORED);
    Serial.print(F("|P:"));
    PrintDtcList(DTC_LIST_PENDING);
    Serial.print(F("|X:"));
    PrintDtcList(DTC_LIST_PERMANENT);
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
        case ECHO_CAN:       Serial.print(F("CAN="));       Serial.print(CanActiveMode()); break;
        case ECHO_IGN:       Serial.print(F("IGN="));       Serial.print(engine.ignitionOn ? 1 : 0); break;
        case ECHO_LOG:       Serial.print(F("LOG="));       Serial.print(engine.logLevel); break;
        case ECHO_VIN:       Serial.print(F("VIN="));       Serial.print(engine.p.vin); break;
        case ECHO_CALID:     Serial.print(F("CALID="));     Serial.print(engine.p.calibrationId); break;
        case ECHO_ECUNAME:   Serial.print(F("ECUNAME="));   Serial.print(engine.p.ecuName); break;
        case ECHO_ESN:       Serial.print(F("ESN="));       Serial.print(engine.p.esn); break;
        case ECHO_DTC:       PrintDtcState(); break;
        case ECHO_TCM:       Serial.print(F("TCM="));       Serial.print(engine.p.tcmEnabled); break;
        case ECHO_RESPDELAY: Serial.print(F("RESPDELAY=")); Serial.print(engine.responseDelayMs); break;
        case ECHO_ODO:       Serial.print(F("ODO="));       PrintScaled(engine.p.odometerM / 100, 1); break;
        case ECHO_CYCLE:     Serial.print(F("CYCLE="));     Serial.print(engine.cyclesSinceBoot); break;
        case ECHO_FACTORY:   Serial.print(F("FACTORY=1")); break;
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
    Serial.print(MilState());
    Serial.print(F(";LOGDROP="));
    Serial.print(droppedLogLines);

    static const uint8_t items[] =
    {
        ECHO_TCM, ECHO_RESPDELAY, ECHO_ODO, ECHO_CYCLE,
        ECHO_VIN, ECHO_CALID, ECHO_ECUNAME, ECHO_ESN
    };

    for (uint8_t i = 0; i < sizeof(items); i++)
    {
        Serial.print(';');
        PrintEchoItem(items[i]);
    }

    Serial.println();
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

// DTC=S:..|P:..|X:..;DTCX=<code>:<status hex>:<fail>:<pass>:<aging>:<perm>:<E|T>,...;FF=<code|->
static void ReplyDtcs()
{
    Serial.print(F("OK"));
    PrintSequence();
    Serial.print(' ');
    PrintDtcState();
    Serial.print(F(";DTCX="));

    bool first = true;

    for (uint8_t i = 0; i < MAX_DTCS; i++)
    {
        const DtcRecord &record = engine.p.dtcs[i];

        if (record.code == 0)
            continue;

        if (!first)
            Serial.print(',');

        first = false;
        PrintCode(record.code);
        Serial.print(':');

        if (record.status < 0x10)
            Serial.print('0');

        Serial.print(record.status, HEX);
        Serial.print(':');
        Serial.print(record.failCycles);
        Serial.print(':');
        Serial.print(record.passCycles);
        Serial.print(':');
        Serial.print(record.aging);
        Serial.print(':');
        Serial.print((record.flags & DTC_FLAG_PERMANENT) ? 1 : 0);
        Serial.print((record.flags & DTC_FLAG_TCM) ? F(":T") : F(":E"));
    }

    Serial.print(F(";FF="));

    const Snapshot *snapshot = PrimarySnapshot();

    if (snapshot != NULL)
        PrintCode(snapshot->dtc);
    else
        Serial.print('-');

    Serial.println();
}

// =============================================
// TEXT VALIDATION
// =============================================

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

// Copies validated text into a fixed field, padding with zeros.
static bool SetText(char *field, uint8_t size, const char *value, bool vin)
{
    if (vin ? !IsValidVin(value) : !IsValidText(value, size - 1))
        return false;

    memset(field, 0, size);
    strncpy(field, value, size - 1);
    PersistMarkDirty(true);
    return true;
}

// =============================================
// DTC COMMANDS
// =============================================

static bool ParseDtcType(char letter, DtcListType &type)
{
    switch (letter)
    {
        case 'S': type = DTC_LIST_STORED; return true;
        case 'P': type = DTC_LIST_PENDING; return true;
        case 'X': type = DTC_LIST_PERMANENT; return true;
        default: return false;
    }
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
        ClearDiagnosticInformation(0xFF);
        return true;
    }

    if (strcmp_P(value, PSTR("CLEAR:ALL")) == 0)
    {
        ClearAllDtcs();
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
        uint16_t codes[MAX_DTCS];
        uint8_t count = 0;
        char *token = argument;

        // Validate the whole list before changing anything.
        while (*token != '\0')
        {
            char *comma = strchr(token, ',');

            if (comma != NULL)
                *comma = '\0';

            if (count >= MAX_DTCS)
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

        // Remove codes no longer wanted, then add new ones; codes already
        // in the list keep their state and freeze frame.
        for (uint8_t i = 0; i < MAX_DTCS; i++)
        {
            uint16_t code = engine.p.dtcs[i].code;
            bool keep = false;

            if (!DtcInList(engine.p.dtcs[i], type))
                continue;

            for (uint8_t j = 0; j < count; j++)
            {
                if (codes[j] == code)
                    keep = true;
            }

            if (!keep)
                DtcManualRemove(type, code);
        }

        for (uint8_t j = 0; j < count; j++)
        {
            if (!DtcManualAdd(type, codes[j]))
            {
                reason = F("LIST_FULL");
                return false;
            }
        }

        return true;
    }

    uint16_t code;

    if (!ParseDtc(argument, code))
        return false;

    if (strncmp_P(value, PSTR("ADD"), 3) == 0)
    {
        if (!DtcManualAdd(type, code))
        {
            reason = F("LIST_FULL");
            return false;
        }

        return true;
    }

    if (strncmp_P(value, PSTR("DEL"), 3) == 0)
    {
        DtcManualRemove(type, code);
        return true;
    }

    return false;
}

// FAULT:<code>:<0|1|2>
static bool ApplyFaultCommand(char *value, const __FlashStringHelper *&reason)
{
    reason = F("BAD_VALUE");

    char *separator = strchr(value, ':');

    if (separator == NULL)
        return false;

    *separator = '\0';

    uint16_t code;
    uint32_t severity;

    if (!ParseDtc(value, code) || !ParseUnsigned(separator + 1, 2, severity))
        return false;

    if (!ReportFault(code, (uint8_t)severity))
    {
        reason = F("LIST_FULL");
        return false;
    }

    return true;
}

// =============================================
// NAME:VALUE PAIRS
// =============================================

static bool NameIs(const char *name, const char *flashName)
{
    return strcmp_P(name, flashName) == 0;
}

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
        ParamDef def;
        ReadParamDef(index, def);

        int32_t parsed;

        if (!ParseScaled(value, def.decimals, parsed))
        {
            PrintError(F("BAD_VALUE"), name);
            return;
        }

        // PID 03 fuel system status must have exactly one bit set (or 0).
        if (index == P_FUELSYS && (parsed & (parsed - 1)) != 0)
        {
            PrintError(F("BAD_VALUE"), name);
            return;
        }

        engine.values[index] = constrain(parsed, def.minimum, def.maximum);
        AddEcho(index);
        return;
    }

    uint32_t number;
    const __FlashStringHelper *reason = F("BAD_VALUE");
    uint8_t echo = 0;
    bool ok = false;

    if (NameIs(name, PSTR("CAN")))
    {
        bool applied;
        bool started;

        ok = ParseUnsigned(value, 255, number) &&
             CanRequestMode((uint8_t)number, applied, started);

        if (ok && applied && !started)
        {
            reason = F("CAN_INIT_FAILED");
            ok = false;
        }

        echo = ECHO_CAN;
    }
    else if (NameIs(name, PSTR("IGN")))
    {
        ok = ParseUnsigned(value, 1, number);

        if (ok)
            SetIgnition(number == 1);

        echo = ECHO_IGN;
    }
    else if (NameIs(name, PSTR("LOG")))
    {
        ok = ParseUnsigned(value, 2, number);

        if (ok)
            engine.logLevel = (uint8_t)number;

        echo = ECHO_LOG;
    }
    else if (NameIs(name, PSTR("TCM")))
    {
        ok = ParseUnsigned(value, 1, number);

        if (ok)
        {
            engine.p.tcmEnabled = (uint8_t)number;
            PersistMarkDirty(true);
        }

        echo = ECHO_TCM;
    }
    else if (NameIs(name, PSTR("RESPDELAY")))
    {
        ok = ParseUnsigned(value, 5000, number);

        if (ok)
            engine.responseDelayMs = (uint16_t)number;

        echo = ECHO_RESPDELAY;
    }
    else if (NameIs(name, PSTR("VIN")))
    {
        ok = SetText(engine.p.vin, sizeof(engine.p.vin), value, true);
        echo = ECHO_VIN;
    }
    else if (NameIs(name, PSTR("CALID")))
    {
        ok = SetText(engine.p.calibrationId, sizeof(engine.p.calibrationId), value, false);
        echo = ECHO_CALID;
    }
    else if (NameIs(name, PSTR("ECUNAME")))
    {
        ok = SetText(engine.p.ecuName, sizeof(engine.p.ecuName), value, false);
        echo = ECHO_ECUNAME;
    }
    else if (NameIs(name, PSTR("ESN")))
    {
        ok = SetText(engine.p.esn, sizeof(engine.p.esn), value, false);
        echo = ECHO_ESN;
    }
    else if (NameIs(name, PSTR("DTC")))
    {
        ok = ApplyDtcCommand(value, reason);
        echo = ECHO_DTC;
    }
    else if (NameIs(name, PSTR("FAULT")))
    {
        ok = ApplyFaultCommand(value, reason);
        echo = ECHO_DTC;
    }
    else if (NameIs(name, PSTR("CYCLE")))
    {
        ok = ParseUnsigned(value, 1, number) && number == 1;

        // Needs an open drive cycle (ignition on, not already ended).
        if (ok && !EndDriveCycle())
        {
            reason = F("IGN_OFF");
            ok = false;
        }

        echo = ECHO_CYCLE;
    }
    else if (NameIs(name, PSTR("ODO")))
    {
        int32_t km10;
        // Odometer is kept in metres (uint32): at most 4 294 967.2 km.
        ok = ParseScaled(value, 1, km10) && km10 >= 0 && km10 <= 42949672L;

        if (ok)
        {
            engine.p.odometerM = (uint32_t)km10 * 100;
            PersistMarkDirty(true);
        }

        echo = ECHO_ODO;
    }
    else if (NameIs(name, PSTR("FACTORY")))
    {
        ok = ParseUnsigned(value, 1, number) && number == 1;

        if (ok)
            PersistFactoryReset();

        echo = ECHO_FACTORY;
    }
    else
    {
        reason = F("UNKNOWN");
    }

    if (!ok)
    {
        PrintError(reason, name);
        return;
    }

    AddEcho(echo);
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
        char *end = line + 1;

        while (*end >= '0' && *end <= '9')
            end++;

        char saved = *end;
        *end = '\0';

        uint32_t parsed;
        bool valid = ParseUnsigned(line + 1, 65535, parsed);

        *end = saved;

        if (!valid || (saved != ' ' && saved != '\0'))
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
