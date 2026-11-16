
#include <SPI.h>
#include <mcp_can.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdint.h>

// =============================================
// HARDWARE CONFIGURATION
// =============================================

const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);

const unsigned long CAN_FUNCTIONAL_REQUEST = 0x18DB33F1UL;
const unsigned long CAN_PHYSICAL_REQUEST   = 0x18DA10F1UL;
const unsigned long CAN_ECU_RESPONSE      = 0x18DAF110UL;

const byte CAN_EXTENDED = 1;

// =============================================
// ENGINE PARAMETERS
// =============================================

float engineRPM = 850;
float manifoldPressure = 85;

float shortTermFuelTrim = 3.1;
float longTermFuelTrim = -2.3;

float commandedAFR = 14.7;
float measuredAFR = 14.7;

float coolantTemperature = 90;
float intakeTemperature = 25;

float throttlePosition = 0;
float vehicleSpeed = 0;
float engineLoad = 20;

float massAirFlow = 3.5;
float ignitionTiming = 10;

// =============================================
// SERIAL COMMAND BUFFER
// =============================================

char serialBuffer[64];
byte serialPosition = 0;

// =============================================
// FUNCTION DECLARATIONS
// =============================================

void ReadSerialCommands();
void ProcessSerialCommand(char *command);
void ReadCANMessages();
void ProcessOBDRequest(byte pid);
void SendOBDResponse(byte pid, byte *responseData, byte dataLength);

// =============================================
// STARTUP
// =============================================

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("================================");
    Serial.println("       OBD2 ECU SIMULATOR");
    Serial.println("================================");

    Serial.println("Initialising MCP2515...");

    if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
    {
        Serial.println("MCP2515 INITIALISED");
    }
    else
    {
        Serial.println("MCP2515 INITIALISATION FAILED");

        while (1)
        {
            delay(1000);
        }
    }

    CAN.setMode(MCP_NORMAL);

    Serial.println("CAN Speed: 500 kbit/s");
    Serial.println("CAN Format: 29-bit configured");
    Serial.println("CAN Diagnostics: ENABLED");

    Serial.println("--------------------------------");
    Serial.println("SUPPORTED PARAMETERS:");
    Serial.println("RPM: 0-12000");
    Serial.println("MAP: 0-170 kPa");
    Serial.println("STFT");
    Serial.println("LTFT");
    Serial.println("AFR");
    Serial.println("MEASURED_AFR");
    Serial.println("ECT");
    Serial.println("IAT");
    Serial.println("TPS");
    Serial.println("SPEED");
    Serial.println("LOAD");
    Serial.println("MAF: 0-254 g/s");
    Serial.println("TIMING");

    Serial.println("--------------------------------");
    Serial.println("OBD2 ECU READY");
    Serial.println("--------------------------------");
}

// =============================================
// MAIN LOOP
// =============================================

void loop()
{
    ReadSerialCommands();
    ReadCANMessages();
}

// =============================================
// SERIAL COMMAND RECEPTION
// =============================================

void ReadSerialCommands()
{
    while (Serial.available())
    {
        char incoming = Serial.read();

        if (incoming == '\n')
        {
            serialBuffer[serialPosition] = '\0';

            ProcessSerialCommand(serialBuffer);

            serialPosition = 0;
        }
        else if (incoming != '\r')
        {
            if (serialPosition < sizeof(serialBuffer) - 1)
            {
                serialBuffer[serialPosition++] = incoming;
            }
            else
            {
                serialPosition = 0;
                Serial.println("ERROR: Serial command too long");
            }
        }
    }
}

// =============================================
// PROCESS WINDOWS COMMANDS
// =============================================

void ProcessSerialCommand(char *command)
{
    char *separator = strchr(command, ':');

    if (separator == NULL)
        return;

    *separator = '\0';

    char *name = command;
    char *valueText = separator + 1;

    if (*valueText == '\0')
        return;

    char *endPointer;
    float value = strtod(valueText, &endPointer);

    if (endPointer == valueText ||
        *endPointer != '\0' ||
        !isfinite(value))
    {
        Serial.println("ERROR: Invalid parameter value");
        return;
    }

    if (strcmp(name, "RPM") == 0)
        engineRPM = constrain(value, 0, 12000);

    else if (strcmp(name, "MAP") == 0)
        manifoldPressure = constrain(value, 0, 170);

    else if (strcmp(name, "STFT") == 0)
        shortTermFuelTrim = constrain(value, -100, 99.2);

    else if (strcmp(name, "LTFT") == 0)
        longTermFuelTrim = constrain(value, -100, 99.2);

    else if (strcmp(name, "AFR") == 0)
        commandedAFR = constrain(value, 8, 20);

    else if (strcmp(name, "MEASURED_AFR") == 0)
        measuredAFR = constrain(value, 8, 20);

    else if (strcmp(name, "ECT") == 0)
        coolantTemperature = constrain(value, -40, 215);

    else if (strcmp(name, "IAT") == 0)
        intakeTemperature = constrain(value, -40, 215);

    else if (strcmp(name, "TPS") == 0)
        throttlePosition = constrain(value, 0, 100);

    else if (strcmp(name, "SPEED") == 0)
        vehicleSpeed = constrain(value, 0, 255);

    else if (strcmp(name, "LOAD") == 0)
        engineLoad = constrain(value, 0, 100);

    else if (strcmp(name, "MAF") == 0)
        massAirFlow = constrain(value, 0, 254);

    else if (strcmp(name, "TIMING") == 0)
        ignitionTiming = constrain(value, -64, 63.5);

    else
    {
        Serial.print("ERROR: Unknown parameter: ");
        Serial.println(name);
    }
}

// =============================================
// CAN MESSAGE RECEPTION + DIAGNOSTICS
// =============================================

void ReadCANMessages()
{
    if (CAN.checkReceive() != CAN_MSGAVAIL)
        return;

    unsigned long receivedID = 0;
    byte extended = 0;
    byte length = 0;
    byte data[8] = {0};

    CAN.readMsgBuf(&receivedID, &extended, &length, data);

    // -----------------------------------------
    // PRINT EVERY RECEIVED CAN FRAME
    // -----------------------------------------

    Serial.print("CAN RX | ID: 0x");
    Serial.print(receivedID, HEX);

    Serial.print(" | FORMAT: ");
    Serial.print(extended ? "29-bit" : "11-bit");

    Serial.print(" | LENGTH: ");
    Serial.print(length);

    Serial.print(" | DATA: ");

    for (byte i = 0; i < length && i < 8; i++)
    {
        if (data[i] < 0x10)
            Serial.print("0");

        Serial.print(data[i], HEX);
        Serial.print(" ");
    }

    Serial.println();

    // -----------------------------------------
    // EXISTING OBD REQUEST FILTERING
    // -----------------------------------------

    if (!extended)
        return;

    if (receivedID != CAN_FUNCTIONAL_REQUEST &&
        receivedID != CAN_PHYSICAL_REQUEST)
        return;

    if (length < 3)
        return;

    if (data[1] != 0x01)
        return;

    Serial.print("OBD REQUEST | PID: 0x");
    Serial.println(data[2], HEX);

    ProcessOBDRequest(data[2]);
}

// =============================================
// OBD PID REQUEST PROCESSING
// =============================================

void ProcessOBDRequest(byte pid)
{
    byte responseData[4] = {0, 0, 0, 0};
    byte dataLength = 0;

    switch (pid)
    {
        // SUPPORTED PIDS 01-20
        case 0x00:
            responseData[0] = 0x1E;
            responseData[1] = 0x3F;
            responseData[2] = 0xA0;
            responseData[3] = 0x01;
            dataLength = 4;
            break;

        // SUPPORTED PIDS 21-40
        case 0x20:
            responseData[0] = 0x10;
            responseData[1] = 0x00;
            responseData[2] = 0x00;
            responseData[3] = 0x01;
            dataLength = 4;
            break;

        // SUPPORTED PIDS 41-60
        case 0x40:
            responseData[0] = 0x10;
            responseData[1] = 0x00;
            responseData[2] = 0x00;
            responseData[3] = 0x00;
            dataLength = 4;
            break;

        // OXYGEN SENSOR LOCATIONS - PID 13
        case 0x13:
            responseData[0] = 0x01;
            dataLength = 1;
            break;

        // ENGINE LOAD - PID 04
        case 0x04:
            responseData[0] =
                (byte)round(engineLoad * 255.0 / 100.0);
            dataLength = 1;
            break;

        // COOLANT TEMPERATURE - PID 05
        case 0x05:
            responseData[0] =
                (byte)round(coolantTemperature + 40);
            dataLength = 1;
            break;

        // SHORT TERM FUEL TRIM - PID 06
        case 0x06:
            responseData[0] =
                (byte)round((shortTermFuelTrim + 100.0) * 128.0 / 100.0);
            dataLength = 1;
            break;

        // LONG TERM FUEL TRIM - PID 07
        case 0x07:
            responseData[0] =
                (byte)round((longTermFuelTrim + 100.0) * 128.0 / 100.0);
            dataLength = 1;
            break;

        // MANIFOLD PRESSURE - PID 0B
        case 0x0B:
            responseData[0] = (byte)round(manifoldPressure);
            dataLength = 1;
            break;

        // ENGINE RPM - PID 0C
        case 0x0C:
        {
            unsigned int raw =
                (unsigned int)round(engineRPM * 4.0);

            responseData[0] = highByte(raw);
            responseData[1] = lowByte(raw);
            dataLength = 2;
            break;
        }

        // VEHICLE SPEED - PID 0D
        case 0x0D:
            responseData[0] = (byte)round(vehicleSpeed);
            dataLength = 1;
            break;

        // IGNITION TIMING - PID 0E
        case 0x0E:
            responseData[0] =
                (byte)round((ignitionTiming + 64.0) * 2.0);
            dataLength = 1;
            break;

        // INTAKE AIR TEMPERATURE - PID 0F
        case 0x0F:
            responseData[0] =
                (byte)round(intakeTemperature + 40);
            dataLength = 1;
            break;

        // MASS AIR FLOW - PID 10
        case 0x10:
        {
            unsigned int raw =
                (unsigned int)round(massAirFlow * 100.0);

            responseData[0] = highByte(raw);
            responseData[1] = lowByte(raw);
            dataLength = 2;
            break;
        }

        // THROTTLE POSITION - PID 11
        case 0x11:
            responseData[0] =
                (byte)round(throttlePosition * 255.0 / 100.0);
            dataLength = 1;
            break;

        // WIDE-RANGE OXYGEN SENSOR - PID 24
        case 0x24:
        {
            float lambda = measuredAFR / 14.7;

            uint16_t rawLambda =
                (uint16_t)round(lambda * 65535.0 / 2.0);

            responseData[0] = highByte(rawLambda);
            responseData[1] = lowByte(rawLambda);

            uint16_t rawVoltage =
                (uint16_t)round(0.5 * 65535.0 / 8.0);

            responseData[2] = highByte(rawVoltage);
            responseData[3] = lowByte(rawVoltage);

            dataLength = 4;
            break;
        }

        // COMMANDED EQUIVALENCE RATIO - PID 44
        case 0x44:
        {
            float equivalenceRatio = 14.7 / commandedAFR;

            unsigned int raw =
                (unsigned int)round(equivalenceRatio * 32768.0);

            responseData[0] = highByte(raw);
            responseData[1] = lowByte(raw);
            dataLength = 2;
            break;
        }

        default:
            Serial.print("UNSUPPORTED PID: 0x");
            Serial.println(pid, HEX);
            return;
    }

    SendOBDResponse(pid, responseData, dataLength);
}

// =============================================
// SEND OBD RESPONSE + TRANSMISSION DIAGNOSTICS
// =============================================

void SendOBDResponse(byte pid, byte *responseData, byte dataLength)
{
    byte response[8] = {0};

    response[0] = 2 + dataLength;
    response[1] = 0x41;
    response[2] = pid;

    for (byte i = 0; i < dataLength; i++)
    {
        response[3 + i] = responseData[i];
    }

    byte result = CAN.sendMsgBuf(
        CAN_ECU_RESPONSE,
        CAN_EXTENDED,
        8,
        response
    );

    // -----------------------------------------
    // PRINT TRANSMISSION RESULT
    // -----------------------------------------

    Serial.print("CAN TX | ID: 0x");
    Serial.print(CAN_ECU_RESPONSE, HEX);

    Serial.print(" | DATA: ");

    for (byte i = 0; i < 8; i++)
    {
        if (response[i] < 0x10)
            Serial.print("0");

        Serial.print(response[i], HEX);
        Serial.print(" ");
    }

    if (result == CAN_OK)
    {
        Serial.println("| SUCCESS");
    }
    else
    {
        Serial.print("| FAILED | ERROR: ");
        Serial.println(result);
    }
}
