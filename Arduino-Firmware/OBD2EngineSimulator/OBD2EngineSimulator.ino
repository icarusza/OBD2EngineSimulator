// =============================================
// OBD2 ENGINE SIMULATOR
// =============================================
//
// Arduino UNO + MCP2515 CAN shield acting as an OBD-II engine ECU.
//
// Requires the "mcp_can" library by coryjfowler (MCP_CAN_lib).
// Hardware, CAN and identity settings are in Config.h.
//
// Files:
//   Config.h            hardware pins, crystal, defaults
//   EngineState.*       engine values, DTCs, freeze frame, counters
//   CanLink.*           CAN modes, filters, ISO-TP transmit
//   ObdService.*        OBD services 01, 02, 03, 04, 06, 07, 09, 0A
//   SerialCommands.*    USB serial control protocol

#include <SPI.h>
#include <EEPROM.h>
#include <mcp_can.h>

#include "Config.h"
#include "EngineState.h"
#include "CanLink.h"
#include "ObdService.h"
#include "SerialCommands.h"

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(1000);

    EngineStateInit();

    Serial.println();
    Serial.println(F("================================"));
    Serial.println(F("       OBD2 ECU SIMULATOR"));
    Serial.println(F("================================"));
    Serial.println(F("FIRMWARE " FIRMWARE_VERSION));

    CanSetup();

    // The Windows app waits for this exact line after opening the port.
    Serial.println(F("READY"));
}

void loop()
{
    SerialPump();
    CanService();
    ObdService();
    UpdateCounters();
}
