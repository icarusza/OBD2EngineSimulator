// =============================================
// OBD2 ENGINE SIMULATOR
// =============================================
//
// Arduino UNO + MCP2515 CAN shield acting as an OBD-II engine ECU, with an
// optional transmission ECU.
//
// Requires the "mcp_can" library by coryjfowler (MCP_CAN_lib).
// Hardware, CAN, vehicle and feature settings are in Config.h.
//
// Files:
//   Config.h            hardware pins, crystal, defaults, feature switches
//   EngineState.*       engine values, DTC lifecycle, freeze frames, counters
//   Persist.*           EEPROM persistence
//   CanLink.*           CAN modes, filters, ISO-TP, request queue
//   ObdService.*        OBD services 01-0A, request dispatch
//   UdsService.*        minimal OBDonUDS (SAE J1979-2)
//   SerialCommands.*    USB serial control protocol

#include <SPI.h>
#include <mcp_can.h>

#include "Config.h"
#include "EngineState.h"
#include "Persist.h"
#include "CanLink.h"
#include "ObdService.h"
#include "UdsService.h"
#include "SerialCommands.h"

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(1000);

    Serial.println();
    Serial.println(F("OBD2 ECU SIMULATOR " FIRMWARE_VERSION));

    EngineStateInit();
    CanSetup();

    // The Windows app waits for this exact line after opening the port.
    Serial.println(F("READY"));
}

void loop()
{
    SerialPump();
    CanService();
    ObdService();
    EngineStateService();
    PersistService();
#if FEATURE_UDS
    UdsService();
#endif
}
