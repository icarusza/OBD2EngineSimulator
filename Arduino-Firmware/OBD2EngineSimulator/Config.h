#ifndef OBD2SIM_CONFIG_H
#define OBD2SIM_CONFIG_H

#include <Arduino.h>
#include <mcp_can.h>

// =============================================
// FIRMWARE
// =============================================

#define FIRMWARE_VERSION "3.0.0"

// =============================================
// FEATURE SWITCHES
// =============================================
// The Arduino UNO has 32 KB of flash. If a build does not fit, switch
// features off in this order: MODE08, MODE06_EXT, IPT, EXTRA_PIDS, UDS, TCM.
// README.md lists the measured flash cost of each switch.

#ifndef FEATURE_TCM
#define FEATURE_TCM 1          // second ECU (transmission) on 7E9 / 18DAF118
#endif

#ifndef FEATURE_UDS
#define FEATURE_UDS 1          // minimal OBDonUDS (10, 3E, 22, 19, 14) + multi-frame requests
#endif

#ifndef FEATURE_EXTRA_PIDS
#define FEATURE_EXTRA_PIDS 1   // EGR/EVAP, catalyst temperature, torque, sensor PIDs
#endif

#ifndef FEATURE_IPT
#define FEATURE_IPT 1          // Mode 09 InfoType 08 in-use performance tracking
#endif

#ifndef FEATURE_MODE06_EXT
#define FEATURE_MODE06_EXT 1   // Mode 06 catalyst, EGR, EVAP and misfire MIDs
#endif

#ifndef FEATURE_MODE08
#define FEATURE_MODE08 1       // Mode 08 EVAP leak test
#endif

// =============================================
// HARDWARE
// =============================================

// MCP2515 chip-select pin (10 on most UNO CAN shields).
const uint8_t SPI_CS_PIN = 10;

// MCP2515 crystal. Most UNO shields use 16 MHz; many small blue
// MCP2515 modules use 8 MHz. A wrong value gives the wrong CAN bitrate.
#define MCP_CLOCK MCP_16MHZ
// #define MCP_CLOCK MCP_8MHZ

const unsigned long SERIAL_BAUD = 115200;

// =============================================
// CAN DEFAULTS
// =============================================

// CAN modes use ELM327 protocol numbering:
//   6 = ISO 15765-4 CAN, 11-bit, 500 kbit/s
//   7 = ISO 15765-4 CAN, 29-bit, 500 kbit/s
//   8 = ISO 15765-4 CAN, 11-bit, 250 kbit/s
//   9 = ISO 15765-4 CAN, 29-bit, 250 kbit/s
// Used when the EEPROM holds no valid mode. CAN:<n> changes and saves it.
const uint8_t DEFAULT_CAN_MODE = 6;

// 29-bit ECU addresses (18DA<addr>F1 / 18DAF1<addr>).
const uint8_t ECM_ADDRESS_29 = 0x10;
const uint8_t TCM_ADDRESS_29 = 0x18;

// EEPROM layout: bytes 0-1 hold the CAN mode, the persisted state follows.
const int EEPROM_ADDR_MAGIC = 0;
const int EEPROM_ADDR_CAN_MODE = 1;
const int EEPROM_ADDR_STATE = 2;
const uint8_t EEPROM_MAGIC = 0xC7;

// ISO-TP timing.
const unsigned long ISOTP_FLOW_CONTROL_TIMEOUT_MS = 1000;  // N_Bs
const unsigned long ISOTP_CONSECUTIVE_TIMEOUT_MS = 1000;   // N_Cr
const uint8_t ISOTP_MAX_WAIT_FRAMES = 10;                  // N_WFTmax
const uint8_t ISOTP_PADDING = 0xAA;

// Response timing: a reply later than this gets "response pending" (7F xx 78)
// first; further 78s are repeated within P2* (5000 ms).
const unsigned long RESPONSE_PENDING_AFTER_MS = 40;
const unsigned long RESPONSE_PENDING_REPEAT_MS = 4000;

// Retry interval when the MCP2515 fails to initialise.
const unsigned long CAN_INIT_RETRY_MS = 2000;

// =============================================
// VEHICLE DEFAULTS (persisted in EEPROM after the first change)
// =============================================

#define DEFAULT_VIN "1M8GDM9AXKP042788"     // exactly 17 characters (SAE sample VIN)
#define DEFAULT_CALID "OBDSIM-CAL-0001"     // up to 16 characters
#define DEFAULT_ECU_NAME "ECM-EngineControl" // up to 20 characters
#define DEFAULT_ESN "OBDSIM-ENG-00001"      // up to 16 characters
#define TCM_ECU_NAME "TCM-TransmissionCtl"   // up to 20 characters

// PID 1C: OBD standard this vehicle conforms to.
//   0x01 = OBD-II (CARB), 0x03 = OBD and OBD-II, 0x06 = EOBD
const uint8_t OBD_STANDARD = 0x06;

// Engine description (selects which PIDs exist).
#define DUAL_BANK 1                      // V engine: bank 2 trims and O2 sensors
const uint8_t FUEL_TYPE = 0x01;          // PID 51: 0x01 gasoline
const uint8_t ETHANOL_PERCENT = 10;      // PID 52
const uint16_t REFERENCE_TORQUE_NM = 250; // PID 63

// =============================================
// DRIVE CYCLES AND DTCS
// =============================================

// A drive cycle counts (for DTC confirmation, MIL-off and readiness) when
// the engine ran at least this long before the ignition went off.
const unsigned long CYCLE_MIN_RUN_S = 60;

const uint8_t CYCLES_TO_CONFIRM = 2;     // consecutive failing cycles
const uint8_t CYCLES_TO_MIL_OFF = 3;     // consecutive passing cycles
const uint8_t WARMUPS_TO_ERASE = 40;     // warm-ups after MIL off

const uint8_t MAX_DTCS = 10;             // shared by ECM and TCM
const uint8_t MAX_SNAPSHOTS = 3;         // freeze frames

// Counters and odometer are saved at most this often (and on ignition off).
const unsigned long PERSIST_INTERVAL_MS = 60000;

// =============================================
// BUFFERS
// =============================================

const uint8_t SERIAL_LINE_LENGTH = 200;
const uint8_t ISOTP_MAX_PAYLOAD = 48;    // largest response (UDS 19 02 with 10 DTCs: 43)
const uint8_t ISOTP_RX_BUFFER = 32;      // largest multi-frame request (UDS)
const uint8_t REQUEST_QUEUE_SIZE = 4;

#endif
