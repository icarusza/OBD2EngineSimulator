#ifndef OBD2SIM_CONFIG_H
#define OBD2SIM_CONFIG_H

#include <Arduino.h>
#include <mcp_can.h>

// =============================================
// FIRMWARE
// =============================================

#define FIRMWARE_VERSION "2.0.0"

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

// EEPROM layout for the saved CAN mode.
const int EEPROM_ADDR_MAGIC = 0;
const int EEPROM_ADDR_CAN_MODE = 1;
const uint8_t EEPROM_MAGIC = 0xC7;

// ISO-TP timing.
const unsigned long ISOTP_FLOW_CONTROL_TIMEOUT_MS = 1000;  // N_Bs
const uint8_t ISOTP_MAX_WAIT_FRAMES = 10;                  // N_WFTmax
const uint8_t ISOTP_PADDING = 0xAA;

// Retry interval when the MCP2515 fails to initialise.
const unsigned long CAN_INIT_RETRY_MS = 2000;

// =============================================
// OBD IDENTITY DEFAULTS (RAM only, reset at boot)
// =============================================

#define DEFAULT_VIN "1M8GDM9AXKP042788"     // exactly 17 characters (SAE sample VIN)
#define DEFAULT_CALID "OBDSIM-CAL-0001"     // up to 16 characters
#define DEFAULT_ECU_NAME "ECM-EngineControl" // up to 20 characters

// PID 1C: OBD standard this vehicle conforms to.
//   0x01 = OBD-II (CARB), 0x03 = OBD and OBD-II, 0x06 = EOBD
const uint8_t OBD_STANDARD = 0x06;

// =============================================
// LIMITS
// =============================================

const uint8_t MAX_DTCS_PER_LIST = 8;
const uint8_t SERIAL_LINE_LENGTH = 200;
const uint8_t ISOTP_MAX_PAYLOAD = 48;

#endif
