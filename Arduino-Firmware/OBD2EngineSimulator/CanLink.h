#ifndef OBD2SIM_CAN_LINK_H
#define OBD2SIM_CAN_LINK_H

#include <Arduino.h>
#include "Config.h"

struct CanFrame
{
    unsigned long id;
    bool extended;
    uint8_t length;
    uint8_t data[8];
};

struct CanModeInfo
{
    uint8_t mode;            // ELM327 protocol number 6..9
    bool extended;           // 29-bit identifiers
    uint8_t speed;           // MCP_CAN speed constant
    unsigned long functionalId;
    unsigned long physicalId;
    unsigned long responseId;
};

bool CanIsValidMode(uint8_t mode);

// Loads the saved mode from EEPROM (or the default) and starts the MCP2515.
void CanSetup();

// Call every loop: retries a failed init and applies a deferred mode change.
void CanService();

// Changes the CAN mode now, or after the current ISO-TP transfer.
// Returns false if the mode is invalid. 'applied' tells whether the change
// already happened; 'ok' whether the MCP2515 accepted it.
bool CanRequestMode(uint8_t mode, bool &applied, bool &ok);

uint8_t CanActiveMode();
bool CanReady();
bool IsoTpBusy();

bool CanReceive(CanFrame &frame);

// True if the frame is an OBD request for this ECU in the active mode.
bool CanIsRequest(const CanFrame &frame, bool &physical);

// Sends an ISO-TP message (single frame or First Frame + Consecutive Frames).
bool IsoTpSend(const uint8_t *payload, uint8_t length);

#endif
