#ifndef OBD2SIM_CAN_LINK_H
#define OBD2SIM_CAN_LINK_H

#include <Arduino.h>
#include "Config.h"
#include "EngineState.h"

struct CanFrame
{
    unsigned long id;
    bool extended;
    uint8_t length;
    uint8_t data[8];
};

// Who an incoming frame is addressed to.
const uint8_t TARGET_NONE = 0xFF;
const uint8_t TARGET_FUNCTIONAL = 0xFE;   // otherwise an EcuIndex

bool CanIsValidMode(uint8_t mode);

// Loads the saved mode from EEPROM (or the default) and starts the MCP2515.
void CanSetup();

// Call every loop: retries a failed init, applies a deferred mode change,
// and times out a stalled multi-frame request.
void CanService();

// Changes the CAN mode now, or after the current transfer or wait.
// Returns false if the mode is invalid. 'applied' tells whether the change
// already happened; 'ok' whether the MCP2515 accepted it.
bool CanRequestMode(uint8_t mode, bool &applied, bool &ok);

uint8_t CanActiveMode();
bool CanReady();
bool CanBusy();            // inside a transfer or wait

unsigned long CanResponseId(EcuIndex ecu);

// Next frame to handle: frames queued during waits first, then the bus.
bool CanNextFrame(CanFrame &frame);

uint8_t CanRequestTarget(const CanFrame &frame);

// Sends an ISO-TP message from 'ecu' (single frame or multi-frame).
bool IsoTpSend(EcuIndex ecu, const uint8_t *payload, uint8_t length);

// Waits while still serving serial input, queueing requests and collecting
// multi-frame request data, so nothing is dropped during a delay.
void CanWait(unsigned long ms);

// ISO-TP receive (multi-frame requests, UDS). Returns true if the frame
// belonged to a multi-frame request (First or Consecutive Frame).
bool IsoTpReceive(const CanFrame &frame, uint8_t target);

// Takes a completed multi-frame request, if any.
bool IsoTpTakeRequest(EcuIndex &ecu, uint8_t *buffer, uint8_t &length);

#endif
