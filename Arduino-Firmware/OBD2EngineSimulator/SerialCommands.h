#ifndef OBD2SIM_SERIAL_COMMANDS_H
#define OBD2SIM_SERIAL_COMMANDS_H

#include <Arduino.h>

// Reads available serial input and processes every complete line.
// Safe to call while an ISO-TP transfer is waiting (CAN mode changes are
// deferred until the transfer finishes).
void SerialPump();

// Diagnostic log lines (OBD REQ/RSP, raw frames) must never block the loop:
// while the CPU waits for the 64-byte TX buffer, incoming serial data can
// overflow the 64-byte RX buffer. Returns true if a line of 'length'
// characters fits in the TX buffer now; otherwise counts it as dropped.
bool LogLineFits(uint8_t length);

#endif
