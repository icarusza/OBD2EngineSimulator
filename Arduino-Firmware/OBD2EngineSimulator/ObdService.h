#ifndef OBD2SIM_OBD_SERVICE_H
#define OBD2SIM_OBD_SERVICE_H

#include <Arduino.h>
#include "CanLink.h"
#include "EngineState.h"

// Reads pending CAN frames and answers OBD and UDS requests.
void ObdService();

// Handler results: positive response, no response, or a negative response code.
const uint8_t RESULT_POSITIVE = 0x00;
const uint8_t RESULT_SILENT = 0xFF;
const uint8_t NRC_SERVICE_NOT_SUPPORTED = 0x11;
const uint8_t NRC_SUBFUNCTION_NOT_SUPPORTED = 0x12;
const uint8_t NRC_INCORRECT_LENGTH = 0x13;
const uint8_t NRC_CONDITIONS_NOT_CORRECT = 0x22;
const uint8_t NRC_REQUEST_OUT_OF_RANGE = 0x31;
const uint8_t NRC_RESPONSE_PENDING = 0x78;

// Encoders shared by the OBD services and UDS (J1979-2 DIDs).
// Return false if not supported. Range IDs (00, 20, ...) give bitmaps.
bool EncodePid(EcuIndex ecu, uint8_t pid, uint8_t *out, uint8_t &length);   // up to 4 bytes
bool EncodeMid(uint8_t mid, uint8_t *out, uint8_t &length);                 // up to 9 * records
bool EncodeInfoType(EcuIndex ecu, uint8_t infoType, uint8_t *out,
                    uint8_t &length, uint8_t &items);                       // up to 32 bytes

// Freeze frame: encodes the snapshot PIDs of the live values.
void ObdCaptureSnapshot(uint8_t *data);
bool SnapshotPidData(const Snapshot *snapshot, uint8_t pid,
                     const uint8_t *&data, uint8_t &length);

#if FEATURE_MODE08
void StartEvapTest();
#endif

#endif
