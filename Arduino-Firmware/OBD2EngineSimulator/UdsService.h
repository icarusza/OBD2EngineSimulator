#ifndef OBD2SIM_UDS_SERVICE_H
#define OBD2SIM_UDS_SERVICE_H

#include <Arduino.h>
#include "Config.h"
#include "EngineState.h"

#if FEATURE_UDS

// Minimal OBDonUDS (SAE J1979-2 / ISO 14229):
//   10 DiagnosticSessionControl (01 default, 03 extended)
//   3E TesterPresent
//   22 ReadDataByIdentifier (F4xx PIDs, F6xx MIDs, F8xx InfoTypes, F190 VIN)
//   19 ReadDTCInformation (01, 02, 04, 55)
//   14 ClearDiagnosticInformation (FFFFFF, FFFF33)

bool UdsHandlesService(uint8_t service);

// Returns RESULT_POSITIVE, RESULT_SILENT or a negative response code.
uint8_t UdsHandle(EcuIndex ecu, const uint8_t *request, uint8_t requestLength,
                  uint8_t *response, uint8_t &length);

// Call every loop: returns to the default session after S3 (5 s).
void UdsService();

#endif

#endif
