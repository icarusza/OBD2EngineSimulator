#ifndef OBD2SIM_OBD_SERVICE_H
#define OBD2SIM_OBD_SERVICE_H

#include <Arduino.h>
#include "CanLink.h"

// Reads all pending CAN frames and answers OBD requests.
void ObdService();

#endif
