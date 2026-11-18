#ifndef OBD2SIM_PERSIST_H
#define OBD2SIM_PERSIST_H

#include <Arduino.h>

// EEPROM persistence of engine.p (DTCs, freeze frames, counters, odometer,
// vehicle information). Layout: version + struct + checksum after the CAN
// mode bytes; a mismatch resets to defaults.

void PersistLoad();
void PersistSave();          // blocking; boot only
void PersistFactoryReset();

// urgent = save on the next loop (DTC and identity changes);
// otherwise save at most every PERSIST_INTERVAL_MS (counters, odometer).
void PersistMarkDirty(bool urgent);
void PersistService();

#endif
