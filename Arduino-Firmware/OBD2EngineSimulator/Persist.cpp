#include "Persist.h"
#include "EngineState.h"
#include <EEPROM.h>
#include <avr/eeprom.h>
#include <string.h>

// Two copies (A/B slots) of engine.p, each followed by a sequence byte and a
// checksum. A save always goes to the older slot, which is marked invalid
// (version byte 0xFF) while it is written, so a reset or power loss during a
// save keeps the previous copy.
//
// Saving is incremental: PersistService() does at most one EEPROM write per
// call, and only when the EEPROM is idle. A blocking save (~3.3 ms per byte)
// would stall the loop long enough for serial input to overflow the UNO's
// 64-byte receive buffer.

static const uint16_t STATE_SIZE = sizeof(PersistentState);
static const uint16_t SLOT_SIZE = STATE_SIZE + 2;  // + sequence + checksum

static_assert(EEPROM_ADDR_STATE + 2 * SLOT_SIZE <= 1024,
              "persisted state does not fit the UNO's 1 KB EEPROM twice");

enum SaveStep : uint8_t
{
    STEP_IDLE = 0,
    STEP_INVALIDATE,   // mark the target slot invalid
    STEP_DATA,         // copy changed bytes 1..n
    STEP_SEQUENCE,
    STEP_VERSION,      // makes the slot valid (together with the checksum)
    STEP_CHECKSUM
};

static bool dirty = false;
static bool urgent = false;
static unsigned long lastSaveMs = 0;

static uint8_t activeSlot = 0;       // slot holding the newest valid copy
static uint8_t activeSequence = 0;
static uint8_t targetSlot = 1;
static SaveStep step = STEP_IDLE;
static uint16_t cursor = 0;

static int SlotAddress(uint8_t slot)
{
    return EEPROM_ADDR_STATE + slot * SLOT_SIZE;
}

static int SequenceAddress(uint8_t slot) { return SlotAddress(slot) + STATE_SIZE; }
static int ChecksumAddress(uint8_t slot) { return SlotAddress(slot) + STATE_SIZE + 1; }

static uint8_t ChecksumStep(uint8_t sum, uint8_t value)
{
    return (uint8_t)((sum << 1) | (sum >> 7)) ^ value;  // rotate-xor
}

// Checksum over what is stored (state + sequence): values can change in RAM
// while a save runs, so the checksum must describe the EEPROM content.
static uint8_t StoredChecksum(uint8_t slot)
{
    uint8_t sum = 0x5A;
    int address = SlotAddress(slot);

    for (uint16_t i = 0; i < STATE_SIZE + 1; i++)
        sum = ChecksumStep(sum, EEPROM.read(address + i));

    return sum;
}

static bool SlotValid(uint8_t slot)
{
    // The version byte is the first byte of PersistentState.
    return EEPROM.read(SlotAddress(slot)) == PERSIST_VERSION &&
           EEPROM.read(ChecksumAddress(slot)) == StoredChecksum(slot);
}

static void SetDefaults()
{
    memset(&engine.p, 0, sizeof(PersistentState));

    engine.p.version = PERSIST_VERSION;
    strncpy(engine.p.vin, DEFAULT_VIN, sizeof(engine.p.vin) - 1);
    strncpy(engine.p.calibrationId, DEFAULT_CALID, sizeof(engine.p.calibrationId) - 1);
    strncpy(engine.p.ecuName, DEFAULT_ECU_NAME, sizeof(engine.p.ecuName) - 1);
    strncpy(engine.p.esn, DEFAULT_ESN, sizeof(engine.p.esn) - 1);
    engine.p.tcmEnabled = 1;
}

// Blocking save of a complete copy into slot 0; only used at boot.
void PersistSave()
{
    EEPROM.update(SlotAddress(1), 0xFF);  // slot 1 invalid
    EEPROM.put(SlotAddress(0), engine.p);
    EEPROM.update(SequenceAddress(0), 0);
    EEPROM.update(ChecksumAddress(0), StoredChecksum(0));

    activeSlot = 0;
    activeSequence = 0;
    step = STEP_IDLE;
    dirty = false;
    urgent = false;
    lastSaveMs = millis();
}

void PersistLoad()
{
    bool valid0 = SlotValid(0);
    bool valid1 = SlotValid(1);

    if (!valid0 && !valid1)
    {
        Serial.println(F("EEPROM state invalid, using defaults"));
        SetDefaults();
        PersistSave();
        return;
    }

    uint8_t sequence0 = EEPROM.read(SequenceAddress(0));
    uint8_t sequence1 = EEPROM.read(SequenceAddress(1));

    if (valid0 && valid1)
        activeSlot = (int8_t)(sequence1 - sequence0) > 0 ? 1 : 0;  // wrap-safe newer
    else
        activeSlot = valid1 ? 1 : 0;

    activeSequence = activeSlot == 0 ? sequence0 : sequence1;
    EEPROM.get(SlotAddress(activeSlot), engine.p);
}

void PersistFactoryReset()
{
    SetDefaults();
    PersistMarkDirty(true);
}

void PersistMarkDirty(bool isUrgent)
{
    dirty = true;

    if (isUrgent)
        urgent = true;
}

void PersistService()
{
    if (step == STEP_IDLE)
    {
        if (!urgent && !(dirty && millis() - lastSaveMs >= PERSIST_INTERVAL_MS))
            return;

        dirty = false;
        urgent = false;
        targetSlot = activeSlot ^ 1;
        step = STEP_INVALIDATE;
    }

    if (!eeprom_is_ready())
        return;

    const uint8_t *bytes = (const uint8_t *)&engine.p;
    int base = SlotAddress(targetSlot);

    switch (step)
    {
        case STEP_INVALIDATE:
            EEPROM.update(base, 0xFF);
            cursor = 1;
            step = STEP_DATA;
            return;

        case STEP_DATA:
            // Find the next byte that differs and start writing it (non-blocking).
            while (cursor < STATE_SIZE)
            {
                uint16_t index = cursor++;

                if (EEPROM.read(base + index) != bytes[index])
                {
                    EEPROM.write(base + index, bytes[index]);
                    return;
                }
            }

            step = STEP_SEQUENCE;
            return;

        case STEP_SEQUENCE:
            EEPROM.update(SequenceAddress(targetSlot), (uint8_t)(activeSequence + 1));
            step = STEP_VERSION;
            return;

        case STEP_VERSION:
            EEPROM.update(base, PERSIST_VERSION);
            step = STEP_CHECKSUM;
            return;

        case STEP_CHECKSUM:
            // Changes made during the save marked it dirty and go to the other
            // slot next time.
            EEPROM.update(ChecksumAddress(targetSlot), StoredChecksum(targetSlot));
            activeSlot = targetSlot;
            activeSequence++;
            step = STEP_IDLE;
            lastSaveMs = millis();
            return;

        default:
            step = STEP_IDLE;
            return;
    }
}
