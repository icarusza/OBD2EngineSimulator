#include "UdsService.h"

#if FEATURE_UDS

#include "ObdService.h"
#include <string.h>

// DTC format identifier for 3-byte DTCs (2-byte code + failure type byte),
// as used by SAE J1979-2: SAE_J2012-DA_DTCFormat_04.
const uint8_t DTC_FORMAT = 0x04;

// All eight ISO 14229 status bits are supported.
const uint8_t STATUS_AVAILABILITY_MASK = 0xFF;

const unsigned long S3_TIMEOUT_MS = 5000;

static uint8_t session[ECU_COUNT] = { 0x01, 0x01 };
static unsigned long lastRequestMs[ECU_COUNT];

// Freeze frame data returned by 19 04, as F4xx DIDs.
static const uint8_t SNAPSHOT_DIDS[] PROGMEM =
{
    0x0C, 0x0D, 0x05, 0x04, 0x11, 0x0B, 0x06, 0x07
};

bool UdsHandlesService(uint8_t service)
{
    return service == 0x10 || service == 0x3E || service == 0x22 ||
           service == 0x19 || service == 0x14;
}

void UdsService()
{
    for (uint8_t ecu = 0; ecu < ECU_COUNT; ecu++)
    {
        if (session[ecu] != 0x01 && millis() - lastRequestMs[ecu] > S3_TIMEOUT_MS)
            session[ecu] = 0x01;
    }
}

static void Put16(uint8_t *out, uint16_t value)
{
    out[0] = value >> 8;
    out[1] = value & 0xFF;
}

static EcuIndex OwnerOf(const DtcRecord &record)
{
    return (record.flags & DTC_FLAG_TCM) ? ECU_TCM : ECU_ECM;
}

// =============================================
// 10 / 3E
// =============================================

static uint8_t SessionControl(EcuIndex ecu, const uint8_t *request, uint8_t requestLength,
                              uint8_t *response, uint8_t &length)
{
    if (requestLength != 2)
        return NRC_INCORRECT_LENGTH;

    uint8_t subfunction = request[1] & 0x7F;

    if (subfunction != 0x01 && subfunction != 0x03)
        return NRC_SUBFUNCTION_NOT_SUPPORTED;

    session[ecu] = subfunction;

    if (request[1] & 0x80)
        return RESULT_SILENT;  // suppress positive response

    // P2 = 50 ms, P2* = 5000 ms (10 ms units).
    response[0] = 0x50;
    response[1] = subfunction;
    Put16(response + 2, 50);
    Put16(response + 4, 500);
    length = 6;
    return RESULT_POSITIVE;
}

static uint8_t TesterPresent(const uint8_t *request, uint8_t requestLength,
                             uint8_t *response, uint8_t &length)
{
    if (requestLength != 2)
        return NRC_INCORRECT_LENGTH;

    if ((request[1] & 0x7F) != 0x00)
        return NRC_SUBFUNCTION_NOT_SUPPORTED;

    if (request[1] & 0x80)
        return RESULT_SILENT;

    response[0] = 0x7E;
    response[1] = 0x00;
    length = 2;
    return RESULT_POSITIVE;
}

// =============================================
// 22 ReadDataByIdentifier
// =============================================

static bool ReadDid(EcuIndex ecu, uint16_t did, uint8_t *data, uint8_t &dataLength)
{
    uint8_t high = did >> 8;
    uint8_t low = did & 0xFF;
    uint8_t items;

    if (did == 0xF190)
        return EncodeInfoType(ecu, 0x02, data, dataLength, items);

    switch (high)
    {
        case 0xF4: return EncodePid(ecu, low, data, dataLength);
        case 0xF6: return ecu == ECU_ECM && EncodeMid(low, data, dataLength);
        case 0xF8: return EncodeInfoType(ecu, low, data, dataLength, items);
        default: return false;
    }
}

static uint8_t ReadDataByIdentifier(EcuIndex ecu, const uint8_t *request, uint8_t requestLength,
                                    uint8_t *response, uint8_t &length)
{
    if (requestLength < 3 || (requestLength - 1) % 2 != 0)
        return NRC_INCORRECT_LENGTH;

    response[0] = 0x62;
    length = 1;

    for (uint8_t i = 1; i + 1 < requestLength; i += 2)
    {
        uint16_t did = ((uint16_t)request[i] << 8) | request[i + 1];
        uint8_t data[32];
        uint8_t dataLength;

        if (!ReadDid(ecu, did, data, dataLength))
            continue;

        if (length + 2 + dataLength > ISOTP_MAX_PAYLOAD)
            break;

        Put16(&response[length], did);
        memcpy(&response[length + 2], data, dataLength);
        length += 2 + dataLength;
    }

    return length > 1 ? RESULT_POSITIVE : NRC_REQUEST_OUT_OF_RANGE;
}

// =============================================
// 19 ReadDTCInformation
// =============================================

static uint8_t PutDtc(uint8_t *out, const DtcRecord &record)
{
    Put16(out, record.code);
    out[2] = 0x00;  // failure type byte: not used
    out[3] = record.status;
    return 4;
}

static uint8_t ReadDtcInformation(EcuIndex ecu, const uint8_t *request, uint8_t requestLength,
                                  uint8_t *response, uint8_t &length)
{
    if (requestLength < 2)
        return NRC_INCORRECT_LENGTH;

    uint8_t subfunction = request[1] & 0x7F;

    response[0] = 0x59;
    response[1] = subfunction;

    switch (subfunction)
    {
        case 0x01:  // number of DTCs by status mask
        case 0x02:  // DTCs by status mask
        {
            if (requestLength != 3)
                return NRC_INCORRECT_LENGTH;

            uint8_t mask = request[2];
            uint16_t count = 0;

            response[2] = STATUS_AVAILABILITY_MASK;
            length = 3;

            for (uint8_t i = 0; i < MAX_DTCS; i++)
            {
                const DtcRecord &record = engine.p.dtcs[i];

                if (record.code == 0 || OwnerOf(record) != ecu || !(record.status & mask))
                    continue;

                count++;

                if (subfunction == 0x02 && length + 4 <= ISOTP_MAX_PAYLOAD)
                    length += PutDtc(&response[length], record);
            }

            if (subfunction == 0x01)
            {
                response[3] = DTC_FORMAT;
                Put16(&response[4], count);
                length = 6;
            }

            return RESULT_POSITIVE;
        }

        case 0x04:  // snapshot (freeze frame) by DTC
        {
            if (requestLength != 6)
                return NRC_INCORRECT_LENGTH;

            uint16_t code = ((uint16_t)request[2] << 8) | request[3];
            uint8_t recordNumber = request[5];
            const DtcRecord *record = NULL;

            for (uint8_t i = 0; i < MAX_DTCS; i++)
            {
                if (engine.p.dtcs[i].code == code && OwnerOf(engine.p.dtcs[i]) == ecu)
                    record = &engine.p.dtcs[i];
            }

            if (record == NULL || (recordNumber != 0x00 && recordNumber != 0xFF))
                return NRC_REQUEST_OUT_OF_RANGE;

            length = 2 + PutDtc(&response[2], *record);

            const Snapshot *snapshot = FindSnapshot(code);

            if (snapshot == NULL)
                return RESULT_POSITIVE;  // DTC known, no snapshot stored

            uint8_t countIndex = length + 1;
            response[length] = 0x00;  // record number
            response[countIndex] = 0;
            length += 2;

            for (uint8_t i = 0; i < sizeof(SNAPSHOT_DIDS); i++)
            {
                uint8_t pid = pgm_read_byte(&SNAPSHOT_DIDS[i]);
                const uint8_t *data;
                uint8_t dataLength;

                if (!SnapshotPidData(snapshot, pid, data, dataLength) ||
                    length + 2 + dataLength > ISOTP_MAX_PAYLOAD)
                    continue;

                response[length] = 0xF4;
                response[length + 1] = pid;
                memcpy(&response[length + 2], data, dataLength);
                length += 2 + dataLength;
                response[countIndex]++;
            }

            return RESULT_POSITIVE;
        }

        case 0x55:  // WWH-OBD DTCs with permanent status
        {
            if (requestLength != 3)
                return NRC_INCORRECT_LENGTH;

            response[2] = request[2];  // functional group identifier
            response[3] = STATUS_AVAILABILITY_MASK;
            response[4] = DTC_FORMAT;
            length = 5;

            for (uint8_t i = 0; i < MAX_DTCS; i++)
            {
                const DtcRecord &record = engine.p.dtcs[i];

                if (DtcInList(record, DTC_LIST_PERMANENT) && OwnerOf(record) == ecu &&
                    length + 4 <= ISOTP_MAX_PAYLOAD)
                    length += PutDtc(&response[length], record);
            }

            return RESULT_POSITIVE;
        }

        default:
            return NRC_SUBFUNCTION_NOT_SUPPORTED;
    }
}

// =============================================
// 14 ClearDiagnosticInformation
// =============================================

static uint8_t ClearDtcs(EcuIndex ecu, const uint8_t *request, uint8_t requestLength,
                         uint8_t *response, uint8_t &length)
{
    if (requestLength != 4)
        return NRC_INCORRECT_LENGTH;

    // All groups (FFFFFF) or emissions-related (FFFF33).
    if (request[1] != 0xFF || request[2] != 0xFF ||
        (request[3] != 0xFF && request[3] != 0x33))
        return NRC_REQUEST_OUT_OF_RANGE;

    if (EngineRunning())
        return NRC_CONDITIONS_NOT_CORRECT;

    ClearDiagnosticInformation(1 << ecu);
    Serial.println(F("EVENT DTC CLEARED BY TESTER"));

    response[0] = 0x54;
    length = 1;
    return RESULT_POSITIVE;
}

// =============================================
// DISPATCH
// =============================================

uint8_t UdsHandle(EcuIndex ecu, const uint8_t *request, uint8_t requestLength,
                  uint8_t *response, uint8_t &length)
{
    lastRequestMs[ecu] = millis();

    switch (request[0])
    {
        case 0x10: return SessionControl(ecu, request, requestLength, response, length);
        case 0x3E: return TesterPresent(request, requestLength, response, length);
        case 0x22: return ReadDataByIdentifier(ecu, request, requestLength, response, length);
        case 0x19: return ReadDtcInformation(ecu, request, requestLength, response, length);
        case 0x14: return ClearDtcs(ecu, request, requestLength, response, length);
        default: return NRC_SERVICE_NOT_SUPPORTED;
    }
}

#endif
