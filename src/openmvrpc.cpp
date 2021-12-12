//
// OpenMV RPC (Remote Procedure Call) Library
// Copyright (c) 2020 OpenMV
// Written by Larry Bank & Kwabena W. Agyeman
//

#include "openmvrpc.h"

//
// Communication protocol
//
// All communication starts with a UINT16 'magic' value
// followed by a UINT32 command, UINT32 payload length and a UINT16 CRC
// The back and forth protocol works like this:
// (all integers are in little-endian order)
//
// Send Command:
// Master --> Slave magic HEADER value, cmd, payload len, CRC
// Master <-- Slave magic HEADER ack + CRC (4 bytes total)
// Master --> Slave magic DATA value, n-byte payload, CRC
// Master <-- Slave magic DATA ack + UINT16 CRC (4 bytes total)
//
// Get Result:
// Master --> Slave magic RESULT value, CRC
// Master <-- Slave magic RESULT ack, length, CRC
// Master --> Slave magic DATA value, CRC
// Master <-- Slave magic DATA ack, n-byte payload, CRC
//

#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif

#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif

using namespace openmv;

uint8_t *openmv::__buff = NULL;
size_t openmv::__buff_len = 0;

rpc_callback_entry_t *openmv::__dict = NULL;
size_t openmv::__dict_len = 0;
size_t openmv::__dict_alloced = 0;

static unsigned long unpack_unsigned_long(uint8_t *data)
{
    unsigned long ret;
    memcpy(&ret, data, sizeof(ret));
    return ret;
}

static const uint16_t __crc_16_table[256] PROGMEM =
{
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

uint16_t rpc::__crc_16(uint8_t *data, size_t size)
{
    uint16_t crc = 0xFFFF;

    // for (size_t i = 0; i < size; i++) {
    //    crc ^= data[i] << 8;
    //    for (size_t j = 0; j < 8; j++) crc = (crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0x0000);
    // }

    for (size_t i = 0; i < size; i++) crc = pgm_read_word_near(__crc_16_table + (((crc >> 8) ^ data[i]) & 0xff)) ^ (crc << 8);

    return crc;
}

void rpc::_zero(uint8_t *data, size_t size)
{
    memset(data, 0, size);
}

bool rpc::_same(uint8_t *data, size_t size)
{
    if (!size) return false;

    uint8_t old_data = data[0];

    for (size_t i = 1; i < size; i++) {
        uint8_t new_data = data[i];
        if (new_data != old_data) return false;
        old_data = new_data;
    }

    return true;
}

// djb2 algorithm; see http://www.cse.yorku.ca/~oz/hash.html
uint32_t rpc::_hash(const __FlashStringHelper *name)
{
    PGM_P p = reinterpret_cast<PGM_P>(name); uint32_t h = 5381;

    for (;;) {
        unsigned char c = pgm_read_byte(p++);
        if (!c) break;
        h = ((h << 5) + h) ^ c;
    }

    return h;
}

// djb2 algorithm; see http://www.cse.yorku.ca/~oz/hash.html
uint32_t rpc::_hash(const char *name, size_t length)
{
    uint32_t h = 5381;

    for (size_t i = 0; i < length; i++) {
        char c = *name++;
        if (!c) break;
        h = ((h << 5) + h) ^ c;
    }

    return h;
}

// djb2 algorithm; see http://www.cse.yorku.ca/~oz/hash.html
uint32_t rpc::_hash(const char *name)
{
    uint32_t h = 5381;

    for (;;) {
        char c = *name++;
        if (!c) break;
        h = ((h << 5) + h) ^ c;
    }

    return h;
}

bool rpc::_get_packet(uint16_t magic_value, uint8_t *buff, size_t size, unsigned long timeout)
{
    if (!get_bytes(buff, size, timeout)) return false;
    uint16_t magic = buff[0] | (buff[1] << 8);
    uint16_t crc = buff[size - 2] | (buff[size - 1] << 8);
    return (magic == magic_value) && (crc == __crc_16(buff, size - 2));
}

void rpc::_set_packet(uint8_t *buff, uint16_t magic_value, uint8_t *data, size_t size)
{
    buff[0] = magic_value;
    buff[1] = magic_value >> 8;
    if (((buff + 2) != data) && size) memcpy(buff + 2, data, size);
    uint16_t crc = __crc_16(buff, size + 2);
    buff[size + 2] = crc;
    buff[size + 3] = crc >> 8;
}

bool rpc::stream_reader_setup(uint32_t queue_depth)
{
    uint8_t packet[8];
    _set_packet(packet, 0xEDF6, (uint8_t *) &queue_depth, sizeof(queue_depth));
    if (!_stream_put_bytes(packet, sizeof(packet), 1000)) return false;
    __tx_lfsr = 255;
    return true;
}

bool rpc::stream_reader_loop(rpc_stream_reader_callback_t callback, unsigned long read_timeout)
{
    uint8_t packet[8];
    if (!_stream_get_bytes(packet, sizeof(packet), 1000)) return false;
    uint16_t magic = packet[0] | (packet[1] << 8);
    uint16_t crc = packet[6] | (packet[7] << 8);
    if ((magic != 0x542E) && (crc != __crc_16(packet, sizeof(packet) - 2))) return false;
    uint32_t size = unpack_unsigned_long(packet + 2);
    if (__buff_len < size) return false;
    if (!_stream_get_bytes(__buff, size, read_timeout)) return false;
    if (!callback(__buff, size)) return false;
    if (!_stream_put_bytes(&__tx_lfsr, sizeof(__tx_lfsr), 1000)) return false;
    __tx_lfsr = (__tx_lfsr >> 1) ^ ((__tx_lfsr & 1) ? 0xB8 : 0x00);
    return true;
}

bool rpc::stream_writer_setup()
{
    uint8_t packet[8];
    if (!_stream_get_bytes(packet, sizeof(packet), 1000)) return false;
    uint16_t magic = packet[0] | (packet[1] << 8);
    uint16_t crc = packet[6] | (packet[7] << 8);
    if ((magic != 0xEDF6) && (crc != __crc_16(packet, sizeof(packet) - 2))) return false;
    __queue_depth = max(min(unpack_unsigned_long(packet + 2), _stream_writer_queue_depth_max()), 1);
    __rx_lfsr = 255;
    __credits = __queue_depth;
    return true;
}

bool rpc::stream_writer_loop(rpc_stream_writer_callback_t callback, unsigned long write_timeout)
{
    uint8_t packet[8];

    if (__credits <= (__queue_depth / 2)) {
        if ((!_stream_get_bytes(packet, 1, 1000)) || (packet[0] != __rx_lfsr)) return false;
        __rx_lfsr = (__rx_lfsr >> 1) ^ ((__rx_lfsr & 1) ? 0xB8 : 0x00);
        __credits += 1;
    }

    if (__credits > 0) {
        uint8_t *out_data = NULL;
        uint32_t out_data_len = 0;
        if ((!callback(&out_data, &out_data_len)) || (!out_data) || (!out_data_len)) return false;
        _set_packet(packet, 0x542E, (uint8_t *) &out_data_len, sizeof(out_data_len));
        if (!_stream_put_bytes(packet, sizeof(packet), 1000)) return false;
        if (!_stream_put_bytes(out_data, out_data_len, write_timeout)) return false;
        __credits -= 1;
    }

    return true;
}

bool rpc::_stream_get_bytes(uint8_t *buff, size_t size, unsigned long timeout)
{
    return get_bytes(buff, size, timeout);
}

bool rpc::_stream_put_bytes(uint8_t *data, size_t size, unsigned long timeout)
{
    return put_bytes(data, size, timeout);
}

bool rpc_master::__put_command(uint32_t command, uint8_t *data, size_t size, unsigned long timeout)
{
    const uint32_t header[2] = {command, size};
    uint8_t out_header[12];
    if (__buff_len < (size + 4)) return false;
    _put_short_timeout = _put_short_timeout_reset;
    _get_short_timeout = _get_short_timeout_reset;
    unsigned long start = millis();

    while ((millis() - start) < timeout) {
        _set_packet(out_header, _COMMAND_HEADER_PACKET_MAGIC, (uint8_t *) header, 8); // set repeatedly because SPI.transfer() clears.
        _set_packet(__buff, _COMMAND_DATA_PACKET_MAGIC, data, size); // set repeatedly because SPI.transfer() clears.
        _zero(__in_command_header_buf, sizeof(__in_command_header_buf));
        _zero(__in_command_data_buf, sizeof(__in_command_data_buf));
        _flush();
        put_bytes(out_header, sizeof(out_header), _put_short_timeout);
        if (_get_packet(_COMMAND_HEADER_PACKET_MAGIC, __in_command_header_buf, sizeof(__in_command_header_buf), _get_short_timeout)) {
            put_bytes(__buff, size + 4, _put_long_timeout);
            if (_get_packet(_COMMAND_DATA_PACKET_MAGIC, __in_command_data_buf, sizeof(__in_command_data_buf), _get_short_timeout)) {
                return true;
            }
        }

        // Avoid timeout livelocking.
        _put_short_timeout = min((_put_short_timeout * 6) / 4, timeout);
        _get_short_timeout = min((_get_short_timeout * 6) / 4, timeout);
    }

    return false;
}

bool rpc_master::__get_result(uint8_t **data, size_t *size, unsigned long timeout)
{
    _put_short_timeout = _put_short_timeout_reset;
    _get_short_timeout = _get_short_timeout_reset;
    unsigned long start = millis();

    while ((millis() - start) < timeout) {
        _set_packet(__out_result_header_ack, _RESULT_HEADER_PACKET_MAGIC, NULL, 0); // set repeatedly because SPI.transfer() clears.
        _set_packet(__out_result_data_ack, _RESULT_DATA_PACKET_MAGIC, NULL, 0); // set repeatedly because SPI.transfer() clears.
        _zero(__in_result_header_buf, sizeof(__in_result_header_buf));
        _flush();
        put_bytes(__out_result_header_ack, sizeof(__out_result_header_ack), _put_short_timeout);
        if (_get_packet(_RESULT_HEADER_PACKET_MAGIC, __in_result_header_buf, sizeof(__in_result_header_buf), _get_short_timeout)) {
            uint32_t in_result_data_buf_len = unpack_unsigned_long(__in_result_header_buf + 2) + 4;
            if (__buff_len < in_result_data_buf_len) return false;
            put_bytes(__out_result_data_ack, sizeof(__out_result_data_ack), _put_short_timeout);
            if (_get_packet(_RESULT_DATA_PACKET_MAGIC, __buff, in_result_data_buf_len, _get_long_timeout)) {
                *data = __buff + 2;
                *size = in_result_data_buf_len - 4;
                return true;
            }
        }

        // Avoid timeout livelocking.
        _put_short_timeout = min((_put_short_timeout * 6) / 4, timeout);
        _get_short_timeout = min((_get_short_timeout * 6) / 4, timeout);
    }

    return false;
}

bool rpc_master::call_no_copy_no_args(const __FlashStringHelper *name,
                                      void **result_data, size_t *result_data_len,
                                      unsigned long send_timeout, unsigned long recv_timeout)
{
    return __put_command(_hash(name), NULL, 0, send_timeout)
        ? __get_result((uint8_t **) result_data, result_data_len, recv_timeout) : false;
}

bool rpc_master::call_no_copy_no_args(const String &name,
                                      void **result_data, size_t *result_data_len,
                                      unsigned long send_timeout, unsigned long recv_timeout)
{
    return __put_command(_hash(name.c_str(), name.length()), NULL, 0, send_timeout)
        ? __get_result((uint8_t **) result_data, result_data_len, recv_timeout) : false;
}

bool rpc_master::call_no_copy_no_args(const char *name,
                                      void **result_data, size_t *result_data_len,
                                      unsigned long send_timeout, unsigned long recv_timeout)
{
    return __put_command(_hash(name), NULL, 0, send_timeout)
        ? __get_result((uint8_t **) result_data, result_data_len, recv_timeout) : false;
}

bool rpc_master::call_no_copy(const __FlashStringHelper *name,
                              void *command_data, size_t command_data_len,
                              void **result_data, size_t *result_data_len,
                              unsigned long send_timeout, unsigned long recv_timeout)
{
    return __put_command(_hash(name), (uint8_t *) command_data, command_data_len, send_timeout)
        ? __get_result((uint8_t **) result_data, result_data_len, recv_timeout) : false;
}

bool rpc_master::call_no_copy(const String &name,
                              void *command_data, size_t command_data_len,
                              void **result_data, size_t *result_data_len,
                              unsigned long send_timeout, unsigned long recv_timeout)
{
    return __put_command(_hash(name.c_str(), name.length()), (uint8_t *) command_data, command_data_len, send_timeout)
        ? __get_result((uint8_t **) result_data, result_data_len, recv_timeout) : false;
}

bool rpc_master::call_no_copy(const char *name,
                              void *command_data, size_t command_data_len,
                              void **result_data, size_t *result_data_len,
                              unsigned long send_timeout, unsigned long recv_timeout)
{
    return __put_command(_hash(name), (uint8_t *) command_data, command_data_len, send_timeout)
        ? __get_result((uint8_t **) result_data, result_data_len, recv_timeout) : false;
}

bool rpc_master::call_no_args(const __FlashStringHelper *name,
                              void *result_data, size_t result_data_len, bool return_false_if_received_data_is_zero,
                              unsigned long send_timeout, unsigned long recv_timeout)
{
    void *result_pointer;
    size_t result_size;
    bool result = __put_command(_hash(name), NULL, 0, send_timeout)
        ? __get_result((uint8_t **) &result_pointer, &result_size, recv_timeout) : false;
    if (return_false_if_received_data_is_zero) result = result && result_size;
    if (result) memcpy(result_data, result_pointer, min(result_data_len, result_size));
    else memset(result_data, 0, result_data_len);
    return result;
}

bool rpc_master::call_no_args(const String &name,
                              void *result_data, size_t result_data_len, bool return_false_if_received_data_is_zero,
                              unsigned long send_timeout, unsigned long recv_timeout)
{
    void *result_pointer;
    size_t result_size;
    bool result = __put_command(_hash(name.c_str(), name.length()), NULL, 0, send_timeout)
        ? __get_result((uint8_t **) &result_pointer, &result_size, recv_timeout) : false;
    if (return_false_if_received_data_is_zero) result = result && result_size;
    if (result) memcpy(result_data, result_pointer, min(result_data_len, result_size));
    else memset(result_data, 0, result_data_len);
    return result;
}

bool rpc_master::call_no_args(const char *name,
                              void *result_data, size_t result_data_len, bool return_false_if_received_data_is_zero,
                              unsigned long send_timeout, unsigned long recv_timeout)
{
    void *result_pointer;
    size_t result_size;
    bool result = __put_command(_hash(name), NULL, 0, send_timeout)
        ? __get_result((uint8_t **) &result_pointer, &result_size, recv_timeout) : false;
    if (return_false_if_received_data_is_zero) result = result && result_size;
    if (result) memcpy(result_data, result_pointer, min(result_data_len, result_size));
    else memset(result_data, 0, result_data_len);
    return result;
}

bool rpc_master::call(const __FlashStringHelper *name,
                      void *command_data, size_t command_data_len,
                      void *result_data, size_t result_data_len, bool return_false_if_received_data_is_zero,
                      unsigned long send_timeout, unsigned long recv_timeout)
{
    void *result_pointer;
    size_t result_size;
    bool result = __put_command(_hash(name), (uint8_t *) command_data, command_data_len, send_timeout)
        ? __get_result((uint8_t **) &result_pointer, &result_size, recv_timeout) : false;
    if (return_false_if_received_data_is_zero) result = result && result_size;
    if (result) memcpy(result_data, result_pointer, min(result_data_len, result_size));
    else memset(result_data, 0, result_data_len);
    return result;
}

bool rpc_master::call(const String &name,
                      void *command_data, size_t command_data_len,
                      void *result_data, size_t result_data_len, bool return_false_if_received_data_is_zero,
                      unsigned long send_timeout, unsigned long recv_timeout)
{
    void *result_pointer;
    size_t result_size;
    bool result = __put_command(_hash(name.c_str(), name.length()), (uint8_t *) command_data, command_data_len, send_timeout)
        ? __get_result((uint8_t **) &result_pointer, &result_size, recv_timeout) : false;
    if (return_false_if_received_data_is_zero) result = result && result_size;
    if (result) memcpy(result_data, result_pointer, min(result_data_len, result_size));
    else memset(result_data, 0, result_data_len);
    return result;
}

bool rpc_master::call(const char *name,
                      void *command_data, size_t command_data_len,
                      void *result_data, size_t result_data_len, bool return_false_if_received_data_is_zero,
                      unsigned long send_timeout, unsigned long recv_timeout)
{
    void *result_pointer;
    size_t result_size;
    bool result = __put_command(_hash(name), (uint8_t *) command_data, command_data_len, send_timeout)
        ? __get_result((uint8_t **) &result_pointer, &result_size, recv_timeout) : false;
    if (return_false_if_received_data_is_zero) result = result && result_size;
    if (result) memcpy(result_data, result_pointer, min(result_data_len, result_size));
    else memset(result_data, 0, result_data_len);
    return result;
}

bool rpc_slave::__get_command(uint32_t *command, uint8_t **data, size_t *size, unsigned long timeout)
{
    _set_packet(__out_command_data_ack, _COMMAND_DATA_PACKET_MAGIC, NULL, 0);
    _put_short_timeout = _put_short_timeout_reset;
    _get_short_timeout = _get_short_timeout_reset;
    unsigned long start = millis();

    while ((millis() - start) < timeout) {
        _set_packet(__out_command_header_ack, _COMMAND_HEADER_PACKET_MAGIC, NULL, 0); // set repeatedly because SPI.transfer() clears.
        _zero(__in_command_header_buf, sizeof(__in_command_header_buf));
        _flush();
        if (_get_packet(_COMMAND_HEADER_PACKET_MAGIC, __in_command_header_buf, sizeof(__in_command_header_buf), _get_short_timeout)) {
            uint32_t cmd = unpack_unsigned_long(__in_command_header_buf + 2);
            uint32_t in_command_data_buf_len = unpack_unsigned_long(__in_command_header_buf + 6) + 4;
            if (__buff_len < in_command_data_buf_len) return false;
            put_bytes(__out_command_header_ack, sizeof(__out_command_header_ack), _put_short_timeout);
            if (_get_packet(_COMMAND_DATA_PACKET_MAGIC, __buff, in_command_data_buf_len, _get_long_timeout)) {
               put_bytes(__out_command_data_ack, sizeof(__out_command_data_ack), _put_short_timeout);
               *command = cmd;
               *data = __buff + 2;
               *size = in_command_data_buf_len - 4;
               return true;
            }
        }

        // Avoid timeout livelocking.
        _put_short_timeout = min(_put_short_timeout + 1, timeout);
        _get_short_timeout = min(_get_short_timeout + 1, timeout);
    }

    return false;
}

bool rpc_slave::__put_result(uint8_t *data, size_t size, unsigned long timeout)
{
    const uint32_t header[1] = {size};
    uint8_t out_header[8];
    if (__buff_len < (size + 4)) return false;
    _set_packet(__buff, _RESULT_DATA_PACKET_MAGIC, data, size);
    _put_short_timeout = _put_short_timeout_reset;
    _get_short_timeout = _get_short_timeout_reset;
    unsigned long start = millis();

    while ((millis() - start) < timeout) {
        _set_packet(out_header, _RESULT_HEADER_PACKET_MAGIC, (uint8_t *) header, 4); // set repeatedly because SPI.transfer() clears.
        _zero(__in_response_header_buf, sizeof(__in_response_header_buf));
        _zero(__in_response_data_buf, sizeof(__in_response_data_buf));
        _flush();
        if (_get_packet(_RESULT_HEADER_PACKET_MAGIC, __in_response_header_buf, sizeof(__in_response_header_buf), _get_short_timeout)) {
            put_bytes(out_header, sizeof(out_header), _put_short_timeout);
            if (_get_packet(_RESULT_DATA_PACKET_MAGIC, __in_response_data_buf, sizeof(__in_response_data_buf), _get_short_timeout)) {
                put_bytes(__buff, size + 4, _put_long_timeout);
                return true;
            }
        }

        // Avoid timeout livelocking.
        _put_short_timeout = min(_put_short_timeout + 1, timeout);
        _get_short_timeout = min(_get_short_timeout + 1, timeout);
    }

    return false;
}

bool rpc_slave::__register_callback(uint32_t hash, rpc_callback_type_t type, void *value)
{
    for (size_t i = 0; i < __dict_alloced; i++) {
        if ((__dict[i].object == this) && (__dict[i].key == hash)) {
            __dict[i].type = type;
            __dict[i].value = value;
            return true;
        }
    }

    if (__dict_alloced < __dict_len) {
        __dict[__dict_alloced].object = this;
        __dict[__dict_alloced].key = hash;
        __dict[__dict_alloced].type = type;
        __dict[__dict_alloced].value = value;
        __dict_alloced += 1;
        return true;
    }

    return false;
}

bool rpc_slave::register_callback(const __FlashStringHelper *name, rpc_callback_t callback)
{
    return __register_callback(_hash(name), __CALLBACK, (void *) callback);
}

bool rpc_slave::register_callback(const String &name, rpc_callback_t callback)
{
    return __register_callback(_hash(name.c_str(), name.length()), __CALLBACK, (void *) callback);
}

bool rpc_slave::register_callback(const char *name, rpc_callback_t callback)
{
    return __register_callback(_hash(name), __CALLBACK, (void *) callback);
}

bool rpc_slave::register_callback(const __FlashStringHelper *name, rpc_callback_with_args_t callback)
{
    return __register_callback(_hash(name), __CALLBACK_WITH_ARGS, (void *) callback);
}

bool rpc_slave::register_callback(const String &name, rpc_callback_with_args_t callback)
{
    return __register_callback(_hash(name.c_str(), name.length()), __CALLBACK_WITH_ARGS, (void *) callback);
}

bool rpc_slave::register_callback(const char *name, rpc_callback_with_args_t callback)
{
    return __register_callback(_hash(name), __CALLBACK_WITH_ARGS, (void *) callback);
}

bool rpc_slave::register_callback(const __FlashStringHelper *name, rpc_callback_returns_result_t callback)
{
    return __register_callback(_hash(name), __CALLBACK_RETURNS_RESULT, (void *) callback);
}

bool rpc_slave::register_callback(const String &name, rpc_callback_returns_result_t callback)
{
    return __register_callback(_hash(name.c_str(), name.length()), __CALLBACK_RETURNS_RESULT, (void *) callback);
}

bool rpc_slave::register_callback(const char *name, rpc_callback_returns_result_t callback)
{
    return __register_callback(_hash(name), __CALLBACK_RETURNS_RESULT, (void *) callback);
}

bool rpc_slave::register_callback(const __FlashStringHelper *name, rpc_callback_returns_result_no_copy_t callback)
{
    return __register_callback(_hash(name), __CALLBACK_RETURNS_RESULT_NO_COPY, (void *) callback);
}

bool rpc_slave::register_callback(const String &name, rpc_callback_returns_result_no_copy_t callback)
{
    return __register_callback(_hash(name.c_str(), name.length()), __CALLBACK_RETURNS_RESULT_NO_COPY, (void *) callback);
}

bool rpc_slave::register_callback(const char *name, rpc_callback_returns_result_no_copy_t callback)
{
    return __register_callback(_hash(name), __CALLBACK_RETURNS_RESULT_NO_COPY, (void *) callback);
}

bool rpc_slave::register_callback(const __FlashStringHelper *name, rpc_callback_with_args_returns_result_t callback)
{
    return __register_callback(_hash(name), __CALLBACK_WITH_ARGS_RETURNS_RESULT, (void *) callback);
}

bool rpc_slave::register_callback(const String &name, rpc_callback_with_args_returns_result_t callback)
{
    return __register_callback(_hash(name.c_str(), name.length()), __CALLBACK_WITH_ARGS_RETURNS_RESULT, (void *) callback);
}

bool rpc_slave::register_callback(const char *name, rpc_callback_with_args_returns_result_t callback)
{
    return __register_callback(_hash(name), __CALLBACK_WITH_ARGS_RETURNS_RESULT, (void *) callback);
}

bool rpc_slave::register_callback(const __FlashStringHelper *name, rpc_callback_with_args_returns_result_no_copy_t callback)
{
    return __register_callback(_hash(name), __CALLBACK_WITH_ARGS_RETURNS_RESULT_NO_COPY, (void *) callback);
}

bool rpc_slave::register_callback(const String &name, rpc_callback_with_args_returns_result_no_copy_t callback)
{
    return __register_callback(_hash(name.c_str(), name.length()), __CALLBACK_WITH_ARGS_RETURNS_RESULT_NO_COPY, (void *) callback);
}

bool rpc_slave::register_callback(const char *name, rpc_callback_with_args_returns_result_no_copy_t callback)
{
    return __register_callback(_hash(name), __CALLBACK_WITH_ARGS_RETURNS_RESULT_NO_COPY, (void *) callback);
}

bool rpc_slave::loop(unsigned long recv_timeout, unsigned long send_timeout)
{
    uint32_t command;
    uint8_t *data;
    size_t size;

    if (__get_command(&command, &data, &size, recv_timeout)) {
        uint8_t *out_data = __buff + 2;
        size_t out_data_len = 0;

        for (size_t i = 0; i < __dict_alloced; i++) {
            if ((__dict[i].object == this) && (__dict[i].key == command) && __dict[i].value) {
                switch (__dict[i].type) {
                    case __CALLBACK: {
                        (rpc_callback_t (__dict[i].value))();
                        break;
                    }
                    case __CALLBACK_WITH_ARGS: {
                        (rpc_callback_with_args_t (__dict[i].value))(data, size);
                        break;
                    }
                    case __CALLBACK_RETURNS_RESULT: {
                        (rpc_callback_returns_result_t (__dict[i].value))((void **) &out_data, &out_data_len);
                        break;
                    }
                    case __CALLBACK_RETURNS_RESULT_NO_COPY: {
                        out_data_len = (rpc_callback_returns_result_no_copy_t (__dict[i].value))(__buff + 2);
                        break;
                    }
                    case __CALLBACK_WITH_ARGS_RETURNS_RESULT: {
                        (rpc_callback_with_args_returns_result_t (__dict[i].value))(data, size, (void **) &out_data, &out_data_len);
                        break;
                    }
                    case __CALLBACK_WITH_ARGS_RETURNS_RESULT_NO_COPY: {
                        out_data_len = (rpc_callback_with_args_returns_result_no_copy_t (__dict[i].value))(data, size, __buff + 2);
                        break;
                    }
                }

                break;
            }
        }

        return __put_result(out_data, out_data_len, send_timeout);
    }

    return false;
}

#if !defined(ARDUINO_ARCH_ESP8266) && !defined(ARDUINO_ARCH_NRF52840) && !defined(ARDUINO_ARCH_MBED)
void rpc_can_master::_flush()
{
    for (int i = 0, ii = CAN.parsePacket(); i < ii; i++) CAN.read();
}

bool rpc_can_master::get_bytes(uint8_t *buff, size_t size, unsigned long timeout)
{
    size_t i = 0;
    unsigned long start = millis();

    while (((millis() - start) <= timeout) && (i < size)) {
        for (int j = 0, jj = CAN.parsePacket(); j < jj; j++) {
            buff[i++] = CAN.read();
            if (i >= size) break;
        }
    }

    bool ok = i == size;
    if (!ok) delay(_get_short_timeout);
    return ok;
}

bool rpc_can_master::put_bytes(uint8_t *data, size_t size, unsigned long timeout)
{
    size_t i = 0;
    unsigned long start = millis();

    while (((millis() - start) <= timeout) && (i < size)) {
        if (CAN.beginPacket(__message_id)) {
            size_t sent = CAN.write(data + i, min(size - i, 8));
            if (CAN.endPacket()) i += sent;
        }
    }

    return i == size;
}

void rpc_can_slave::_flush()
{
    for (int i = 0, ii = CAN.parsePacket(); i < ii; i++) CAN.read();
}

bool rpc_can_slave::get_bytes(uint8_t *buff, size_t size, unsigned long timeout)
{
    size_t i = 0;
    unsigned long start = millis();

    while (((millis() - start) <= timeout) && (i < size)) {
        for (int j = 0, jj = CAN.parsePacket(); j < jj; j++) {
            buff[i++] = CAN.read();
            if (i >= size) break;
        }
    }

    return i == size;
}

bool rpc_can_slave::put_bytes(uint8_t *data, size_t size, unsigned long timeout)
{
    size_t i = 0;
    unsigned long start = millis();

    while (((millis() - start) <= timeout) && (i < size)) {
        if (CAN.beginPacket(__message_id)) {
            size_t sent = CAN.write(data + i, min(size - i, 8));
            if (CAN.endPacket()) i += sent;
        }
    }

    return i == size;
}
#endif

#if (!defined(ARDUINO_ARCH_ESP32)) && (!defined(ARDUINO_ARCH_ESP8266))

#define RPC_I2C_MASTER_IMPLEMENTATION(name, port) \
void rpc_i2c##name##_master::_flush() \
{ \
    for (int i = port.available(); i > 0; i--) port.read(); \
} \
\
bool rpc_i2c##name##_master::get_bytes(uint8_t *buff, size_t size, unsigned long timeout) \
{ \
    (void) timeout; \
    bool ok = true; \
    port.begin(); \
    port.setClock(__rate); \
\
    for (size_t i = 0; i < size; i += 32) { \
        size_t size_remaining = size - i; \
        uint8_t request_size = min(size_remaining, 32); \
        uint8_t request_stop = size_remaining <= 32; \
        delayMicroseconds(100); \
        if (port.requestFrom(__slave_addr, request_size, request_stop) != request_size) { ok = false; break; } \
        for (size_t j = 0; j < request_size; j++) buff[i+j] = port.read(); \
    } \
\
    port.end(); \
    if (ok) ok = ok && (!_same(buff, size)); \
    if (!ok) delay(_get_short_timeout); \
    return ok; \
} \
\
bool rpc_i2c##name##_master::put_bytes(uint8_t *data, size_t size, unsigned long timeout) \
{ \
\
    (void) timeout; \
    bool ok = true; \
    port.begin(); \
    port.setClock(__rate); \
\
    for (size_t i = 0; i < size; i += 32) { \
        size_t size_remaining = size - i; \
        uint8_t request_size = min(size_remaining, 32); \
        uint8_t request_stop = size_remaining <= 32; \
        delayMicroseconds(100); \
        port.beginTransmission(__slave_addr); \
        if ((port.write(data + i, request_size) != request_size) || port.endTransmission(request_stop)) { ok = false; break; } \
    } \
\
    port.end(); \
    return ok; \
}

#else

#define RPC_I2C_MASTER_IMPLEMENTATION(name, port) \
void rpc_i2c##name##_master::_flush() \
{ \
    for (int i = port.available(); i > 0; i--) port.read(); \
} \
\
bool rpc_i2c##name##_master::get_bytes(uint8_t *buff, size_t size, unsigned long timeout) \
{ \
    (void) timeout; \
    bool ok = true; \
    port.begin(); \
    port.setClock(__rate); \
\
    for (size_t i = 0; i < size; i += 32) { \
        size_t size_remaining = size - i; \
        uint8_t request_size = min(size_remaining, 32); \
        uint8_t request_stop = size_remaining <= 32; \
        delayMicroseconds(100); \
        if (port.requestFrom(__slave_addr, request_size, request_stop) != request_size) { ok = false; break; } \
        for (size_t j = 0; j < request_size; j++) buff[i+j] = port.read(); \
    } \
\
    if (ok) ok = ok && (!_same(buff, size)); \
    if (!ok) delay(_get_short_timeout); \
    return ok; \
} \
\
bool rpc_i2c##name##_master::put_bytes(uint8_t *data, size_t size, unsigned long timeout) \
{ \
\
    (void) timeout; \
    bool ok = true; \
    port.begin(); \
    port.setClock(__rate); \
\
    for (size_t i = 0; i < size; i += 32) { \
        size_t size_remaining = size - i; \
        uint8_t request_size = min(size_remaining, 32); \
        uint8_t request_stop = size_remaining <= 32; \
        delayMicroseconds(100); \
        port.beginTransmission(__slave_addr); \
        if ((port.write(data + i, request_size) != request_size) || port.endTransmission(request_stop)) { ok = false; break; } \
    } \
\
    return ok; \
}

#endif

RPC_I2C_MASTER_IMPLEMENTATION(,Wire)

#if WIRE_HOWMANY > 1
RPC_I2C_MASTER_IMPLEMENTATION(1,Wire1)
#endif

#undef RPC_I2C_MASTER_IMPLEMENTATION

#define RPC_I2C_SLAVE_IMPLEMENTATION(name, port) \
volatile uint8_t *rpc_i2c##name##_slave::__bytes_buff = NULL; \
volatile int rpc_i2c##name##_slave::__bytes_size = 0; \
\
void rpc_i2c##name##_slave::onReceiveHandler(int numBytes) \
{ \
    if (!__bytes_size) return; \
    for (int i = 0, j = min(__bytes_size, numBytes); i < j; i++) __bytes_buff[i] = port.read(); \
    __bytes_buff += numBytes; \
    __bytes_size -= numBytes; \
} \
\
void rpc_i2c##name##_slave::onRequestHandler() \
{ \
    if (!__bytes_size) return; \
    size_t written = port.write((uint8_t *) __bytes_buff, min(__bytes_size, 32)); \
    __bytes_buff += written; \
    __bytes_size -= written; \
} \
\
void rpc_i2c##name##_slave::_flush() \
{ \
    for (int i = port.available(); i > 0; i--) port.read(); \
} \
\
bool rpc_i2c##name##_slave::get_bytes(uint8_t *buff, size_t size, unsigned long timeout) \
{ \
    __bytes_buff = buff; \
    __bytes_size = size; \
    unsigned long start = millis(); \
    while (((millis() - start) <= timeout) && __bytes_size); \
    bool ok = !__bytes_size; \
    __bytes_size = 0; \
    return ok; \
} \
\
bool rpc_i2c##name##_slave::put_bytes(uint8_t *data, size_t size, unsigned long timeout) \
{ \
    __bytes_buff = data; \
    __bytes_size = size; \
    unsigned long start = millis(); \
    while (((millis() - start) <= timeout) && __bytes_size); \
    bool ok = !__bytes_size; \
    __bytes_size = 0; \
    return ok; \
}

RPC_I2C_SLAVE_IMPLEMENTATION(,Wire)

#if WIRE_HOWMANY > 1
RPC_I2C_SLAVE_IMPLEMENTATION(1,Wire1)
#endif

#if WIRE_HOWMANY > 2
RPC_I2C_SLAVE_IMPLEMENTATION(2,Wire2)
#endif

#if WIRE_HOWMANY > 3
RPC_I2C_SLAVE_IMPLEMENTATION(3,Wire3)
#endif

#undef RPC_I2C_SLAVE_IMPLEMENTATION

#define RPC_SPI_MASTER_IMPLEMENTATION(name, port) \
bool rpc_spi##name##_master::get_bytes(uint8_t *buff, size_t size, unsigned long timeout) \
{ \
    put_bytes(buff, size, timeout); \
    bool ok = !_same(buff, size); \
    if (!ok) delay(_get_short_timeout); \
    return ok; \
} \
\
bool rpc_spi##name##_master::put_bytes(uint8_t *data, size_t size, unsigned long timeout) \
{ \
    (void) timeout; \
\
    digitalWrite(__cs_pin, LOW); \
    delayMicroseconds(100); /* Give slave time to get ready */ \
    port.beginTransaction(__settings); \
    port.transfer(data, size); \
    port.endTransaction(); \
    digitalWrite(__cs_pin, HIGH); \
    return true; \
}

RPC_SPI_MASTER_IMPLEMENTATION(,SPI)

#if SPI_HOWMANY > 1
RPC_SPI_MASTER_IMPLEMENTATION(1,SPI1)
#endif

#if SPI_HOWMANY > 2
RPC_SPI_MASTER_IMPLEMENTATION(2,SPI2)
#endif

#if SPI_HOWMANY > 3
RPC_SPI_MASTER_IMPLEMENTATION(3,SPI3)
#endif

#undef RPC_SPI_MASTER_IMPLEMENTATION

#define RPC_HARDWARE_SERIAL_UART_MASTER_IMPLEMENTATION(name, port) \
void rpc_hardware_serial##name##_uart_master::_flush() \
{ \
    for (int i = port.available(); i > 0; i--) port.read(); \
} \
\
bool rpc_hardware_serial##name##_uart_master::get_bytes(uint8_t *buff, size_t size, unsigned long timeout) \
{ \
    port.setTimeout(timeout + 1); \
    bool ok = (port.readBytes((char *) buff, size) == size) && (!_same(buff, size)); \
    if (!ok) delay(_get_short_timeout); \
    return ok; \
} \
\
bool rpc_hardware_serial##name##_uart_master::put_bytes(uint8_t *buff, size_t size, unsigned long timeout) \
{ \
    (void) timeout; \
    return port.write((char *) buff, size) == size; \
}

#ifdef SERIAL_HOWMANY

#if SERIAL_HOWMANY > 0
RPC_HARDWARE_SERIAL_UART_MASTER_IMPLEMENTATION(1,Serial1)
#endif

#if SERIAL_HOWMANY > 1
RPC_HARDWARE_SERIAL_UART_MASTER_IMPLEMENTATION(2,Serial2)
#endif

#if SERIAL_HOWMANY > 2
RPC_HARDWARE_SERIAL_UART_MASTER_IMPLEMENTATION(3,Serial3)
#endif

#if SERIAL_HOWMANY > 3
RPC_HARDWARE_SERIAL_UART_MASTER_IMPLEMENTATION(4,Serial4)
#endif

#else

#ifdef SERIAL_PORT_HARDWARE
RPC_HARDWARE_SERIAL_UART_MASTER_IMPLEMENTATION(,SERIAL_PORT_HARDWARE)
#endif

#ifdef SERIAL_PORT_HARDWARE1
RPC_HARDWARE_SERIAL_UART_MASTER_IMPLEMENTATION(1,SERIAL_PORT_HARDWARE1)
#endif

#ifdef SERIAL_PORT_HARDWARE2
RPC_HARDWARE_SERIAL_UART_MASTER_IMPLEMENTATION(2,SERIAL_PORT_HARDWARE2)
#endif

#ifdef SERIAL_PORT_HARDWARE3
RPC_HARDWARE_SERIAL_UART_MASTER_IMPLEMENTATION(3,SERIAL_PORT_HARDWARE3)
#endif

#ifdef SERIAL_PORT_HARDWARE4
RPC_HARDWARE_SERIAL_UART_MASTER_IMPLEMENTATION(4,SERIAL_PORT_HARDWARE4)
#endif

#ifdef SERIAL_PORT_HARDWARE5
RPC_HARDWARE_SERIAL_UART_MASTER_IMPLEMENTATION(5,SERIAL_PORT_HARDWARE5)
#endif

#ifdef SERIAL_PORT_HARDWARE6
RPC_HARDWARE_SERIAL_UART_MASTER_IMPLEMENTATION(6,SERIAL_PORT_HARDWARE6)
#endif

#ifdef SERIAL_PORT_HARDWARE7
RPC_HARDWARE_SERIAL_UART_MASTER_IMPLEMENTATION(7,SERIAL_PORT_HARDWARE7)
#endif

#endif

#ifdef SERIAL_PORT_USBVIRTUAL
RPC_HARDWARE_SERIAL_UART_MASTER_IMPLEMENTATION(USB,SERIAL_PORT_USBVIRTUAL)
#endif

#undef RPC_HARDWARE_SERIAL_UART_MASTER_IMPLEMENTATION

#define RPC_HARDWARE_SERIAL_UART_SLAVE_IMPLEMENTATION(name, port) \
void rpc_hardware_serial##name##_uart_slave::_flush() \
{ \
    for (int i = port.available(); i > 0; i--) port.read(); \
} \
\
bool rpc_hardware_serial##name##_uart_slave::get_bytes(uint8_t *buff, size_t size, unsigned long timeout) \
{ \
    port.setTimeout(timeout + 1); \
    return port.readBytes((char *) buff, size) == size; \
} \
\
bool rpc_hardware_serial##name##_uart_slave::put_bytes(uint8_t *buff, size_t size, unsigned long timeout) \
{ \
    (void) timeout; \
    return port.write((char *) buff, size) == size; \
}

#ifdef SERIAL_HOWMANY

#if SERIAL_HOWMANY > 0
RPC_HARDWARE_SERIAL_UART_SLAVE_IMPLEMENTATION(1,Serial1)
#endif

#if SERIAL_HOWMANY > 1
RPC_HARDWARE_SERIAL_UART_SLAVE_IMPLEMENTATION(2,Serial2)
#endif

#if SERIAL_HOWMANY > 2
RPC_HARDWARE_SERIAL_UART_SLAVE_IMPLEMENTATION(3,Serial3)
#endif

#if SERIAL_HOWMANY > 3
RPC_HARDWARE_SERIAL_UART_SLAVE_IMPLEMENTATION(4,Serial4)
#endif

#else

#ifdef SERIAL_PORT_HARDWARE
RPC_HARDWARE_SERIAL_UART_SLAVE_IMPLEMENTATION(,SERIAL_PORT_HARDWARE)
#endif

#ifdef SERIAL_PORT_HARDWARE1
RPC_HARDWARE_SERIAL_UART_SLAVE_IMPLEMENTATION(1,SERIAL_PORT_HARDWARE1)
#endif

#ifdef SERIAL_PORT_HARDWARE2
RPC_HARDWARE_SERIAL_UART_SLAVE_IMPLEMENTATION(2,SERIAL_PORT_HARDWARE2)
#endif

#ifdef SERIAL_PORT_HARDWARE3
RPC_HARDWARE_SERIAL_UART_SLAVE_IMPLEMENTATION(3,SERIAL_PORT_HARDWARE3)
#endif

#ifdef SERIAL_PORT_HARDWARE4
RPC_HARDWARE_SERIAL_UART_SLAVE_IMPLEMENTATION(4,SERIAL_PORT_HARDWARE4)
#endif

#ifdef SERIAL_PORT_HARDWARE5
RPC_HARDWARE_SERIAL_UART_SLAVE_IMPLEMENTATION(5,SERIAL_PORT_HARDWARE5)
#endif

#ifdef SERIAL_PORT_HARDWARE6
RPC_HARDWARE_SERIAL_UART_SLAVE_IMPLEMENTATION(6,SERIAL_PORT_HARDWARE6)
#endif

#ifdef SERIAL_PORT_HARDWARE7
RPC_HARDWARE_SERIAL_UART_SLAVE_IMPLEMENTATION(7,SERIAL_PORT_HARDWARE7)
#endif

#endif

#ifdef SERIAL_PORT_USBVIRTUAL
RPC_HARDWARE_SERIAL_UART_SLAVE_IMPLEMENTATION(USB,SERIAL_PORT_USBVIRTUAL)
#endif

#undef RPC_HARDWARE_SERIAL_UART_SLAVE_IMPLEMENTATION

#ifdef ARDUINO_ARCH_AVR
void rpc_software_serial_uart_master::_flush()
{
    __serial.listen();
    for (int i = __serial.available(); i > 0; i--) __serial.read();
}

bool rpc_software_serial_uart_master::get_bytes(uint8_t *buff, size_t size, unsigned long timeout)
{
    (void) timeout;
    __serial.listen();

    size_t i = 0;
    unsigned long start = millis();

    while ((i < size) && ((millis() - start) <= 2)) {
        if (__serial.available()) {
            buff[i++] = __serial.read();
            start = millis(); // new 2ms timeout per character
        }
    }

    bool ok = i == size;
    if (!ok) delay(_get_short_timeout);
    return ok;
}

bool rpc_software_serial_uart_master::put_bytes(uint8_t *buff, size_t size, unsigned long timeout)
{
    (void) timeout;
    return __serial.write(buff, size) == size;
}

void rpc_software_serial_uart_slave::_flush()
{
    __serial.listen();
    for (int i = __serial.available(); i > 0; i--) __serial.read();
}

bool rpc_software_serial_uart_slave::get_bytes(uint8_t *buff, size_t size, unsigned long timeout)
{
    (void) timeout;
    __serial.listen();

    size_t i = 0;
    unsigned long start = millis();

    while ((i < size) && ((millis() - start) <= 2)) {
        if (__serial.available()) {
            buff[i++] = __serial.read();
            start = millis(); // new 2ms timeout per character
        }
    }

    return i == size;
}

bool rpc_software_serial_uart_slave::put_bytes(uint8_t *buff, size_t size, unsigned long timeout)
{
    (void) timeout;
    return __serial.write(buff, size) == size;
}
#endif // ARDUINO_ARCH_AVR
