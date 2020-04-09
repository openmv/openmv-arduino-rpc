/* Arduino RPC (Remote Procedure Call) library
   Copyright (c) 2020 OpenMV
   written by Kwabena Agyeman and Larry Bank
*/

#include <ArduinoRPC.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>

//
// Public methods
//

//
// Initialize the RPC class
// returns false if passed invalid parameters
// true if initialized correctly
//
bool RPC::begin(int comm_type, unsigned long speed)
{
bool rc = false;

    _comm_type = comm_type;
    _speed = speed;
    switch (comm_type) {
        case RPC_I2C:
           Wire.begin();
           Wire.setClock(speed);
           rc = true;
           break;
        case RPC_UART:
           Serial.begin(speed);
           rc = true;
           break;
    }
    return rc;
} /* begin() */

bool RPC::begin(int comm_type, unsigned long speed, int pin1=-1, int pin2=-1)
{
bool rc = false;

    _comm_type = comm_type;
    _speed = speed;
    _pin1 = pin1;
    _pin2 = pin2;

    switch (comm_type) {
#if !defined( __AVR__ ) && !defined( NRF52 )
        case RPC_I2C:
           Wire.begin(pin1, pin2);
           Wire.setClock(speed);
           rc = true;
           break;
#endif
        case RPC_UART: // if pins are specified, use the SoftwareSerial library instead of the hardware
           comm_type = RPC_SOFTUART;
           _sserial = new SoftwareSerial(pin1, pin2);
           _sserial->begin(speed);
           rc = true;
           break;
    }
    return rc;

} /* begin() */
//
// Register the callback function for the specific remote procedure
// returns false if memory has been exhausted
// or true for success
//
bool RPC::register_callback(int rpc_id, RPC_CALLBACK *pfnCB)
{
    if (_rpc_count >= MAX_CALLBACKS) // out of space to save this
        return false;
    _rpcList[_rpc_count].id = rpc_id;
    _rpcList[_rpc_count].pfnCallback = pfnCB:
    _rpc_count++; 
    return true;
} /* register_callback() */
//
// Make a remote function call
//
bool RPC::call(int rpc_id, uint8_t *out_data, int out_data_len, uint8_t *in_data, int *in_data_len, int send_timeout, int recv_timeout);
{
bool rc = false;

    if (_put_command(rpc_id, out_data, out_data_len, send_timeout)) {
       rc = _get_result(in_data, in_data_len, recv_timeout);
    }
    return rc;
} /* call() */

//
// Protected methods
//

//
// Calculate a 16-bit CRC value for a set of data bytes
//
uint16_t  RPC::_crc16(uint8_t *data, int len)
{
uint8_t * d = data;
uint16_t crc = 0xFFFF;

    for(int i=0; i<len; i++) {
        crc ^= (d[i] << 8);
        for(int j=0; j<8; j++) {
            crc = (crc << 1);
            if (crc & 0x8000)
                crc ^= 0x1021;
        } // for j
    } // for i
    return crc;
} /* _crc16() */

bool RPC::_put_command(int cmd, uint8_t *data, int data_len, int timeout)
{
unsigned long start, end;
bool rc = false;

    start = millis();
    end = start + timeout;
    while (millis() < end) {
        _put_packet(__COMMAND_HEADER_PACKET_MAGIC, struct.pack("<II", command, len(data)))
        if (_get_packet(__COMMAND_HEADER_PACKET_MAGIC, 20)) { 
            _put_packet(__COMMAND_DATA_PACKET_MAGIC, data, 5000);
            if (_get_packet(__COMMAND_DATA_PACKET_MAGIC))
                rc = true;
        }
    } // while waiting for main timeout
    return rc;
} /* _put_command() */

bool RPC::_get_result(uint8_t *data, int *data_len, int timeout)
{
unsigned long start, end;
bool rc = false;

    start = millis();
    end = start + timeout;
    while (millis() < end) {
        _put_packet(__RESULT_HEADER_PACKET_MAGIC);
        if (_get_packet(__RESULT_HEADER_PACKET_MAGIC, data, 4, 20)) {
            _put_packet(__RESULT_DATA_PACKET_MAGIC);
            if (_get_packet(__RESULT_DATA_PACKET_MAGIC, struct.unpack("<I", packet)[0], 5000))
               rc = true;
        }
    } // while not timeout
    return rc;
} /* _get_result() */

//
// Receive a packet from a RPC device
// returns the number of bytes received
//
bool RPC::_get_packet(uint16_t magic_value, uint8_t *payload, int *payload_len, int timeout)
{
unsigned long start, end;
bool rc = false;

    start = millis();
    end = start + timeout;
    while (millis() < end) {
        switch (_comm_type) {
        }
    } // while not timeout
return rc;
} /* _get_packet() */
//
// Send a package to a RPC device
//
bool RPC::_put_packet(uint16_t magic_value, uint8_t *payload, int timeout)
{
unsigned long start, end;
bool rc = false;

    start = millis();
    end = start + timeout;
    while (millis() < end) {
        switch (_comm_type) {
        }
    } // while not timeout
return rc;
} /* _put_packet() */

