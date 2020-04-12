//
// Arduino RPC (Remote Procedure Call) library
// Copyright (c) 2020 OpenMV
// written by Kwabena Agyeman and Larry Bank
//
// project started April, 2020
//

#include <ArduinoRPC.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>

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

//
// Public methods
//

//
// Initialize the RPC class (default bus/port)
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

        case RPC_SPI:
           SPI.begin();
           rc = true;
           break;

        case RPC_UART:
           Serial.begin(speed);
           rc = true;
           break;
    }
    return rc;
} /* begin() */
//
// Initialize the RPC Class (specific port/pins)
// For I2C, pin1 = SDA, pin2 = SCL
// for UART, pin1 = TX, pin2 = RX
// 
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
        case RPC_UART: // Since pins are specified, use the SoftwareSerial library instead of the hardware
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
// This is used on the Slave side to manage execution of incoming commands
//
bool RPC::register_callback(int rpc_id, RPC_CALLBACK *pfnCB)
{
    if (_rpc_count >= MAX_CALLBACKS) // out of space to save this
        return false;
    _rpcList[_rpc_count].id = rpc_id;
    _rpcList[_rpc_count].pfnCallback = pfnCB;
    _rpc_count++; 
    return true;
} /* register_callback() */
//
// Match the command id to the registered callback function
// returns NULL if none found
//
RPC_CALLBACK *find_callback(uint32_t rpc_id)
{
int i;
  for (i=0; i<_rpc_count; i++) {
    if (rpc_id == _rpcList[i].id)
      return _rpcList[i].pfnCallback;
  }
  return NULL;
} /* find_callback() */

//
// Make a remote function call
// 
// Used by the Master side to direct the Slave to execute a command
//
bool RPC::call(int rpc_id, uint8_t *out_data, uint32_t out_data_len, uint8_t *in_data, uint32_t *in_data_len, int send_timeout, int recv_timeout)
{
bool rc = false;

    if (_put_command(rpc_id, out_data, out_data_len, send_timeout)) {
       rc = _get_result(in_data, in_data_len, recv_timeout);
    }
    return rc;
} /* call() */

//
// Private methods
//

//
// Calculate a 16-bit CRC value for a stream of data bytes
//
uint16_t  RPC::_crc16(uint8_t *data, uint32_t len)
{
uint8_t * d = data;
uint16_t crc = 0xFFFF;

    for(uint32_t i=0; i<len; i++) {
        crc ^= (d[i] << 8);
        for(int j=0; j<8; j++) {
            crc = (crc << 1);
            if (crc & 0x8000)
                crc ^= 0x1021;
        } // for j
    } // for i
    return crc;
} /* _crc16() */
//
// Construct and send a command packet from Master to Slave
//
bool RPC::_put_command(int cmd, uint8_t *data, uint32_t data_len, int timeout)
{
unsigned long start, end;
bool rc = false;
uint8_t ucTemp[8];
uint32_t *uiTemp = (uint32_t *)ucTemp;

    start = millis();
    end = start + timeout;
    while (millis() < end) {
        uiTemp[0] = cmd; uiTemp[1] = data_len; 
        _put_packet(__COMMAND_HEADER_PACKET_MAGIC, ucTemp, 8, 10);
        if (_get_packet(__COMMAND_HEADER_PACKET_MAGIC, ucTemp, 0, 20)) { 
            _put_packet(__COMMAND_DATA_PACKET_MAGIC, data, data_len, 5000);
            if (_get_packet(__COMMAND_DATA_PACKET_MAGIC, ucTemp, 0, 20))
                rc = true;
        }
    } // while waiting for main timeout
    return rc;
} /* _put_command() */
//
// Read the resulting data from a command (Master calls to get from Slave)
//
bool RPC::_get_result(uint8_t *data, uint32_t *data_len, int timeout)
{
unsigned long start, end;
bool rc = false;
uint32_t len;

    start = millis();
    end = start + timeout;
    while (millis() < end) {
        _put_packet(__RESULT_HEADER_PACKET_MAGIC, data, 0, 10);
        if (_get_packet(__RESULT_HEADER_PACKET_MAGIC, data, 4, 20)) {
            len = *(uint32_t *)data; 
            _put_packet(__RESULT_DATA_PACKET_MAGIC, data, 0, 10);
            if (_get_packet(__RESULT_DATA_PACKET_MAGIC, data, len, 5000)) {
                rc = true;
                *data_len = len;
            }
        }
    } // while not timeout
    return rc;
} /* _get_result() */

//
// Receive a packet from a RPC device
// Confirms matching magic value and calculated CRC
//
// returns true if the data was received + the magic value and crc match
//
bool RPC::_get_packet(uint16_t magic_value, uint8_t *payload, uint32_t payload_len, int timeout)
{
bool rc = false;
unsigned long end;
uint32_t i, len = payload_len + 4;
uint16_t crc, in_magic, in_crc;

    end = millis() + timeout;
    while (millis() < end && !rc) {
        // Confirm the packet has the right length, magic number and CRC
        if (_get_bytes(payload, len, timeout)) { // got the length requested
            in_magic = payload[0] | (payload[1] << 8);
            crc = _crc16(payload, len-2);
            in_crc = payload[len-2] | (payload[len-1] << 8);
            if (in_magic == magic_value && crc == in_crc)
                rc = true;
            else
                break;
        } else break;
    } // while not timeout
    if (rc) { // success, remove the magic value from the packet
        for (i=0; i<len-2; i++) {
            payload[i] = payload[i+2];
        }
    }
return rc;
} /* _get_packet() */
//
// Receive bytes (either master or slave)
//
// Lowest level data reception - reads raw bytes from the chosen channel
..
bool RPC::_get_bytes(uint8_t *data, uint32_t len, int timeout)
{
unsigned long end;
bool rc = false;
uint32_t i = 0;

    end = millis() + timeout;

    switch (_comm_type) {
        case RPC_I2C:
            Wire.requestFrom(I2C_ADDR, len);
            while (millis() < end && i < len && Wire.available()) {
                data[i++] = Wire.read();
            }
            break;
        case RPC_UART:
            while (millis() < end && i < len) {
                data[i++] = Serial.read();
            }
            break;
        case RPC_SOFTUART:
            while (millis() < end && i < len) {
                data[i++] = _sserial->read();
            }
            break;
        case RPC_SPI:
            SPI.beginTransaction(SPISettings(_speed, MSBFIRST, SPI_MODE0));
            while (millis() < end && i < len) {
                data[i++] = SPI.transfer(0);
            }
            SPI.endTransaction();
            break;
    } // switch on comm type
    rc = (i == len);
    return rc;
} /* _get_bytes() */
//
// Send bytes (either master or slave)
//
// Lowest level data transmission - sends bytes to the chosen channel
//
bool RPC::_put_bytes(uint8_t *data, uint32_t data_len, int timeout)
{
unsigned long end;
bool rc = false;

    end = millis() + timeout;
    while (!rc && millis() < end) { 
        switch (_comm_type) {
            case RPC_I2C:
               Wire.beginTransmission(I2C_ADDR);
               Wire.write(data, data_len);
               rc = !Wire.endTransmission();
               break;
            case RPC_UART:
               Serial.write(data, data_len);
               rc = true;
               break;
            case RPC_SOFTUART:
               _sserial->write(data, data_len);
               rc = true;
               break;
            case RPC_SPI:
               SPI.beginTransaction(SPISettings(_speed, MSBFIRST, SPI_MODE0));
               SPI.transfer(data, data_len);
               SPI.endTransaction();
               rc = true;
               break;
        } // switch
    } // while waiting for timeout

    return rc;
} /* _put_bytes() */

//
// Send a packet to a RPC device
//
// Generates a crc value for the given data, repacks it into a homogenous
// buffer and then transmits it in a single transaction
//
// returns true if the transmission succeeds
//
bool RPC::_put_packet(uint16_t magic_value, uint8_t *data, uint32_t data_len, int timeout)
{
bool rc = false;
uint16_t crc;
uint32_t len = 2;
uint8_t ucTemp[MAX_LOCAL_BUFFER]; // this limits the amount of data we can send


// It's best to combine all of the bytes we're going to transmit into a single
// buffer so that they can be sent in a single comm transaction

    *(uint16_t *)ucTemp = magic_value; // start with 2 bytes of magic value
    memcpy(&ucTemp[len], data, data_len);
    len += data_len;
    crc = _crc16(ucTemp, len);
    ucTemp[len++] = (uint8_t)crc;
    ucTemp[len++] = (uint8_t)(crc >> 8);
    rc = _put_bytes(ucTemp, len, timeout);

return rc;
} /* _put_packet() */

// For RPC Slave
//
// Wait to receive a command from the master
//
uint32_t RPC::get_command(uint8_t *data, uint32_t *data_len)
{
unsigned long end;
uint32_t len = 0, cmd = 0;
uint8_t ucTemp[MIN_PACKET_SIZE];

    end = millis() + 100;
    while (millis() < end && cmd != 0) {
        if (_get_packet(__COMMAND_HEADER_PACKET_MAGIC, data, 8, 10)) {
            cmd = *(uint32_t *)&data[0];
            len = *(uint32_t *)&data[4];
            _put_packet(__COMMAND_HEADER_PACKET_MAGIC, data, 0, 10); // send ack
            if (_get_packet(__COMMAND_DATA_PACKET_MAGIC, data, len, 5000)) {
                _put_packet(__COMMAND_DATA_PACKET_MAGIC, ucTemp, 0, 10); // send ack
            } else { // something went wrong, nullify it
                cmd = len = 0;
            }
        }
    } // while waiting for incoming data
    *data_len = len;
    return cmd;
} /* get_command() */

//
// Send the master the result of the last command
//
void RPC::put_result(uint8_t *data, uint32_t data_len)
{
bool done = false;
unsigned long end;
uint8_t ucTemp[MIN_PACKET_SIZE];

    end = millis() + 100;
    while (millis() < end && !done) {
        if (_get_packet(__RESULT_HEADER_PACKET_MAGIC, ucTemp, 0, 10)) {
            *(uint32_t *)ucTemp = data_len;
            // send length first as ack
            _put_packet(__RESULT_HEADER_PACKET_MAGIC, ucTemp, 4, 10);
            if (_get_packet(__RESULT_DATA_PACKET_MAGIC, ucTemp, 0, 10)) {
                _put_packet(__RESULT_DATA_PACKET_MAGIC, data, data_len, 5000);
                done = true;
            }
        }
    } // while not finished or not timed out
} /* put_result() */
