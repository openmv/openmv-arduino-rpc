//
// Arduino RPC (Remote Procedure Call) library
// Copyright (c) 2020 OpenMV
// written by Larry Bank
//
// project started April, 2020
//

#include <ArduinoRPC.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>

// Need to declare storage for these static class variables
// which are part of the rpc_i2c_slave class
uint32_t rpc_i2c_slave::_receive_len;
uint32_t rpc_i2c_slave::_response_len;
uint8_t rpc_i2c_slave::_buf[MAX_LOCAL_BUFFER];

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

void rpc_slave::loop(void)
{
uint32_t len, new_len, cmd;
uint8_t ucTemp[MAX_LOCAL_BUFFER];
RPC_CALLBACK pfnCB;

    while (1) {
      cmd = get_command(ucTemp, &len);
      if (cmd) {
        pfnCB = find_callback(cmd);
        if (pfnCB) { // we registered a callback for this command
          new_len = (*pfnCB)(cmd, ucTemp, len);
          put_result(ucTemp, new_len);
        } else { // no callback to respond
          put_result(ucTemp, 0); // DEBUG - need a NACK
        }
      }
    }
} /* rpc_slave::loop() */

rpc_i2c_master::rpc_i2c_master(int iAddr, unsigned long speed)
{
    _iAddr = iAddr;
    Wire.begin();
    Wire.setClock(speed);

} /* rpc_i2c::rpc_i2c_master() */

rpc_i2c_slave::rpc_i2c_slave(int iAddr, unsigned long speed)
{
    Wire.begin(iAddr); // register as slave at address 'iAddr'
    Wire.setClock(speed);
    Wire.onReceive(receive_event); // register callback event
    Wire.onRequest(request_event);
    _receive_len = _response_len = 0;
} /* rpc_i2c::rpc_i2c_master() */

void rpc_i2c_slave::receive_event(int len)
{
    while(0 < Wire.available() && _receive_len < (unsigned)len && _receive_len < MAX_LOCAL_BUFFER)
    {
      _buf[_receive_len++] = Wire.read(); // gather incoming data
    }
} /* rpc_i2c_slave::receive_event() */

void rpc_i2c_slave::request_event(void)
{
    if (_response_len) {
        Wire.write(_buf, _response_len);
        _response_len = 0; // indicate we transsmitted the buffer to the master
    }
} /* rpc_i2c_slave::request_event() */

bool rpc_i2c_slave::get_bytes(uint8_t *data, uint32_t len, int timeout)
{
unsigned long end = millis() + timeout;
    
    // wait for data to arrive
    while (millis() < end && _receive_len < len) {
        delay(1); // allow CPU to do something else
    }
    if (_receive_len >= len) { // we fulfilled the request
        memcpy(data, _buf, len);
        memcpy(_buf, &_buf[len], _receive_len - len); // move unused data down
        _receive_len -= len;
        return true;
    }
    return false;
} /* rpc_i2c_slave::get_bytes() */

bool rpc_i2c_slave::put_bytes(uint8_t *data, uint32_t data_len, int timeout)
{
    unsigned long end = millis() + timeout;
    // Prepare the outbound data for the master
    // The master must request it, we can't push it
    // so place it in our buffer, ready to go
    memcpy(_buf, data, data_len);
    _response_len = data_len;
    while (millis() < end && _response_len != 0) {
        delay(1); // wait for master to read the data it asked for
    }
    return (_response_len == 0); // indicates data was read by master

} /* rpc_i2c_master::put_bytes() */

bool rpc_i2c_master::get_bytes(uint8_t *data, uint32_t len, int timeout)
{
unsigned long end = millis() + timeout;
uint32_t i = 0;
    
    Wire.requestFrom(_iAddr, len);
    while (millis() < end && i < len && Wire.available()) {
        data[i++] = Wire.read();
    }
    
    return (i == len);
} /* rpc_i2c_master::get_bytes() */

bool rpc_i2c_master::put_bytes(uint8_t *data, uint32_t data_len, int timeout)
{
    (void)timeout;
    Wire.beginTransmission(_iAddr);
    Wire.write(data, data_len);
    return !Wire.endTransmission();

} /* rpc_i2c_master::put_bytes() */

rpc_spi_master::rpc_spi_master(unsigned long speed)
{
    _speed = speed;
    SPI.begin();

} /* rpc_spi_master::rpc_spi_master() */

bool rpc_spi_master::get_bytes(uint8_t *data, uint32_t len, int timeout)
{
uint32_t i = 0;
unsigned long end = millis() + timeout;
    
    SPI.beginTransaction(SPISettings(_speed, MSBFIRST, SPI_MODE0));
    while (millis() < end && i < len) {
        data[i++] = SPI.transfer(0);
    }
    SPI.endTransaction();
    return (i == len);
} /* rpc_spi_master::get_bytes() */

bool rpc_spi_master::put_bytes(uint8_t *data, uint32_t data_len, int timeout)
{
    (void)timeout;
    SPI.beginTransaction(SPISettings(_speed, MSBFIRST, SPI_MODE0));
    SPI.transfer(data, data_len);
    SPI.endTransaction();
    return false;
} /* rpc_spi_master::put_bytes() */

rpc_uart_master::rpc_uart_master(unsigned long speed)
{
    Serial.begin(speed);
} /* rpc_uart_master::rpc_uart_master() */

bool rpc_uart_master::get_bytes(uint8_t *data, uint32_t len, int timeout)
{
uint32_t i = 0;
unsigned long end = millis() + timeout;
    
    while (millis() < end && i < len) {
        data[i++] = Serial.read();
    }
    return (i == len);
} /* rpc_uart_master::get_bytes() */

bool rpc_uart_master::put_bytes(uint8_t *data, uint32_t data_len, int timeout)
{
    (void)timeout;
    Serial.write(data, data_len);
    return true;
} /* rpc_uart_master::put_bytes() */

rpc_softuart_master::rpc_softuart_master(int pin1, int pin2, unsigned long speed)
{
    _sserial = new SoftwareSerial(pin1, pin2);
    _sserial->begin(speed);
} /* rpc_softuart_master::rpc_softuart_master() */

bool rpc_softuart_master::get_bytes(uint8_t *data, uint32_t len, int timeout)
{
uint32_t i = 0;
unsigned long end = millis() + timeout;
    
    while (millis() < end && i < len) {
        data[i++] = _sserial->read();
    }
    return (i == len);
} /* rpc_softuart_master::get_bytes() */

bool rpc_softuart_master::put_bytes(uint8_t *data, uint32_t data_len, int timeout)
{
   (void)timeout;
    _sserial->write(data, data_len);
    return true;
} /* rpc_softuart_master::put_bytes() */

//
// Register the callback function for the specific remote procedure
// returns false if memory has been exhausted
// or true for success
//
// This is used on the Slave side to manage execution of incoming commands
//
bool rpc_slave::register_callback(uint32_t rpc_id, RPC_CALLBACK pfnCB)
{
    if (_rpc_count >= MAX_CALLBACKS) // out of space to save this
        return false;
    _rpcList[_rpc_count].id = rpc_id;
    _rpcList[_rpc_count].pfnCallback = pfnCB;
    _rpc_count++; 
    return true;
} /* rpc_slave::register_callback() */
//
// Match the command id to the registered callback function
// returns NULL if none found
//
RPC_CALLBACK rpc_slave::find_callback(uint32_t rpc_id)
{
int i;

  for (i=0; i<_rpc_count; i++) {
    if (rpc_id == _rpcList[i].id)
      return _rpcList[i].pfnCallback;
  }
  return NULL;
} /* rpc_slave::find_callback() */

//
// Make a remote function call
// 
// Used by the Master side to direct the Slave to execute a command
//
bool rpc_master::call(uint32_t rpc_id, uint8_t *out_data, uint32_t out_data_len, uint8_t *in_data, uint32_t *in_data_len, int send_timeout, int recv_timeout)
{
bool rc = false;

    if (put_command(rpc_id, out_data, out_data_len, send_timeout)) {
       rc = get_result(in_data, in_data_len, recv_timeout);
    }
    return rc;
} /* rpc_master::call() */

//
// Calculate a 16-bit CRC value for a stream of data bytes
//
uint16_t  RPC::crc16(uint8_t *data, uint32_t len)
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
} /* RPC::crc16() */
//
// Construct and send a command packet from Master to Slave
//
bool rpc_master::put_command(uint32_t cmd, uint8_t *data, uint32_t data_len, int timeout)
{
unsigned long start, end;
bool rc = false;
uint8_t ucTemp[8];
uint32_t *uiTemp = (uint32_t *)ucTemp;

    start = millis();
    end = start + timeout;
    while (millis() < end) {
        uiTemp[0] = cmd; uiTemp[1] = data_len; 
        put_packet(__COMMAND_HEADER_PACKET_MAGIC, ucTemp, 8, 10);
        if (get_packet(__COMMAND_HEADER_PACKET_MAGIC, ucTemp, 0, 20)) {
            put_packet(__COMMAND_DATA_PACKET_MAGIC, data, data_len, 5000);
            if (get_packet(__COMMAND_DATA_PACKET_MAGIC, ucTemp, 0, 20))
                rc = true;
        }
    } // while waiting for main timeout
    return rc;
} /* rpc_master::put_command() */
//
// Read the resulting data from a command (Master calls to get from Slave)
//
bool rpc_master::get_result(uint8_t *data, uint32_t *data_len, int timeout)
{
unsigned long start, end;
bool rc = false;
uint32_t len;

    start = millis();
    end = start + timeout;
    while (millis() < end) {
        put_packet(__RESULT_HEADER_PACKET_MAGIC, data, 0, 10);
        if (get_packet(__RESULT_HEADER_PACKET_MAGIC, data, 4, 20)) {
            len = *(uint32_t *)data; 
            put_packet(__RESULT_DATA_PACKET_MAGIC, data, 0, 10);
            if (get_packet(__RESULT_DATA_PACKET_MAGIC, data, len, 5000)) {
                rc = true;
                *data_len = len;
            }
        }
    } // while not timeout
    return rc;
} /* rpc_master::get_result() */

//
// Receive a packet from a RPC device
// Confirms matching magic value and calculated CRC
//
// returns true if the data was received + the magic value and crc match
//
bool RPC::get_packet(uint16_t magic_value, uint8_t *payload, uint32_t payload_len, int timeout)
{
bool rc = false;
unsigned long end;
uint32_t i, len = payload_len + 4;
uint16_t crc, in_magic, in_crc;

    end = millis() + timeout;
    while (millis() < end && !rc) {
        // Confirm the packet has the right length, magic number and CRC
        if (get_bytes(payload, len, timeout)) { // got the length requested
            in_magic = payload[0] | (payload[1] << 8);
            crc = crc16(payload, len-2);
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
} /* RPC::get_packet() */
//
// Send a packet to a RPC device
//
// Generates a crc value for the given data, repacks it into a homogenous
// buffer and then transmits it in a single transaction
//
// returns true if the transmission succeeds
//
bool RPC::put_packet(uint16_t magic_value, uint8_t *data, uint32_t data_len, int timeout)
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
    crc = crc16(ucTemp, len);
    ucTemp[len++] = (uint8_t)crc;
    ucTemp[len++] = (uint8_t)(crc >> 8);
    rc = put_bytes(ucTemp, len, timeout);

return rc;
} /* RPC::put_packet() */

//
// Wait to receive a command from the master
//
uint32_t rpc_slave::get_command(uint8_t *data, uint32_t *data_len)
{
unsigned long end;
uint32_t len = 0, cmd = 0;
uint8_t ucTemp[MIN_PACKET_SIZE];

    end = millis() + 100;
    while (millis() < end && cmd != 0) {
        if (get_packet(__COMMAND_HEADER_PACKET_MAGIC, data, 8, 10)) {
            cmd = *(uint32_t *)&data[0];
            len = *(uint32_t *)&data[4];
            put_packet(__COMMAND_HEADER_PACKET_MAGIC, data, 0, 10); // send ack
            if (get_packet(__COMMAND_DATA_PACKET_MAGIC, data, len, 5000)) {
                put_packet(__COMMAND_DATA_PACKET_MAGIC, ucTemp, 0, 10); // send ack
            } else { // something went wrong, nullify it
                cmd = len = 0;
            }
        }
    } // while waiting for incoming data
    *data_len = len;
    return cmd;
} /* rpc_slave::get_command() */

//
// Send the master the result of the last command
//
void rpc_slave::put_result(uint8_t *data, uint32_t data_len)
{
bool done = false;
unsigned long end;
uint8_t ucTemp[MIN_PACKET_SIZE];

    end = millis() + 100;
    while (millis() < end && !done) {
        if (get_packet(__RESULT_HEADER_PACKET_MAGIC, ucTemp, 0, 10)) {
            *(uint32_t *)ucTemp = data_len;
            // send length first as ack
            put_packet(__RESULT_HEADER_PACKET_MAGIC, ucTemp, 4, 10);
            if (get_packet(__RESULT_DATA_PACKET_MAGIC, ucTemp, 0, 10)) {
                put_packet(__RESULT_DATA_PACKET_MAGIC, data, data_len, 5000);
                done = true;
            }
        }
    } // while not finished or not timed out
} /* rpc_slave::put_result() */
