//
// Arduino RPC (Remote Procedure Call) library
// Copyright (c) 2020 OpenMV
// written by Larry Bank
//
// project started April, 2020
//

#include <ArduinoRPC.h>

#ifdef _LINUX_
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>
#include <time.h>
#include <termios.h>

#else // Arduino

#ifndef HAL_ESP32_HAL_H_
#include <SoftwareSerial.h>
#endif // !ESP32
#include <Wire.h>
#include <SPI.h>
#endif // Arduino

// Need to declare storage for these static class variables
// which are part of the rpc_i2c_slave class
uint32_t rpc_i2c_slave::_receive_len;
uint32_t rpc_i2c_slave::_response_len;
uint8_t rpc_i2c_slave::_buf[MAX_LOCAL_BUFFER];
#ifdef _LINUX_
// on Linux the default maximum transfer size is 4K
#define MAX_SPI_BUF 4096
static uint8_t ucTXBuf[MAX_SPI_BUF];
static uint8_t ucRXBuf[MAX_SPI_BUF];

#define DEBUG_MSG(n) printf(n)
#define pgm_read_byte(n) (*n)

int OpenSerialPort(char *szName, int iBaud)
{
struct termios toptions;
int fdSerial;

  fdSerial = open(szName, O_RDWR | O_NDELAY);
  if (fdSerial < 0)
    return -1;
  tcgetattr(fdSerial, &toptions);
  cfsetospeed(&toptions, iBaud);
  cfsetispeed(&toptions, iBaud);
// These options must be set up properly or it won't work
// By default, Linux likes to use CANONICAL mode which filters the data stream
// for special control characters and waits for ENTER to continue
/* 8 bits, no parity, no stop bits */
  toptions.c_cflag &= ~PARENB;
  toptions.c_cflag &= ~CSTOPB;
  toptions.c_cflag &= ~CSIZE;
  toptions.c_cflag |= CS8; // 8 bit, no parity, 1 stop bit
  /* no hardware flow control */
  toptions.c_cflag &= ~CRTSCTS;
  /* enable receiver, ignore status lines */
  toptions.c_cflag |= CREAD | CLOCAL;
  /* disable input/output flow control, disable restart chars */
  toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
  /* disable canonical input, disable echo,
  disable visually erase chars,
  disable terminal-generated signals */
  toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  /* disable output processing */
// toptions.c_oflag &= ~OPOST;
  toptions.c_oflag = 0; // raw output

  toptions.c_cc[VMIN] = 0;
  /* wait up to 200ms for data to arrive */
  toptions.c_cc[VTIME] = 0; //2;

  tcflush(fdSerial, TCIFLUSH);
  tcsetattr(fdSerial, TCSANOW, &toptions);
  usleep(500*1000); // wait 1/2 second for baud rate change
  return fdSerial;

} /* OpenSerialPort() */

unsigned long millis()
{
unsigned long iTime;
struct timespec res;

    clock_gettime(CLOCK_MONOTONIC, &res);
    iTime = 1000*res.tv_sec + res.tv_nsec/1000000;

    return iTime;
} /* millis() */

void delay(unsigned int iMillis)
{
  usleep(iMillis * 1000);
}
#else
#define DEBUG_MSG(n) Serial.println(n)
#endif

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
      cmd = get_command(ucTemp, &len, 1000);
      if (cmd) {
        DEBUG_MSG("Got command");
        pfnCB = find_callback(cmd);
        if (pfnCB) { // we registered a callback for this command
          DEBUG_MSG("Valid command");
          new_len = (*pfnCB)(cmd, ucTemp, len);
            if (put_result(ucTemp, new_len, 1000)) {
                DEBUG_MSG("Result sent successfully");
                if (_schedule_cb != NULL) { // call another function indicating success
                    (*_schedule_cb)(cmd, ucTemp, len);
                    _schedule_cb = NULL; // only call it once
                }
            }
        } else { // no callback to respond
          DEBUG_MSG("Command not registered");
          put_result(ucTemp, 0, 1000); // DEBUG - need a NACK
        }
      }
    }
} /* rpc_slave::loop() */

#ifdef _LINUX_
rpc_i2c_master::rpc_i2c_master(int iAddr, int iBus)
{
char filename[32];

  sprintf(filename, "/dev/i2c-%d", iBus); // I2C bus number
  if ((_iHandle = open(filename, O_RDWR)) < 0)
     return;
  if (ioctl(_iHandle, I2C_SLAVE, iAddr) < 0) // set slave address
  {
     close(_iHandle);
     _iHandle = 0;
  }

} /* rpc_i2c_master::rpc_i2c_master() */
// There is no I2C slave class on Linux
#else // Arduino
rpc_i2c_master::rpc_i2c_master(int iAddr, unsigned long speed)
{
    _iAddr = iAddr;
    Wire.begin();
    Wire.setClock(speed);

} /* rpc_i2c_master::rpc_i2c_master() */

rpc_i2c_slave::rpc_i2c_slave(int iAddr, unsigned long speed)
{
    Wire.begin(iAddr); // register as slave at address 'iAddr'
    Wire.setClock(speed);
    Wire.onReceive(receive_event); // register callback event
    Wire.onRequest(request_event);
    _receive_len = _response_len = 0;
} /* rpc_i2c_slave::rpc_i2c_slave() */
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
#endif // LINUX

#ifndef _LINUX_
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

} /* rpc_i2c_slave::put_bytes() */
#endif // !_LINUX_

#ifdef _LINUX_
bool rpc_i2c_master::get_bytes(uint8_t *data, uint32_t len, int timeout)
{
int rc;

  (void)timeout;
  rc = read(_iHandle, data, len);
  return (rc > 0); 
} /* rpc_i2c_master::get_bytes() */

bool rpc_i2c_master::put_bytes(uint8_t *data, uint32_t len, int timeout)
{
int rc;

  (void)timeout;
  rc = write(_iHandle, data, len);
  return (rc > 0);
} /* rpc_i2c_master::put_bytes() */

#else // Arduino
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
#endif // _LINUX_

#ifdef _LINUX_
rpc_spi_master::rpc_spi_master(int iBus, int speed)
{
char szName[32];
int rc, iSPIMode = SPI_MODE_0;

  sprintf(szName, "/dev/spidev%d.0", iBus);
  _iHandle = open(szName, O_RDWR);
  if (_iHandle >= 0)
  {
    rc = ioctl(_iHandle, SPI_IOC_WR_MODE, &iSPIMode);
    if (rc >= 0)
       ioctl(_iHandle, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    memset(&xfer, 0, sizeof(xfer));
    xfer.speed_hz = speed; 
    xfer.bits_per_word = 8;
    xfer.tx_buf = (unsigned long long)ucTXBuf;
    xfer.rx_buf = (unsigned long long)ucRXBuf;
  }
} /* rpc_spi_master::rpc_spi_master() */

bool rpc_spi_master::get_bytes(uint8_t *data, uint32_t len, int timeout)
{
int rc;

    (void)timeout;
    xfer.len = len;
    rc = ioctl(_iHandle, SPI_IOC_MESSAGE(1), &xfer);
    memcpy(data, ucRXBuf, len);
    return (rc >= 0);
} /* rpc_spi_master::get_bytes() */

bool rpc_spi_master::put_bytes(uint8_t *data, uint32_t len, int timeout)
{
int rc;

  (void)timeout;
  xfer.len = len;
  memcpy(ucTXBuf, data, len);
  rc = ioctl(_iHandle, SPI_IOC_MESSAGE(1), &xfer);
  return (rc >= 0); 
} /* rpc_spi_master::put_bytes() */
#else // Arduino
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
#endif // _LINUX_

#ifdef _LINUX_
rpc_uart_master::rpc_uart_master(char *port_name, int speed)
{
  _iHandle = OpenSerialPort(port_name, speed);
} /* rpc_uart_master::rpc_uart_master() */

bool rpc_uart_master::get_bytes(uint8_t *data, uint32_t len, int timeout)
{
int rc;
unsigned long iEnd;
uint8_t *pBuf = data;

  iEnd = millis() + timeout;
  if (_iHandle >= 0)
  {   
    while (len && millis() < iEnd)
    {
      rc = read(_iHandle, pBuf, len);
      if (rc <= 0)
      {
        usleep(2000); // 2ms to wait for data to arrive
      }
      else
      {
        len -= rc;
        pBuf += rc;
      }
    }
  }
  return (len == 0);
} /* rpc_uart_master::get_bytes() */

bool rpc_uart_master::put_bytes(uint8_t *data, uint32_t len, int timeout)
{
int rc = -1;

  if (_iHandle >= 0)
  {
    rc = write(_iHandle, data, len);
    // This waits for data to full transmit before returning
    // If we don't do this, multiple requests will get corrupted
    tcdrain(_iHandle);
  }
  return (rc >= 0);
} /* rpc_uart_master::put_bytes() */

#else // Arduino
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

#endif // _LINUX_

#ifdef _LINUX_
rpc_uart_slave::rpc_uart_slave(char *port_name, int speed)
{
  _iHandle = OpenSerialPort(port_name, speed);
} /* rpc_uart_slave::rpc_uart_slave() */

bool rpc_uart_slave::get_bytes(uint8_t *data, uint32_t len, int timeout)
{
int rc;
unsigned long iEnd;
uint8_t *pBuf = data;

  iEnd = millis() + timeout;
  if (_iHandle >= 0)
  {
    while (len && millis() < iEnd)
    {
      rc = read(_iHandle, pBuf, len);
      if (rc <= 0)
      {
        usleep(2000); // 2ms to wait for data to arrive
      }
      else
      {
        len -= rc;
        pBuf += rc;
      }
    }
  }
  return (len == 0);
} /* rpc_uart_slave::get_bytes() */

bool rpc_uart_slave::put_bytes(uint8_t *data, uint32_t len, int timeout)
{
int rc = -1;

  if (_iHandle >= 0)
  {
    rc = write(_iHandle, data, len);
    // This waits for data to full transmit before returning
    // If we don't do this, multiple requests will get corrupted
    tcdrain(_iHandle);
  }
  return (rc >= 0);
} /* rpc_uart_slave::put_bytes() */

#else // Arduino
rpc_uart_slave::rpc_uart_slave(unsigned long speed)
{
    Serial.begin(speed);
} /* rpc_uart_slave::rpc_uart_slave() */

bool rpc_uart_slave::get_bytes(uint8_t *data, uint32_t len, int timeout)
{
unsigned long end = millis() + timeout;
uint32_t i = 0;
    
    // wait for data to arrive
    while (millis() < end && i < len) {
        if (Serial.available() > 0)
            data[i++] = Serial.read();
        else
            delay(1); // allow time for data to arrive
    }
    return (i == len);
} /* rpc_uart_slave::get_bytes() */

bool rpc_uart_slave::put_bytes(uint8_t *data, uint32_t len, int timeout)
{
unsigned long end = millis() + timeout;
uint32_t i = 0;
    
    while (millis() < end && i < len) {
        Serial.write(data[i]); // small MCUs can't handle a single, large write, so do it byte by byte
    }
    return (i == len); // indicates data was read by master

} /* rpc_uart_slave::put_bytes() */
#endif // _LINUX_

#if !defined( HAL_ESP32_HAL_H_ ) && !defined( _LINUX_ )
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

rpc_softuart_slave::rpc_softuart_slave(int pin1, int pin2, unsigned long speed)
{
    _sserial = new SoftwareSerial(pin1, pin2);
    _sserial->begin(speed);
} /* rpc_softuart_slave::rpc_softuart_slave() */

bool rpc_softuart_slave::get_bytes(uint8_t *data, uint32_t len, int timeout)
{
unsigned long end = millis() + timeout;
uint32_t i = 0;
    
    // wait for data to arrive
    while (millis() < end && i < len) {
        if (_sserial->available() > 0)
            data[i++] = _sserial->read();
        else
            delay(1); // allow time for data to arrive
    }
    return (i == len);
} /* rpc_softuart_slave::get_bytes() */

bool rpc_softuart_slave::put_bytes(uint8_t *data, uint32_t len, int timeout)
{
unsigned long end = millis() + timeout;
uint32_t i = 0;
    
    while (millis() < end && i < len) {
        _sserial->write(data[i]); // small MCUs can't handle a single, large write, so do it byte by byte
    }
    return (i == len); // indicates data was read by master

} /* rpc_softuart_slave::put_bytes() */
#endif // !ESP32 && !_LINUX_

//
// Register the callback function for the specific remote procedure
// returns false if memory has been exhausted
// or true for success
//
// This is used on the Slave side to manage execution of incoming commands
//
bool rpc_slave::register_callback(const char *name, RPC_CALLBACK pfnCB)
{
uint32_t rpc_id;

    if (_rpc_count >= MAX_CALLBACKS) // out of space to save this
        return false;
    rpc_id = hash(name);
    _rpcList[_rpc_count].id = rpc_id;
    _rpcList[_rpc_count].pfnCallback = pfnCB;
    _rpc_count++; 
    return true;
} /* rpc_slave::register_callback() */
//
// Schedule a callback for when RPC handshake has completed and
// the slave and master are in sync. This can be used to initiate
// higher speed transfers without back-and-forth handshake protocol
//
void rpc_slave::schedule_callback(RPC_CALLBACK pfnCB)
{
    _schedule_cb = pfnCB;
} /* rpc_slave::schedule_callback() */

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
bool rpc_master::call(const char *name, uint8_t *out_data, uint32_t out_data_len, uint8_t *in_data, uint32_t *in_data_len, int send_timeout, int recv_timeout)
{
bool rc = false;
uint32_t rpc_id;

    DEBUG_MSG("About to send command");
    rpc_id = hash(name);
    if (put_command(rpc_id, out_data, out_data_len, send_timeout)) {
        DEBUG_MSG("Command sent, awaiting result...");
       rc = get_result(in_data, in_data_len, recv_timeout);
        if (rc)
            DEBUG_MSG("Result received");
        else
            DEBUG_MSG("Error receiving result");
    }
    return rc;
} /* rpc_master::call() */
//
// Calculate the string hash value
// on AVR, the string will be in FLASH/PROGMEM
//
uint32_t RPC::hash(const char *name)
{
uint32_t c, h = 5381;

  c = pgm_read_byte(name++);
  while (c) {
     h = ((h << 5UL) + h) ^ c;
     c = pgm_read_byte(name++);
  }
  return h;
} /* RPC::hash() */
//
// Calculate a 16-bit CRC value for a stream of data bytes
//
uint16_t  RPC::crc16(uint8_t *data, uint32_t len)
{
uint8_t c, *d = data;
uint16_t crc = 0xFFFF;

    for(uint32_t i=0; i<len; i++) {
//        crc ^= (d[i] << 8);
//        for(int j=0; j<8; j++) {
//            crc = (crc << 1);
//            if (crc & 0x8000)
//                crc ^= 0x1021;
//        } // for j
        c = crc >> 8 ^ *d++;
        c ^= c>>4;
        crc = (crc << 8) ^ ((unsigned short)(c << 12)) ^ ((unsigned short)(c << 5)) ^ ((unsigned short)c);
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
uint32_t *uiTemp = (uint32_t *)&ucTemp[0];

    start = millis();
    end = start + timeout;
    _put_short_timeout = _put_short_timeout_reset;
    _get_short_timeout = _get_short_timeout_reset;
    while (millis() < end) {
        uiTemp[0] = cmd; uiTemp[1] = data_len; 
        put_packet(__COMMAND_HEADER_PACKET_MAGIC, ucTemp, 8, _put_short_timeout);
        if (get_packet(__COMMAND_HEADER_PACKET_MAGIC, ucTemp, 0, _get_short_timeout)) {
            put_packet(__COMMAND_DATA_PACKET_MAGIC, data, data_len, _put_long_timeout);
            if (get_packet(__COMMAND_DATA_PACKET_MAGIC, ucTemp, 0, _get_short_timeout))
                rc = true;
        }
        _put_short_timeout = (_put_short_timeout * 6) / 4;
        _get_short_timeout = (_get_short_timeout * 6) / 4;
    } // while waiting for main timeout
    if (!rc)
       DEBUG_MSG("Timed out trying to send a packet");
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
    _put_short_timeout = _put_short_timeout_reset;
    _get_short_timeout = _get_short_timeout_reset;
    while (millis() < end) {
        put_packet(__RESULT_HEADER_PACKET_MAGIC, data, 0, _put_short_timeout);
        if (get_packet(__RESULT_HEADER_PACKET_MAGIC, data, 4, _get_short_timeout)) {
            len = *(uint32_t *)data; 
            put_packet(__RESULT_DATA_PACKET_MAGIC, data, 0, _put_short_timeout);
            if (get_packet(__RESULT_DATA_PACKET_MAGIC, data, len, _get_long_timeout)) {
                rc = true;
                *data_len = len;
            }
        }
        _put_short_timeout = (_put_short_timeout * 6) / 4;
        _get_short_timeout = (_get_short_timeout * 6) / 4;
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
            in_magic = payload[0] | ((uint16_t)payload[1] << 8);
//            Serial.print("in_magic = 0x");
//            Serial.println(in_magic, HEX);
            crc = crc16(payload, len-2);
            in_crc = payload[len-2] | (payload[len-1] << 8);
//            Serial.print("crc match? ");
//            if (crc == in_crc) Serial.println("yes");
//            else Serial.println("no");
//            Serial.print("magic match? ");
//            if (in_magic == magic_value) Serial.println("yes");
//            else Serial.println("no");
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
uint32_t len;
uint8_t ucTemp[MAX_LOCAL_BUFFER]; // this limits the amount of data we can send


// It's best to combine all of the bytes we're going to transmit into a single
// buffer so that they can be sent in a single comm transaction

    ucTemp[0] = (uint8_t)magic_value; // start with 2 bytes of magic value
    ucTemp[1] = (uint8_t)(magic_value >> 8);
    len = 2;
    if (data != NULL && data_len > 0)
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
uint32_t rpc_slave::get_command(uint8_t *data, uint32_t *data_len, int timeout)
{
unsigned long end;
uint32_t len = 0, cmd = 0;
uint8_t ucTemp[MIN_PACKET_SIZE];

    end = millis() + timeout;
    _put_short_timeout = _put_short_timeout_reset;
    _get_short_timeout = _get_short_timeout_reset;
    while (millis() < end && cmd != 0) {
        if (get_packet(__COMMAND_HEADER_PACKET_MAGIC, data, 8, _get_short_timeout)) {
            cmd = *(uint32_t *)&data[0];
            len = *(uint32_t *)&data[4];
            put_packet(__COMMAND_HEADER_PACKET_MAGIC, data, 0, _put_short_timeout); // send ack
            if (get_packet(__COMMAND_DATA_PACKET_MAGIC, data, len, _get_long_timeout)) {
                put_packet(__COMMAND_DATA_PACKET_MAGIC, ucTemp, 0, _put_short_timeout); // send ack
            } else { // something went wrong, nullify it
                cmd = len = 0;
            }
        }
       _put_short_timeout++;
       _get_short_timeout++;
    } // while waiting for incoming data
    *data_len = len;
    return cmd;
} /* rpc_slave::get_command() */

//
// Send the master the result of the last command
//
bool rpc_slave::put_result(uint8_t *data, uint32_t data_len, int timeout)
{
bool done = false;
unsigned long end;
uint8_t ucTemp[MIN_PACKET_SIZE];
uint32_t *p32 = (uint32_t *)ucTemp;

    end = millis() + timeout;
    _put_short_timeout = _put_short_timeout_reset;
    _get_short_timeout = _get_short_timeout_reset;
    while (millis() < end && !done) {
        if (get_packet(__RESULT_HEADER_PACKET_MAGIC, ucTemp, 0, _get_short_timeout)) {
            p32[0] = data_len;
            // send length first as ack
            put_packet(__RESULT_HEADER_PACKET_MAGIC, ucTemp, 4, _put_short_timeout);
            if (get_packet(__RESULT_DATA_PACKET_MAGIC, ucTemp, 0, _get_short_timeout)) {
                put_packet(__RESULT_DATA_PACKET_MAGIC, data, data_len, _put_long_timeout);
                done = true;
            }
        }
        _put_short_timeout++;
        _get_short_timeout++;
    } // while not finished or not timed out
    return done; // returns true if it didn't time out
} /* rpc_slave::put_result() */
