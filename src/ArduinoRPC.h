//
// Arduino RPC (Remote Procedure Call) library
// Copyright (c) 2020 OpenMV
// written by Kwabena Agyeman and Larry Bank
//
// project started April, 2020
//

#ifndef __ARDUINO_RPC__
#define __ARDUINO_RPC__

#include <Arduino.h>
#include <SoftwareSerial.h>

//
// Prefix sent at the beginning of every communication packet
//
const uint16_t __COMMAND_HEADER_PACKET_MAGIC = 0x1209;
const uint16_t __COMMAND_DATA_PACKET_MAGIC = 0xABD1;
const uint16_t __RESULT_HEADER_PACKET_MAGIC = 0x9021;
const uint16_t __RESULT_DATA_PACKET_MAGIC = 0x1DBA;

//
// The list of registered callback functions for remote procedure calls
// is limited in length by the amount of available RAM. On AVR targets,
// using a std::map or dictionary library would use too much FLASH and RAM
// so we'll use a fixed size list instead. For non-AVR targets, we can allow
// more simultaneous callbacks.
//
// The local buffer size is a stack variable used to receive data packets.
// As of this first version, receiving image data on an Arduino is not
// supported.
//
#ifdef __AVR__
#define MAX_CALLBACKS 16
#define MAX_LOCAL_BUFFER 32
#else
#define MAX_CALLBACKS 32
#define MAX_LOCAL_BUFFER 256
#endif

// Minimum sized buffer needed to process an empty packet
#define MIN_PACKET_SIZE 8

// The address of the I2C slave. This is an arbitrary value that was chosen
// to not interfere with existing devices.
#define I2C_ADDR 0x12

// Callback function prototype
typedef uint32_t (RPC_CALLBACK)(int iEvent, uint8_t *data, uint32_t data_len);

//
// Structure holding the list of function IDs matched with their
// corresponding callback function.
//
typedef struct tagrpclist
{
  RPC_CALLBACK *pfnCallback;
  int id;
} RPCLIST;
//
// Supported communication types
//
enum
{
  RPC_UART=0,
  RPC_SOFTUART,
  RPC_I2C,
  RPC_CAN,
  RPC_SPI,
  RPC_USB  // VCP - virtual comm port on USB
};

class RPC
{
  public:
    bool begin(int comm_type, unsigned long speed, int pin1, int pin2);
    bool begin(int comm_type, unsigned long speed);
// for Master
    bool register_callback(int rpc_id, RPC_CALLBACK *);
    bool call(int rpc_id, uint8_t *out_data, uint32_t out_data_len, uint8_t *in_data, uint32_t *in_data_len, int send_timeout, int recv_timeout);
// for Slave
    uint32_t get_command(uint8_t *data, uint32_t *data_len);
    RPC_CALLBACK *find_callback(uint32_t rpc_id);
    void put_result(uint8_t *data, uint32_t data_len);

  private:
    uint16_t _crc16(uint8_t *data, uint32_t len);
    bool _get_packet(uint16_t magic_value, uint8_t *data, uint32_t data_len, int timeout);
    bool _put_packet(uint16_t magic_value, uint8_t *data, uint32_t data_len, int timeout);
    bool _put_command(int cmd, uint8_t *data, uint32_t data_len, int timeout);
    bool _get_result(uint8_t *data, uint32_t *data_len, int timeout);
    bool _get_bytes(uint8_t *data, uint32_t len, int timeout);
    bool _put_bytes(uint8_t *data, uint32_t data_len, int timeout);

    int _comm_type; // communication method (e.g. UART/I2C/SPI/USB/CAN)
    unsigned long _speed; // baud/bit rate
    int _pin1, _pin2; // optional pins to specify the comm bus
    SoftwareSerial *_sserial;
    RPCLIST _rpcList[MAX_CALLBACKS]; // list of registered callbacks
    int _rpc_count; // number of registered callback functions
};

#endif // __ARDUINO_RPC__

