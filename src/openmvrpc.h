//
// OpenMV RPC (Remote Procedure Call) Library
// Copyright (c) 2020 OpenMV
// Written by Larry Bank & Kwabena W. Agyeman
//

#ifndef __OPENMVRPC__
#define __OPENMVRPC__

#include <Arduino.h>
#include <CAN.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>

namespace openmv {

typedef void (*rpc_callback_t)(uint8_t *in_data, size_t in_data_len, uint8_t **out_data, size_t *out_data_len);
typedef void (*rpc_plain_callback_t)();

typedef struct rpc_callback_entry {
    uint32_t key;
    rpc_callback_t value;
} rpc_callback_entry_t;

typedef void (*rpc_stream_reader_callback_t)(uint8_t *in_data, uint32_t in_data_len);
typedef void (*rpc_stream_writer_callback_t)(uint8_t **out_data, uint32_t *out_data_len);

class rpc
{
public:
    rpc(uint8_t *buff, size_t buff_len);
    ~rpc() {}
    virtual bool get_bytes(uint8_t *buff, size_t size, unsigned long timeout) = 0;
    virtual bool put_bytes(uint8_t *data, size_t size, unsigned long timeout) = 0;
    void stream_reader(rpc_stream_reader_callback_t callback, unsigned long queue_depth = 1, unsigned long read_timeout = 5000);
    void stream_writer(rpc_stream_writer_callback_t callback, unsigned long write_timeout = 5000);
protected:
    const uint16_t _COMMAND_HEADER_PACKET_MAGIC = 0x1209;
    const uint16_t _COMMAND_DATA_PACKET_MAGIC = 0xABD1;
    const uint16_t _RESULT_HEADER_PACKET_MAGIC = 0x9021;
    const uint16_t _RESULT_DATA_PACKET_MAGIC = 0x1DBA;
    const unsigned long _put_long_timeout = 5000;
    const unsigned long _get_long_timeout = 5000;
    unsigned long _put_short_timeout;
    unsigned long _get_short_timeout;
    void _zero(uint8_t *data, size_t size);
    bool _same(uint8_t *data, size_t size);
    uint32_t _hash(const __FlashStringHelper *name);
    uint32_t _hash(const char *name, size_t length);
    uint32_t _hash(const char *name);
    bool _get_packet(uint16_t magic_value, uint8_t *buff, size_t size, unsigned long timeout);
    void _set_packet(uint8_t *buff, uint16_t magic_value, uint8_t *data, size_t size);
    virtual void _flush() {}
    virtual bool _stream_get_bytes(uint8_t *buff, size_t size, unsigned long timeout);
    virtual bool _stream_put_bytes(uint8_t *data, size_t size, unsigned long timeout);
    uint8_t *_buff;
    size_t _buff_len;
    unsigned long _stream_writer_queue_depth_max;
private:
    rpc(const rpc &);
    uint16_t __crc_16(uint8_t *data, size_t size);
};

class rpc_master : public rpc
{
public:
    rpc_master(uint8_t *buff, size_t buff_len);
    ~rpc_master() {}
    bool call_no_copy_no_args(const __FlashStringHelper *name,
                              void **result_data=NULL, size_t *result_data_len=NULL,
                              unsigned long send_timeout=1000, unsigned long recv_timeout=1000);
    bool call_no_copy_no_args(const String &name,
                              void **result_data=NULL, size_t *result_data_len=NULL,
                              unsigned long send_timeout=1000, unsigned long recv_timeout=1000);
    bool call_no_copy_no_args(const char *name,
                              void **result_data=NULL, size_t *result_data_len=NULL,
                              unsigned long send_timeout=1000, unsigned long recv_timeout=1000);
    bool call_no_copy(const __FlashStringHelper *name,
                      void *command_data, size_t command_data_len,
                      void **result_data=NULL, size_t *result_data_len=NULL,
                      unsigned long send_timeout=1000, unsigned long recv_timeout=1000);
    bool call_no_copy(const String &name,
                      void *command_data, size_t command_data_len,
                      void **result_data=NULL, size_t *result_data_len=NULL,
                      unsigned long send_timeout=1000, unsigned long recv_timeout=1000);
    bool call_no_copy(const char *name,
                      void *command_data, size_t command_data_len,
                      void **result_data=NULL, size_t *result_data_len=NULL,
                      unsigned long send_timeout=1000, unsigned long recv_timeout=1000);
    bool call_no_args(const __FlashStringHelper *name,
                      void *result_data=NULL, size_t result_data_len=0, bool return_false_if_received_data_is_zero=true,
                      unsigned long send_timeout=1000, unsigned long recv_timeout=1000);
    bool call_no_args(const String &name,
                      void *result_data=NULL, size_t result_data_len=0, bool return_false_if_received_data_is_zero=true,
                      unsigned long send_timeout=1000, unsigned long recv_timeout=1000);
    bool call_no_args(const char *name,
                      void *result_data=NULL, size_t result_data_len=0, bool return_false_if_received_data_is_zero=true,
                      unsigned long send_timeout=1000, unsigned long recv_timeout=1000);
    bool call(const __FlashStringHelper *name,
              void *command_data, size_t command_data_len,
              void *result_data=NULL, size_t result_data_len=0, bool return_false_if_received_data_is_zero=true,
              unsigned long send_timeout=1000, unsigned long recv_timeout=1000);
    bool call(const String &name,
              void *command_data, size_t command_data_len,
              void *result_data=NULL, size_t result_data_len=0, bool return_false_if_received_data_is_zero=true,
              unsigned long send_timeout=1000, unsigned long recv_timeout=1000);
    bool call(const char *name,
              void *command_data, size_t command_data_len,
              void *result_data=NULL, size_t result_data_len=0, bool return_false_if_received_data_is_zero=true,
              unsigned long send_timeout=1000, unsigned long recv_timeout=1000);
protected:
    const unsigned long _put_short_timeout_reset = 3;
    const unsigned long _get_short_timeout_reset = 3;
private:
    rpc_master(const rpc_master &);
    uint8_t __in_command_header_buf[4];
    uint8_t __in_command_data_buf[4];
    uint8_t __out_result_header_ack[4];
    uint8_t __in_result_header_buf[8];
    uint8_t __out_result_data_ack[4];
    bool __put_command(uint32_t command, uint8_t *data, size_t size, unsigned long timeout);
    bool __get_result(uint8_t **data, size_t *size, unsigned long timeout);
};

class rpc_slave : public rpc
{
public:
    rpc_slave(uint8_t *buff, size_t buff_len, rpc_callback_entry_t *callback_dict, size_t callback_dict_len);
    ~rpc_slave() {}
    bool register_callback(const char *name, rpc_callback_t callback);
    void schedule_callback(rpc_plain_callback_t callback);
    void setup_loop_callback(rpc_plain_callback_t callback);
    void loop(unsigned long send_timeout=1000, unsigned long recv_timeout=1000);
protected:
    const unsigned long _put_short_timeout_reset = 2;
    const unsigned long _get_short_timeout_reset = 2;
private:
    rpc_slave(const rpc_slave &);
    rpc_callback_entry_t *__dict;
    size_t __dict_len;
    size_t __dict_alloced = 0;
    rpc_plain_callback_t __schedule_cb = NULL;
    rpc_plain_callback_t __loop_cb = NULL;
    uint8_t __in_command_header_buf[12];
    uint8_t __out_command_header_ack[4];
    uint8_t __out_command_data_ack[4];
    uint8_t __in_response_header_buf[4];
    uint8_t __in_response_data_buf[4];
    bool __get_command(uint32_t *command, uint8_t **data, size_t *size, unsigned long timeout);
    bool __put_result(uint8_t *data, size_t size, unsigned long timeout);
};

class rpc_can_master : public rpc_master
{
public:
    rpc_can_master(uint8_t *buff, size_t buff_len,
                   long message_id=0x7FF, long bit_rate=250E3);
    ~rpc_can_master();
    virtual void _flush() override;
    virtual bool get_bytes(uint8_t *buff, size_t size, unsigned long timeout) override;
    virtual bool put_bytes(uint8_t *data, size_t size, unsigned long timeout) override;
    void set_message_id(long message_id) { __message_id = message_id; }
    long get_message_id() { return __message_id; }
private:
    long __message_id;
    rpc_can_master(const rpc_can_master &);
};

class rpc_can_slave : public rpc_slave
{
public:
    rpc_can_slave(uint8_t *buff, size_t buff_len,
                  rpc_callback_entry_t *callback_dict, size_t callback_dict_len,
                  long message_id=0x7FF, long bit_rate=250E3);
    ~rpc_can_slave();
    virtual void _flush() override;
    virtual bool get_bytes(uint8_t *buff, size_t size, unsigned long timeout) override;
    virtual bool put_bytes(uint8_t *data, size_t size, unsigned long timeout) override;
    void set_message_id(long message_id) { __message_id = message_id; }
    long get_message_id() { return __message_id; }
private:
    long __message_id;
    rpc_can_slave(const rpc_can_slave &);
};

class rpc_i2c_master : public rpc_master
{
public:
    rpc_i2c_master(uint8_t *buff, size_t buff_len,
                   int slave_addr=0x12, unsigned long rate=100000);
    ~rpc_i2c_master() {}
    virtual void _flush() override;
    virtual bool get_bytes(uint8_t *buff, size_t size, unsigned long timeout) override;
    virtual bool put_bytes(uint8_t *data, size_t size, unsigned long timeout) override;
    void set_slave_addr(int slave_addr) { __slave_addr = slave_addr; }
    int get_slave_addr() { return __slave_addr; }
private:
    int __slave_addr;
    unsigned long __rate;
    rpc_i2c_master(const rpc_i2c_master &);
};

class rpc_i2c_slave : public rpc_slave
{
public:
    rpc_i2c_slave(uint8_t *buff, size_t buff_len, 
                  rpc_callback_entry_t *callback_dict, size_t callback_dict_len,
                  int slave_addr=0x12);
    ~rpc_i2c_slave() {}
    virtual void _flush() override;
    virtual bool get_bytes(uint8_t *buff, size_t size, unsigned long timeout) override;
    virtual bool put_bytes(uint8_t *data, size_t size, unsigned long timeout) override;
private:
    int __slave_addr;
    rpc_i2c_slave(const rpc_i2c_slave &);
};

class rpc_spi_master : public rpc_master
{
public:
    rpc_spi_master(uint8_t *buff, size_t buff_len,
                   unsigned long cs_pin, unsigned long freq=1000000, unsigned long spi_mode=SPI_MODE2);
    ~rpc_spi_master();
    virtual bool get_bytes(uint8_t *buff, size_t size, unsigned long timeout) override;
    virtual bool put_bytes(uint8_t *data, size_t size, unsigned long timeout) override;
    void set_cs_pin(unsigned long cs_pin) { pinMode(__cs_pin, INPUT); __cs_pin = cs_pin; pinMode(__cs_pin, OUTPUT); }
    unsigned long get_cs_pin() { return __cs_pin; }
private:
    unsigned long __cs_pin;
    SPISettings __settings;
    rpc_spi_master(const rpc_spi_master &);
};

#define RPC_HARDWARE_SERIAL_UART_MASTER(name) \
class rpc_hardware_serial##name##_uart_master : public rpc_master \
{ \
public: \
    rpc_hardware_serial##name##_uart_master(uint8_t *buff, size_t buff_len, \
                                            unsigned long baudrate=115200); \
    ~rpc_hardware_serial##name##_uart_master(); \
    virtual void _flush() override; \
    virtual bool get_bytes(uint8_t *buff, size_t size, unsigned long timeout) override; \
    virtual bool put_bytes(uint8_t *data, size_t size, unsigned long timeout) override; \
private: \
    rpc_hardware_serial##name##_uart_master(const rpc_hardware_serial##name##_uart_master &); \
};

#ifdef HAVE_HWSERIAL0
RPC_HARDWARE_SERIAL_UART_MASTER()
#endif

#ifdef HAVE_HWSERIAL1
RPC_HARDWARE_SERIAL_UART_MASTER(1)
#endif

#ifdef HAVE_HWSERIAL2
RPC_HARDWARE_SERIAL_UART_MASTER(2)
#endif

#ifdef HAVE_HWSERIAL3
RPC_HARDWARE_SERIAL_UART_MASTER(3)
#endif

#ifdef HAVE_CDCSERIAL
RPC_HARDWARE_SERIAL_UART_MASTER()
#endif

#define RPC_HARDWARE_SERIAL_UART_SLAVE(name) \
class rpc_hardware_serial##name##_uart_slave : public rpc_slave \
{ \
public: \
    rpc_hardware_serial##name##_uart_slave(uint8_t *buff, size_t buff_len, \
                                           rpc_callback_entry_t *callback_dict, size_t callback_dict_len, \
                                           unsigned long baudrate=115200); \
    ~rpc_hardware_serial##name##_uart_slave(); \
    virtual void _flush() override; \
    virtual bool get_bytes(uint8_t *buff, size_t size, unsigned long timeout) override; \
    virtual bool put_bytes(uint8_t *data, size_t size, unsigned long timeout) override; \
private: \
    rpc_hardware_serial##name##_uart_slave(const rpc_hardware_serial##name##_uart_slave &); \
};

#ifdef HAVE_HWSERIAL0
RPC_HARDWARE_SERIAL_UART_SLAVE()
#endif

#ifdef HAVE_HWSERIAL1
RPC_HARDWARE_SERIAL_UART_SLAVE(1)
#endif

#ifdef HAVE_HWSERIAL2
RPC_HARDWARE_SERIAL_UART_SLAVE(2)
#endif

#ifdef HAVE_HWSERIAL3
RPC_HARDWARE_SERIAL_UART_SLAVE(3)
#endif

#ifdef HAVE_CDCSERIAL
RPC_HARDWARE_SERIAL_UART_SLAVE()
#endif

class rpc_software_serial_uart_master : public rpc_master
{
public:
    rpc_software_serial_uart_master(uint8_t *buff, size_t buff_len,
                                    unsigned long rx_pin=2, unsigned long tx_pin=3, unsigned long baudrate=19200);
    ~rpc_software_serial_uart_master() { }
    virtual void _flush() override;
    virtual bool get_bytes(uint8_t *buff, size_t size, unsigned long timeout) override;
    virtual bool put_bytes(uint8_t *data, size_t size, unsigned long timeout) override;
private:
    SoftwareSerial __serial;
    rpc_software_serial_uart_master(const rpc_software_serial_uart_master &);   
};

class rpc_software_serial_uart_slave : public rpc_slave
{
public:
    rpc_software_serial_uart_slave(uint8_t *buff, size_t buff_len,
                                   rpc_callback_entry_t *callback_dict, size_t callback_dict_len,
                                   unsigned long rx_pin=2, unsigned long tx_pin=3, unsigned long baudrate=19200);
    ~rpc_software_serial_uart_slave() { }
    virtual void _flush() override;
    virtual bool get_bytes(uint8_t *buff, size_t size, unsigned long timeout) override;
    virtual bool put_bytes(uint8_t *data, size_t size, unsigned long timeout) override;
private:
    SoftwareSerial __serial;
    rpc_software_serial_uart_slave(const rpc_software_serial_uart_slave &);   
};

} // namespace openmv

#endif // __OPENMVRPC__
