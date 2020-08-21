// Streaming Remote Control - As The Controller Device
//
// This script configures your Arduino to remotely control another Arduino.
//
// In particular, this script shows off transfering large chunks of data at high speeds from one
// Arduino to another demonstrating the power of the OpenMV RPC Library.
//
// This script is designed to pair with "arduino_to_arduino_streaming_comms_as_the_remote_device.ino"
// example sketch included with this library.

#include <openmvrpc.h>

// The RPC library above provides mutliple classes for controlling an Arduino over
// CAN, I2C, SPI, or Serial (UART).

// We need to define a scratch buffer for holding messages. The maximum amount of data
// you may pass in any on direction is limited to the size of this buffer.

openmv::rpc_scratch_buffer<256> scratch_buffer; // All RPC objects share this buffer.

///////////////////////////////////////////////////////////////
// Choose the interface you wish to control an Arduino over.
///////////////////////////////////////////////////////////////

// Uncomment the below line to setup your Arduino for controlling over CAN.
//
// * message_id - CAN message to use for data transport on the can bus (11-bit).
// * bit_rate - CAN bit rate.
//
// NOTE: Master and slave message ids and can bit rates must match. Connect master can high to slave
//       can high and master can low to slave can lo. The can bus must be terminated with 120 ohms.
//
// openmv::rpc_can_master interface(0x7FF, 250E3);

// Uncomment the below line to setup your Arduino for controlling over I2C.
//
// * slave_addr - I2C address.
// * rate - I2C Bus Clock Frequency.
//
// NOTE: Master and slave addresses must match. Connect master scl to slave scl and master sda
//       to slave sda. You must use external pull ups. Finally, both devices must share a ground.
//
// openmv::rpc_i2c_master interface(0x12, 100000);

// Uncomment the below line to setup your Arduino for controlling over a hardware UART.
//
// * baudrate - Serial Baudrate.
//
// NOTE: Master and slave baud rates must match. Connect master tx to slave rx and master rx to
//       slave tx. Finally, both devices must share a common ground.
//
// WARNING: The program and debug port for your Arduino may be "Serial". If so, you cannot use
//          "Serial" to connect to an OpenMV Cam without blocking your Arduino's ability to
//          be programmed and use print/println.
//
// openmv::rpc_hardware_serial_uart_master -> Serial
// openmv::rpc_hardware_serial1_uart_master -> Serial1
// openmv::rpc_hardware_serial2_uart_master -> Serial2
// openmv::rpc_hardware_serial3_uart_master -> Serial3
//
// openmv::rpc_hardware_serial3_uart_master interface(115200);

// Uncomment the below line to setup your Arduino for controlling over a software UART.
//
// * rx_pin - RX Pin (See the reference guide about what pins can be used)
// * tx_pin - TX Pin (see the reference guide about what pins can be used)
// * baudrate - Serial Baudrate (See the reference guide https://www.arduino.cc/en/Reference/SoftwareSerial)
//
// NOTE: Master and slave baud rates must match. Connect master tx to slave rx and master rx to
//       slave tx. Finally, both devices must share a common ground.
//
openmv::rpc_software_serial_uart_master interface(2, 3, 19200);

///////////////////////////////////////////////////////////////
// Streaming Code
///////////////////////////////////////////////////////////////

void setup() {

    // For MCP2515 CAN we might need to change the default CAN settings for the Arduino Uno.
    //
    // CAN.setPins(9, 2); // CS & INT
    // CAN.setClockFrequency(16E6); // 16 MHz

    // Startup the RPC interface and a debug channel.
    interface.begin();
    Serial.begin(115200);
}

// This will get called repeatedly while streaming is setup.
bool stream_reader_cb(uint8_t *in_data, uint32_t in_data_len) {
    (void) in_data_len; // unused
    Serial.print((char *) in_data); // in_data is a null terminated string
    return true; // always continue streaming
}

void loop() {

    // Need to track the state of streaming (this gets initialized to false only once)
    static bool streaming = false;

    if (!streaming) {
        streaming = interface.call_no_args(F("setup_streaming_cb"), NULL, 0, false) && interface.stream_reader_setup(1);
    }

    if (streaming) {
        streaming = interface.stream_reader_loop(stream_reader_cb);
    }
}
