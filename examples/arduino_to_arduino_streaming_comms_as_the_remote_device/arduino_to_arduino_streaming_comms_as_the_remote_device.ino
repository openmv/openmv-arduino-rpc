
// Streaming Remote Control - As The Remote Device
//
// This script configures your Arduino to be remotely controlled by another Arduino.
//
// In particular, this script shows off transfering large chunks of data at high speeds from one
// Arduino to another demonstrating the power of the OpenMV RPC Library.
//
// This script is designed to pair with "arduino_to_arduino_streaming_comms_as_the_controller_device.ino"
// example sketch included with this library.

#include <openmvrpc.h>

// The RPC library above provides mutliple classes for being cotnrolled over
// CAN, I2C, SPI, or Serial (UART).

// We need to define a scratch buffer for holding messages. The maximum amount of data
// you may pass in any on direction is limited to the size of this buffer.

openmv::rpc_scratch_buffer<256> scratch_buffer; // All RPC objects share this buffer.

// The interface library executes call backs on this device which have to be registered
// before they can be called. To avoid dyanmic memory allocation we have to create a buffer
// with the maximum number of call backs we plan to support to hold the registrations.
//
// Note that callback registrations only work on the rpc interface that registered them.

openmv::rpc_callback_buffer<8> callback_buffer; // All RPC objects share this buffer.

///////////////////////////////////////////////////////////////
// Choose the interface you wish to be controlled over.
///////////////////////////////////////////////////////////////

// Uncomment the below line to setup for be controlled over CAN.
//
// * message_id - CAN message to use for data transport on the can bus (11-bit).
// * bit_rate - CAN bit rate.
//
// NOTE: Master and slave message ids and can bit rates must match. Connect master can high to slave
//       can high and master can low to slave can lo. The can bus must be terminated with 120 ohms.
//
// openmv::rpc_can_slave interface(0x7FF, 250E3);

// Uncomment the below line to setup for be controlled over I2C.
//
// * slave_addr - I2C address.
// * rate - I2C Bus Clock Frequency.
//
// NOTE: Master and slave addresses must match. Connect master scl to slave scl and master sda
//       to slave sda. You must use external pull ups. Finally, both devices must share a ground.
//
// openmv::rpc_i2c_slave interface(0x12);

// Uncomment the below line to setup for be controlled over a hardware UART.
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
// openmv::rpc_hardware_serial_uart_slave -> Serial
// openmv::rpc_hardware_serial1_uart_slave -> Serial1
// openmv::rpc_hardware_serial2_uart_slave -> Serial2
// openmv::rpc_hardware_serial3_uart_slave -> Serial3
//
// openmv::rpc_hardware_serial_uart_slave interface(115200);

// Uncomment the below line to setup for be controlled over a software UART.
//
// * rx_pin - RX Pin (See the reference guide about what pins can be used)
// * tx_pin - TX Pin (see the reference guide about what pins can be used)
// * baudrate - Serial Baudrate (See the reference guide https://www.arduino.cc/en/Reference/SoftwareSerial)
//
// NOTE: Master and slave baud rates must match. Connect master tx to slave rx and master rx to
//       slave tx. Finally, both devices must share a common ground.
//
openmv::rpc_software_serial_uart_slave interface(2, 3, 19200);

///////////////////////////////////////////////////////////////
// Streaming Code
///////////////////////////////////////////////////////////////

enum { NOT_STREAMING, SETUP_STREAMING, STREAMING } streaming_state = NOT_STREAMING;

bool stream_writer_cb(uint8_t **out_data, uint32_t *out_data_len) {
    static char sentence[] = "How much wood would a woodchuck chuck if a woodchuck could chuck wood?\r\n";
    *out_data = (uint8_t *) sentence;
    *out_data_len = strlen(sentence) + 1; // plus null terminator
    return true; // always continue streaming
}

// The controller device will execute this function to kick off streaming.
void setup_streaming_cb() {
    streaming_state = SETUP_STREAMING;
}

// NOTE: The string name can be anything below. It just needs to match between the master/slave devices.

void setup() {

    // For MCP2515 CAN we might need to change the default CAN settings for the Arduino Uno.
    //
    // CAN.setPins(9, 2); // CS & INT
    // CAN.setClockFrequency(16E6); // 16 MHz

    interface.register_callback(F("setup_streaming_cb"), setup_streaming_cb);

    // Startup the RPC interface and a debug channel.
    interface.begin();
    Serial.begin(115200);
}

// Once all call backs have been registered we can start
// processing remote events.

void loop() {

    // It's important the loop() returns constantly for your Arduino to work properly.
    // The state machine below keeps our state while returning constantly.

    if (streaming_state == NOT_STREAMING) {
        interface.loop();
    } else if (streaming_state == SETUP_STREAMING) {
        streaming_state = interface.stream_writer_setup() ? STREAMING : NOT_STREAMING;
    } else if (streaming_state == STREAMING) {
        streaming_state = interface.stream_writer_loop(stream_writer_cb) ? STREAMING : NOT_STREAMING;
    }
}
