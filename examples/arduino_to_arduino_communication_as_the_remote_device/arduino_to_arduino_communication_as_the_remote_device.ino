
// Remote Control - As The Remote Device
//
// This script configures your Arduino to be remotely controlled by another Arduino.
//
// This script is designed to pair with "arduino_to_arduino_communication_as_the_controller_device.ino"
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
// openmv::rpc_hardware_serial3_uart_slave interface(115200);

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

//////////////////////////////////////////////////////////////
// Call Backs
//////////////////////////////////////////////////////////////

size_t digital_read_example(void *out_data) {
    // Get what we want to return into a variable.
    uint8_t state = digitalRead(4);

    // Move that variable into a transmit buffer.
    memcpy(out_data, &state, sizeof(state));

    // Return how much we will send.
    return sizeof(state);
}

size_t analog_read_example(void *out_data) {
    // Get what we want to return into a variable.
    uint16_t state = analogRead(A0);

    // Move that variable into a transmit buffer.
    memcpy(out_data, &state, sizeof(state));

    // Return how much we will send.
    return sizeof(state);
}

void digital_write_example(void *in_data, size_t in_data_len) {
    // Create the primitive or complex data type on the stack.
    uint8_t state;

    // Check that we received the amount of data we expected.
    if (in_data_len != sizeof(state)) return;

    // Copy what we received into our data type container.
    memcpy(&state, in_data, sizeof(state));

    // Use it now.
    digitalWrite(5, state);
}

void analog_write_example(void *in_data, size_t in_data_len) {
    // Create the primitive or complex data type on the stack.
    uint8_t state;

    // Check that we received the amount of data we expected.
    if (in_data_len != sizeof(state)) return;

    // Copy what we received into our data type container.
    memcpy(&state, in_data, sizeof(state));

    // Use it now.
    analogWrite(A1, state);
}

void serial_print_example(void *in_data, size_t in_data_len) {
    // Create the string on the stack (extra byte for the null terminator).
    char buff[in_data_len + 1]; memset(buff, 0, in_data_len + 1);

    // Copy what we received into our data type container.
    memcpy(buff, in_data, in_data_len);

    // Use it now.
    Serial.println(buff);
}

// NOTE: The string name can be anything below. It just needs to match between the master/slave devices.

void setup() {

    // For MCP2515 CAN we might need to change the default CAN settings for the Arduino Uno.
    //
    // CAN.setPins(9, 2); // CS & INT
    // CAN.setClockFrequency(16E6); // 16 MHz

    interface.register_callback(F("digital_read"), digital_read_example);
    interface.register_callback(F("analog_read"), analog_read_example);
    interface.register_callback(F("digital_write"), digital_write_example);
    interface.register_callback(F("analog_write"), analog_write_example);
    interface.register_callback(F("serial_print"), serial_print_example);

    // Startup the RPC interface and a debug channel.
    interface.begin();
    Serial.begin(115200);
}

// Once all call backs have been registered we can start
// processing remote events.

void loop() {
    interface.loop();
}
