// Remote Control - As The Controller Device
//
// This script configures your Arduino to remotely control an OpenMV Cam using the RPC
// library.
//
// This script is designed to pair with "popular_features_as_the_remote_device.py" running
// on the OpenMV Cam. The script is in OpenMV IDE under Files -> Examples -> Remote Control. 

#include <openmvrpc.h>

// The RPC library above provides mutliple classes for controlling an OpenMV Cam over
// CAN, I2C, SPI, or Serial (UART).

// We need to define a scratch buffer for holding messages. The maximum amount of data
// you may pass in any on direction is limited to the size of this buffer.

openmv::rpc_scratch_buffer<256> scratch_buffer; // All RPC objects share this buffer.

///////////////////////////////////////////////////////////////
// Choose the interface you wish to control an OpenMV Cam over.
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

// Uncomment the below line to setup your Arduino for controlling over SPI.
//
// * cs_pin - Slave Select Pin.
// * freq - SPI Bus Clock Frequency.
// * spi_mode - See (https://www.arduino.cc/en/reference/SPI)
//
// NOTE: Master and slave settings much match. Connect CS, SCLK, MOSI, MISO to CS, SCLK, MOSI, MISO.
//       Finally, both devices must share a common ground.
//
// openmv::rpc_spi_master interface(10, 1000000, SPI_MODE2);

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
// openmv::rpc_hardware_serial1_uart_master interface(115200);

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

void setup() {

    // For MCP2515 CAN we might need to change the default CAN settings for the Arduino Uno.
    //
    // CAN.setPins(9, 2); // CS & INT
    // CAN.setClockFrequency(16E6); // 16 MHz

    // Startup the RPC interface and a debug channel.
    interface.begin();
    Serial.begin(115200);
}

//////////////////////////////////////////////////////////////
// Call Back Handlers
//////////////////////////////////////////////////////////////

void exe_face_detection()
{
    struct { uint16_t x, y, w, h; } face_detection_result;
    if (interface.call_no_args(F("face_detection"), &face_detection_result, sizeof(face_detection_result))) {
        Serial.print(F("Largest Face Detected [x="));
        Serial.print(face_detection_result.x);
        Serial.print(F(", y="));
        Serial.print(face_detection_result.y);
        Serial.print(F(", w="));
        Serial.print(face_detection_result.w);
        Serial.print(F(", h="));
        Serial.print(face_detection_result.h);
        Serial.println(F("]"));
    }
}

void exe_person_detection()
{
    char buff[32 + 1] = {}; // null terminator
    if (interface.call_no_args(F("person_detection"), buff, sizeof(buff) - 1)) {
        Serial.println(buff);
    }
}

void exe_qrcode_detection()
{
    char buff[128 + 1] = {}; // null terminator
    if (interface.call_no_args(F("qrcode_detection"), buff, sizeof(buff) - 1)) {
        Serial.println(buff);
    }
}

void exe_apriltag_detection()
{
    struct { uint16_t cx, cy, id, rot; } apriltag_detection_result;
    if (interface.call_no_args(F("apriltag_detection"), &apriltag_detection_result, sizeof(apriltag_detection_result))) {
        Serial.print(F("Largest Tag Detected [cx="));
        Serial.print(apriltag_detection_result.cx);
        Serial.print(F(", cy="));
        Serial.print(apriltag_detection_result.cy);
        Serial.print(F(", id="));
        Serial.print(apriltag_detection_result.id);
        Serial.print(F(", rot="));
        Serial.print(apriltag_detection_result.rot);
        Serial.println(F("]"));
    }
}

void exe_datamatrix_detection()
{
    char buff[128 + 1] = {}; // null terminator
    if (interface.call_no_args(F("datamatrix_detection"), buff, sizeof(buff) - 1)) {
        Serial.println(buff);
    }
}

void exe_barcode_detection()
{
    char buff[128 + 1] = {}; // null terminator
    if (interface.call_no_args(F("barcode_detection"), buff, sizeof(buff) - 1)) {
        Serial.println(buff);
    }
}

void exe_color_detection()
{
    int8_t color_thresholds[6] = {30, 100, 15, 127, 15, 127}; // generic_red_thresholds
    // int8_t color_thresholds[6] = {30, 100, -64, -8, -32, 32}; // generic_green_thresholds
    // int8_t color_thresholds[6] = {0, 30, 0, 64, -128, 0}; // generic_blue_thresholds
    struct { uint16_t cx, cy; } color_detection_result;
    if (interface.call(F("color_detection"), color_thresholds, sizeof(color_thresholds), &color_detection_result, sizeof(color_detection_result))) {
        Serial.print(F("Largest Color Detected [cx="));
        Serial.print(color_detection_result.cx);
        Serial.print(F(", cy="));
        Serial.print(color_detection_result.cy);
        Serial.println(F("]"));
    }
}

// Execute remote functions in a loop. Please choose and uncomment one remote function below.
// Executing multiple at a time may run slowly if the camera needs to change camera modes
// per execution.

void loop() {
    exe_face_detection(); // Face should be about 2ft away.
    // exe_person_detection();
    // exe_qrcode_detection(); // Place the QRCode about 2ft away.
    // exe_apriltag_detection();
    // exe_datamatrix_detection(); // Place the Datamatrix about 2ft away.
    // exe_barcode_detection(); // Place the Barcode about 2ft away.
    // exe_color_detection();
}
