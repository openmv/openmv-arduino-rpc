// Remote Control - As The Controller Device
//
// This script configures your Arduino to remotely control an OpenMV Cam using the RPC
// library.
//
// This script is designed to pair with "image_transfer_jpg_as_the_remote_device.py" running
// on the OpenMV Cam. The script is in OpenMV IDE under Files -> Examples -> Remote Control. 

#include <SPI.h>
#include <SD.h>
#include <openmvrpc.h>

const int SD_CARD_CHIP_SELECT_PIN = 4;

// The RPC library above provides mutliple classes for controlling an OpenMV Cam over
// CAN, I2C, SPI, or Serial (UART).

// We need to define a scratch buffer for holding messages. The maximum amount of data
// you may pass in any on direction is limited to the size of this buffer.

// PLEASE NOTE THAT THE OPENMV CAM OUTPUTS BURST DATA AT A VERY HIGH RATE BACK TO BACK
// THAT SOME ARDUINOS MAY NOT BE ABLE TO HANDLE. ONLY MAKE THE BUFFER SIZE LARGER FOR
// INTERFACES WHICH CAN HANDLE A HIGH BURST RATE.

openmv::rpc_scratch_buffer<32> scratch_buffer; // All RPC objects share this buffer.

///////////////////////////////////////////////////////////////
// Choose the interface you wish to control an OpenMV Cam over.
///////////////////////////////////////////////////////////////

// CAN AND SPI INTERFACES CANNOT BE USED BECAUSE THE SD CARD LIBRARY
// USES THE SPI BUS AND DOES NOT SHARE SPI BUS ACCESS NICELY. 

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

    // Initialize the SD Card

    while (!Serial); // wait for Serial Monitor to connect. Needed for native USB port boards only:

    Serial.println(F("Initializing SD card..."));

    if (!SD.begin(SD_CARD_CHIP_SELECT_PIN)) {
        Serial.println(F("initialization failed. Things to check:"));
        Serial.println(F("1. is a card inserted?"));
        Serial.println(F("2. is your wiring correct?"));
        Serial.println(F("3. did you change the chipSelect pin to match your shield or module?"));
        Serial.println(F("Note: press reset or reopen this serial monitor after fixing your issue!"));
        while (true);
    }

    Serial.println(F("Initialization done."));
}

void loop() {
    // The script running on the OpenMV Cam will evaluate the string below to set the pixformat and framesize.
    const char pixformat_and_framesize[] = "sensor.RGB565,sensor.QQQVGA";
    uint32_t jpeg_size;

    // jpeg_image_snapshot will take a jpeg picture, store it in memory on the OpenMV Cam, and then return the
    // jpg image size in bytes for reading by the Arduino.
    Serial.println(F("Taking a pic..."));
    if (interface.call(F("jpeg_image_snapshot"),
                       pixformat_and_framesize, sizeof(pixformat_and_framesize) - 1, // Do not send NULL terminator
                       &jpeg_size, sizeof(jpeg_size))) {
        Serial.println(F("Success"));

        // We need to generate a filename now to save the jpg image data too. The below code reformats the
        // 5-digit file name string based on a counter that will increment each time we save an image.
        static uint16_t file_counter = 0;
        char filename[] = "00000.JPG";
        filename[0] += ((file_counter / 10000) % 10);
        filename[1] += ((file_counter / 1000) % 10);
        filename[2] += ((file_counter / 100) % 10);
        filename[3] += ((file_counter / 10) % 10);
        filename[4] += ((file_counter / 1) % 10);

        // Try to create the filename. If a file already exists with the same name it will be deleted.
        Serial.println(F("Creating jpg file..."));
        Serial.println(filename);
        File jpg_file = SD.open(filename, O_WRITE | O_CREAT | O_TRUNC);

        if (jpg_file) {
            // jpeg_image_read takes two arguments, offset and size.
            // We can easily pass the two arguments as a struct.
            struct {
                uint32_t offset;
                uint32_t size;
            } arg;

            // To read the file we have to read a chunk at a time which is limited to our max buffer size.
            arg.offset = 0;
            arg.size = scratch_buffer.buffer_size();

            // Now read one chunk of the file after another in order.
            while (true) {
                // interface.call_no_copy() will return a pointer to data inside the scratch buffer that has
                // been transferred. This saves us from needing another data buffer in RAM.
                char *jpg_data;
                size_t jpg_data_len;

                // Transfer the bytes. jpg_data and jpg_data_len on success will point to the data transferred.
                Serial.print(F("Reading bytes "));
                Serial.print((arg.offset * 100) / jpeg_size);
                Serial.println(F("%"));
                if (interface.call_no_copy(F("jpeg_image_read"), &arg, sizeof(arg), &jpg_data, &jpg_data_len)) {
                    Serial.println(F("Writing bytes..."));

                    // Finally, write the bytes to the SD card.
                    if (jpg_file.write(jpg_data, jpg_data_len) == jpg_data_len) {
                        // Once the data has been written increment our offset in the jpeg file we are reading. 
                        arg.offset += jpg_data_len;

                        // Close the file and go to the next one once finished.
                        if (arg.offset >= jpeg_size) {
                            Serial.println(F("File Saved"));
                            jpg_file.close();
                            file_counter += 1;
                            break;
                        }
                    } else {
                        Serial.println(F("Failed!"));
                        jpg_file.close();
                        break;     
                    }
                } else {
                    Serial.println(F("Failed!"));
                    jpg_file.close();
                    break;
                }
            }
        } else {
            Serial.println(F("Failed!"));
        }
    } else {
        Serial.println(F("Failed!"));
    }
}
