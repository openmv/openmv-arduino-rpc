//
// Sample RPC slave sketch
//
#include <ArduinoRPC.h>

//
// Command constants for remote Arduino functions
// For Arduino slave devices, start with 0x1000 to not conflict
// with constants defined for the OpenMV camera boards
//
const uint32_t PIN_MODE_CALL_BACK_INDEX = 0x1000;
const uint32_t DIGITAL_READ_CALL_BACK_INDEX = 0x1010;
const uint32_t DIGITAL_WRITE_CALL_BACK_INDEX = 0x1011;
const uint32_t ANALOG_READ_CALL_BACK_INDEX = 0x1020;
const uint32_t ANALOG_WRITE_CALL_BACK_INDEX = 0x1021;
const uint32_t I2C_READ_CALL_BACK_INDEX = 0x1030;
const uint32_t I2C_WRITE_CALL_BACK_INDEX = 0x1031;
const uint32_t SPI_READ_CALL_BACK_INDEX = 0x1040;
const uint32_t SPI_WRITE_CALL_BACK_INDEX = 0x1041;
const uint32_t SERIAL_READ_CALL_BACK_INDEX = 0x1050;
const uint32_t SERIAL_WRITE_CALL_BACK_INDEX = 0x1051;

rpc_i2c_slave rpc(I2C_ADDR, 400000L);
//
// Write to a GPIO pin
// The data consists of a pair of uin8_t values
// representing the pin number and state
//
uint32_t digitalWriteCallback(uint32_t event, uint8_t *data, uint32_t data_len)
{
  (void)event;
  (void)data_len;
  digitalWrite(data[0], data[1]);
  return 0;
} /* digitalWriteCallback() */

//
// Set the mode of one or more GPIO pins
// The data consists of pairs of pin numbers and modes
//
uint32_t pinModeCallback(uint32_t event, uint8_t *data, uint32_t data_len)
{
uint32_t i;

  (void)event;
  // The length divided by 2 is the number of pins defined
  for (i=0; i<data_len; i+=2) {
    pinMode(data[i], data[i+1]);
  }
  return 0;
} /* pinModeCallback() */

void setup()
{
  rpc.register_callback(PIN_MODE_CALL_BACK_INDEX, pinModeCallback);
  rpc.register_callback(DIGITAL_WRITE_CALL_BACK_INDEX, digitalWriteCallback);
} /* setup() */

void loop()
{
  rpc.loop();
} /* loop() */
