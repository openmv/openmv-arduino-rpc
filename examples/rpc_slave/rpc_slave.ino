//
// Sample RPC slave sketch
//
#include <ArduinoRPC.h>

//
// Command constants for remote Arduino functions
// For Arduino slave devices, start with 0x1000 to not conflict
// with constants defined for the OpenMV camera boards
//
const uint16_t PIN_MODE_CALL_BACK_INDEX = 0x1000;
const uint16_t DIGITAL_READ_CALL_BACK_INDEX = 0x1010;
const uint16_t DIGITAL_WRITE_CALL_BACK_INDEX = 0x1011;
const uint16_t ANALOG_READ_CALL_BACK_INDEX = 0x1020;
const uint16_t ANALOG_WRITE_CALL_BACK_INDEX = 0x1021;
const uint16_t I2C_READ_CALL_BACK_INDEX = 0x1030;
const uint16_t I2C_WRITE_CALL_BACK_INDEX = 0x1031;
const uint16_t SPI_READ_CALL_BACK_INDEX = 0x1040;
const uint16_t SPI_WRITE_CALL_BACK_INDEX = 0x1041;
const uint16_t SERIAL_READ_CALL_BACK_INDEX = 0x1050;
const uint16_t SERIAL_WRITE_CALL_BACK_INDEX = 0x1051;

RPC rpc;
//
// Write to a GPIO pin
// The data consists of a pair of uin8_t values
// representing the pin number and state
//
uint32_t digitalWriteCallback(int iEvent, uint8_t *data, uint32_t data_len)
{
  digitalWrite(data[0], data[1]);
  return 0;
} /* digitalWriteCallback() */

//
// Set the mode of one or more GPIO pins
// The data consists of pairs of pin numbers and modes
//
uint32_t pinModeCallback(int iEvent, uint8_t *data, uint32_t data_len)
{
int i;
  // The length divided by 2 is the number of pins defined
  for (i=0; i<data_len; i+=2) {
    pinMode(data[i], data[i+1]);
  }
  return 0;
} /* pinModeCallback() */

void setup()
{
  rpc.begin(RPC_I2C, 400000L);
  rpc.register_callback(PIN_MODE_CALL_BACK_INDEX, pinModeCallback);
  rpc.register_callback(DIGITAL_WRITE_CALL_BACK_INDEX, digitalWriteCallback);
} /* setup() */

void loop()
{
uint32_t len, new_len, cmd;
uint8_t ucTemp[32];
RPC_CALLBACK pfnCB;

  while (1) {
    cmd = rpc.get_command(ucTemp, &len);
    if (cmd) {
      pfnCB = rpc.find_callback(cmd);
      if (pfnCB) { // we registered a callback for this command
        new_len = (*pfnCB)(cmd, ucTemp, len);
        rpc.put_result(ucTemp, new_len);
      } else { // no callback to respond
        rpc.put_result(ucTemp, 0); // DEBUG - need a NACK
      }
  }
} /* loop() */

