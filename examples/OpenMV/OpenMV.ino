//
// Sample RPC master
// communicates with OpenMV camera as slave device
//
#include <ArduinoRPC.h>

const uint32_t FACE_DETECTION_CALL_BACK_INDEX = 10;
const uint32_t PERSON_DETECTION_CALL_BACK_INDEX = 20;
const uint32_t QRCODE_DETECTION_CALL_BACK_INDEX = 30;
const uint32_t QRCODE_DETECTION_ALL_CALL_BACK_INDEX = 31;
const uint32_t APRILTAG_DETECTION_CALL_BACK_INDEX = 40;
const uint32_t APRILTAG_DETECTION_ALL_CALL_BACK_INDEX = 41;
const uint32_t DATAMATRIX_DETECTION_CALL_BACK_INDEX = 50;
const uint32_t DATAMATRIX_DETECTION_ALL_CALL_BACK_INDEX = 51;
const uint32_t BARCODE_DETECTION_CALL_BACK_INDEX = 60;
const uint32_t BARCODE_DETECTION_ALL_CALL_BACK_INDEX = 61;
const uint32_t COLOR_DETECTION_CALL_BACK_INDEX = 70;
const uint32_t JPEG_SNAPSHOT_CALL_BACK_INDEX = 80;
const uint32_t LARGE_IMAGE_SNAPSHOT_CALL_BACK_INDEX = 90;
const uint32_t LARGE_IMAGE_READ_CALL_BACK_INDEX = 91;

const int8_t  thresholds[] = {30, 100, 15, 127, 15, 127}; // generic red thresholds
// const int8_t thresholds[] = {30, 100, -64, -8, -32, 32}; // generic green thresholds
// const int8_t thresholds[] = {0, 30, 0, 64, -128, 0}; // generic blue thresholds

rpc_i2c_master rpc(I2C_ADDR, 400000L);

void setup()
{
  Serial.begin(115200);
  
} /* setup() */

void loop()
{
uint8_t ucInBuf[32];
uint32_t u32InSize;
uint16_t CX, CY;

  while (1)
  {
    if (rpc.call(COLOR_DETECTION_CALL_BACK_INDEX, (uint8_t *)thresholds, sizeof(thresholds), ucInBuf, &u32InSize, 1000, 1000)) {
      // color detection returns 2 unsigned shorts
      CX = *(uint16_t *)&ucInBuf[0];
      CY = *(uint16_t *)&ucInBuf[2];
      Serial.print("color dot size: ");
      Serial.print(CX, DEC);
      Serial.print("x");
      Serial.println(CY, DEC);
    }
    delay(1000); // check once a second  
  } // while looping forever
} /* loop() */
