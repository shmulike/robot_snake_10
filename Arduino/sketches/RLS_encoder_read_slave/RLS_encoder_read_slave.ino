//==============================================================================
//Shmulik Edelman
//shmulike@post.bgu.ac.il
//==============================================================================

//=====[ INCULDE ]==============================================================
#include "RLS_Encoder.h"
#include <i2c_t3.h>

//=====[ Constants ]========================================
//#define slave_1 8
//=====[ VARIABLES ]============================================================
RLS_Encoder enc;
float value = 0;
uint8_t slave_1 = 0x66;

//=====[ Function declaraion ]========================================
void requestEvent();

//=====[ SETUP ]================================================================
void setup() {
  //Serial.begin(115200); while(!Serial);
  enc.begin(); delay(5);
  enc.set_read(); delay(5);
  Wire.begin(I2C_SLAVE, slave_1, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  delay(10);
  Wire.onRequest(requestEvent);
  //value = 0;
}

//=====[ LOOP ]=================================================================
void loop() {
  delay(2);
}
//==============================================================================

void requestEvent() {
  float value = enc.get_pos();
  byte *data = (byte *)&value;
  Wire.write(data, 4);
  /*Wire.write(data[0]);
    Wire.write(data[1]);
    Wire.write(data[2]);
    Wire.write(data[3]);*/
}
