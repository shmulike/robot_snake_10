#include "RLS_Encoder.h"
#include <i2c_t3.h>

//=====[ VARIABLES ]============================================================
RLS_Encoder enc;
float value = 0, value2=0;
uint8_t slave_1 = 8;

void setup() {
  // put your setup code here, to run once:
  enc.begin(); delay(5);
  value = 0;
  Wire.begin(I2C_SLAVE, 8, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  Wire.onRequest(requestEvent);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
}

/*
void requestEvent() {
  //value = enc.get_pos();
  
  value += 0.01;

  byte *data = (byte *)&value;
  Wire.write(data[0]);
  Wire.write(data[1]);
  Wire.write(data[2]);
  Wire.write(data[3]);
}
*/


void requestEvent(){      
  Serial2.flush();
  // Read first=this encoder position
  while (Serial2.available()){
    //if (Serial2.read()=='1'){
      value = enc.get_pos();
      break;
    //}
    }
  byte *data = (byte *)&value;
  Wire.write(data[0]);
  Wire.write(data[1]);
  Wire.write(data[2]);
  Wire.write(data[3]);
}
