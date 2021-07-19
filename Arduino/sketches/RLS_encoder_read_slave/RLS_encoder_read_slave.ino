#include "RLS_Encoder.h"
#include <i2c_t3.h>

#include <EEPROMex.h>
const int maxAllowedWrites = 20;
const int memBase          = 120;
int addressInt;


//=====[ VARIABLES ]============================================================
RLS_Encoder enc;
float value = 0, value2 = 0;
uint8_t slave_1 = 0x64;

void setup() {
  Serial.begin(115200); while (!Serial);
  //addressInt = EEPROM.getAddress(sizeof(int));

  // ------ write address only once
  //EEPROM.setMemPool(memBase, EEPROMSizeUno);
  //EEPROM.setMaxAllowedWrites(maxAllowedWrites);
  //EEPROM.writeInt(addressInt, 0x6c);
  // ------------------
  //slave_1 = EEPROM.readInt(addressInt);
  //Serial.print("add:");
  //Serial.println(slave_1);

  enc.begin(); delay(5);
  value = 0;
  Wire.begin(I2C_SLAVE, slave_1, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
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


void requestEvent() {
  Serial2.flush();
  // Read first=this encoder position
  /*
    while (Serial2.available()){
    //if (Serial2.read()=='1'){
      value = enc.get_pos();
      break;
    //}
    }
  */
  value = 8.2;
  byte *data = (byte *)&value;
  Wire.write(data[0]);
  Wire.write(data[1]);
  Wire.write(data[2]);
  Wire.write(data[3]);
}
