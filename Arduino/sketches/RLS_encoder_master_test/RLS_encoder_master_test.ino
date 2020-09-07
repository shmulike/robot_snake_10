#include <i2c_t3.h>

uint8_t slave_1 = 0x64;
float val = 0;
union floatToBytes{
  char buffer[4];
  float fval;
  } converter;
  
byte data[4] = {0};
int count;

void setup()
{
    // Setup for Master mode, pins 18/19, external pullups, 400kHz, 200ms default timeout
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
    Wire.setDefaultTimeout(200000); // 200ms
    Serial.begin(115200);
    while(!Serial);
}

void loop()
{
  Serial.print("Reading from Slave: ");
  Wire.requestFrom(slave_1, 4); // Read from Slave (string len unknown, request full buffer)
  if(Wire.getError()){
    Serial.print("FAIL code: ");
    Serial.println(Wire.getError());
    }
    else{
      for (int i=0; i<4; i++)  
        converter.buffer[i] = Wire.read();
      Serial.println(converter.fval, 5);
      }
  delay(50);                       // Delay to space out tests
}
