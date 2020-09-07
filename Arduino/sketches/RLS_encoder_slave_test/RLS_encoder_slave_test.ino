
#include <i2c_t3.h>

// Function prototypes
void requestEvent(void);

float value = 0;
uint8_t slave_1 = 0x64;

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT); // LED

    Wire.begin(I2C_SLAVE, slave_1, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
    Wire.onRequest(requestEvent);
    Serial.begin(115200);
    while(!Serial);
    value = 0;
}

void loop()
{
  delay(50);
  Serial.println("waiting ..");
}

void requestEvent(void)
{
  Serial.println("Send !");
  //digitalWrite(LED_BUILTIN, HIGH); delay(200);
  //digitalWrite(LED_BUILTIN, LOW);
  
  value += 0.01;

  byte *data = (byte *)&value;
  Wire.write(data[0]);
  Wire.write(data[1]);
  Wire.write(data[2]);
  Wire.write(data[3]);
}
