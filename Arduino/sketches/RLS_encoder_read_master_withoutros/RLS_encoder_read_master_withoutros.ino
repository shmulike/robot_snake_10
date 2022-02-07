//==============================================================================
//Shmulik Edelman
//shmulike@post.bgu.ac.il
//==============================================================================

//=====[ INCULDE ]==============================================================
#include "RLS_Encoder.h"
#include <i2c_t3.h>
#define PRINT 1
#define N_links 10
#define N_links_read 10

//=====[ Constants ]========================================


//=====[ VARIABLES ]============================================================
RLS_Encoder enc;

byte data[4] = {0};
//int value = 0;
float arr[N_links] = {0.0}, val_previus = 0.0;

int count = 0;
uint8_t slave_add[N_links - 1] = {0x64, 0x65, 0x66, 0x67, 0x68, 0x69 , 0x6a, 0x6b, 0x6c};
float joint_offset[N_links] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


union floatToBytes {
  char buffer[4];
  float encReading;
} converter;

float cou=0;
int loop_counter=0;             //holds the count for every loop pass
long loop_timer_now;          //holds the current millis
long previous_millis;         //holds the previous millis
float loop_time;              //holds difference (loop_timer_now - previous_millis) = total execution time
int loop_test_times = 100;  //Run loop 20000 times then calculate time


//=====[ FUNCTIONS ]================================================================

float wrapTo180(float val) {
  //return (val > 180)? val-360.0 : val;
  return val;
}
//=====[ SETUP ]================================================================
void setup() {
  Serial.begin(115200); while (!Serial);
  enc.begin();

  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  //Wire.setDefaultTimeout(50000); // 10ms default timeout
  Wire.setClock(1800000);
}

//=====[ LOOP ]=================================================================
void loop() {
  //Serial.println(x++);
  //Serial.print("line:\t");

  // Read first=this encoder position

  Serial2.flush();

  

  loop_counter++;
  if (loop_counter == 1000)
  { previous_millis = loop_timer_now; loop_timer_now = millis(); loop_counter = 0;
    loop_time = ((loop_timer_now - previous_millis)/1000.0)/1000; 
    loop_time=1/loop_time;
    Serial.print(loop_time, 6);
    Serial.print("\n");
  }
  if (loop_counter==0){
    loop_timer_now = millis();
  }
  
  // Read others encoder position
  arr[0] = cou; //master read
  cou++;
  
  for (int i = 1; i < N_links_read; i++) {                 // Loop to run on every slave
    //Serial.print("S");
    Wire.requestFrom(slave_add[i - 1], sizeof(data)); // request slave data
    delay(1);
    if (Wire.getError()) {                            // if error in recieving data from slave
      Serial.print("--Wire Error(");                 // print the error
      Serial.print(slave_add[i]);
      Serial.print("):");
      Serial.print(Wire.getError());
      //      Serial.println();/
    }
    else {
      while (Wire.available()) {                                           // If no error than get the data from slave
        //Serial.print("A");

        /*
          Wire.read(data, Wire.available());
          for (int i=0; i<4; i++)                           // Read 4 bytes from slave and transfer it to float (4 bytes)
          u.b[i] = data[i];
          //arr[i] = u.fval - joint_offset[i];
        */
        
        for (int i = 0; i < 4; i++)                     // 4 bytes
        {
          converter.buffer[i] = Wire.read();
        }
        
        //Serial.print(converter.encReading);
        
        //arr[i] = wrapTo180(converter.encReading - joint_offset[i]);   // Convert angle to [-180,180]
        arr[i] = converter.encReading - joint_offset[i];
        //arr[i] = wrapTo180(converter.encReading);     // Convert angle to [-180,180]


      } //while
    } //else
    //Serial.print("H");
    // Wire.finish();

  } //for

/**
  // print Encoder values
  if (PRINT) {
//    Serial.print("val M"); //print master enc only
   // Serial.print(": ");
    Serial.print(arr[0], 3);
    Serial.print("\t");
    for (int i = 0; i < N_links_read-1; i++) {
      Serial.print("val ");
      Serial.print(slave_add[i], HEX);
      Serial.print(": ");
      Serial.print(arr[i + 1], 6);
      Serial.print("\t");
    }
    Serial.print("\n");
  }
**/
  


  delay(1);
}
//==============================================================================
