
#include "RLS_Encoder.h"

RLS_Encoder enc;
int read_val, junk;
float value = 0;

void print_menu();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial);

  enc.begin();
  
  print_menu();
}

void loop() {
  
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    // read the incoming byte:
    read_val = Serial.read();
    read_val -= 48;
    //Serial.print("commant: ");
    Serial.println(read_val);
    junk = Serial.read();
    
    switch(read_val){
    case 1:
      enc.reset();
    break;
    case 2:
    enc.set_read();
    break;
    case 3:
      enc.start_response();
    break;
    case 4:
      enc.get_status();
    break;
    case 5:
      Serial2.flush();
      value = enc.get_pos();
      Serial.println(value,3);
    break;
    case 6:
      enc.calibrate();
    default:
      Serial.println("wrong command");
    }
    
    print_menu();
  }
  
  delay(10);
  
  
}

void print_menu(){
  Serial.println("----------------------------");
  Serial.println("1 = Reset to factory settings");
  Serial.println("2 = Set read");
  Serial.println("3 = Start response");
  Serial.println("4 = Get status");
  Serial.println("5 = Read position");
  Serial.println("6 = Calibrate");
  Serial.print("--> ");
}
