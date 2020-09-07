//==========================================================
//Shmulik Edelman
//shmulike@post.bgu.ac.il
// This code reading form MULTI HX711 Load-cells and sending there value as ROS-message
//==========================================================

//=====[ INCULDE ]==========================================

#include "HX711-multi.h"
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

//=====[ Constants ]========================================
#define PRINT 0
#define N_joints 4
#define CLK 11      // clock pin to the load cell amp
#define DOUT1 A0    // data pin to the first lca
#define DOUT2 A1    // data pin to the second lca
#define DOUT3 A2    // data pin to the third lca
#define DOUT4 A3    // data pin to the third lca
#define DOUT5 A4    // data pin to the third lca
#define DOUT6 A5    // data pin to the third lca
#define DOUT7 A6    // data pin to the third lca
#define DOUT8 A7    // data pin to the third lca
#define BOOT_MESSAGE "MIT_ML_SCALE V0.8"
#define TARE_TIMEOUT_SECONDS 4
#define d_count 1000
//#define CHANNEL_COUNT sizeof(DOUTS)/sizeof(byte)

#define N_tensions 8
long count = 0;
unsigned long startTime, endTime;

//=====[ VARIABLES ]=========================================
ros::NodeHandle nh;
byte DOUTS_1[4] = {DOUT1, DOUT2, DOUT3, DOUT4};
byte DOUTS_2[4] = {DOUT5, DOUT6, DOUT7, DOUT8};

long int result[4];
float arr[8]={0};
HX711MULTI scales_1(4, DOUTS_1, CLK);
HX711MULTI scales_2(4, DOUTS_2, CLK);

std_msgs::Float32MultiArray tension_val;
ros::Publisher pub_tensions("/robot_snake_1/tension_val", &tension_val);

   double P[N_tensions][3] = {{0.00000000, 0.00002348, 1.47896687},
                              {0.00000000, 0.00002385, 2.88188982},
                              {0.00000000, 0.00002414, 1.27118260},
                              {0.00000000, 0.00002028,-4.84178993},
                              {0.00000000, 0.00002322, 3.38832415},
                              {0.00000000, 0.00002559, 4.95853685},
                              {0.00000000, 0.00002408, 6.59883064},
                              {0.00000000, 0.00002436, 1.13493471}};
                     
//=====[ SETUP ]=============================================
void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pub_tensions);
  tension_val.data_length = N_tensions;
  if (PRINT){
    Serial.begin(115200);
    while(!Serial);
    Serial.println("Serial Communication started");
  }
  startTime = millis();
  endTime = 0;
}

//=====[ LOOP ]==============================================
void loop() {
  Serial.print(count++);
  Serial.print(":");
  if (count%d_count==0){
    endTime = millis();
    //Serial.print(endTime-startTime);
    Serial.print(d_count*1000.0/(endTime-startTime));
    startTime=endTime;
  }
  Serial.print("\t");

  // Read raw data from all tension-sensors - SET 1
  scales_1.readRaw(result);                
  // ---------- Convert RAW data to Kg
  for (int i=0; i<(4); i++){
    arr[i] = P[i][0]*pow(result[i],2) + P[i][1]*result[i] + P[i][2];
    //Serial.print(result[k]);
    Serial.print(arr[i],3);
    Serial.print("\t");
  }


  // Read raw data from all tension-sensors - SET 2
  scales_2.readRaw(result);
  // ---------- Convert RAW data to Kg
  for (int i=0; i<(4); i++){
    arr[i+4] = P[i+4][0]*pow(result[i],2) + P[i+4][1]*result[i] + P[i+4][2];
    //Serial.print(result[k]);
    Serial.print(arr[i+4],3);
    Serial.print("\t");
  }

  Serial.print('\n');
  // PRINT ALL VALUES
  if (PRINT){
    Serial.print("Tension: ");
    for (int i=0; i<(N_joints*2); i++){
      //Serial.print(result[k]);
      //Serial.print("Joint ");
      Serial.print(arr[i],2);
      Serial.print(" ; ");
    }
    Serial.print('\n');
  }

  // Send ROS message
  tension_val.data = arr;
  pub_tensions.publish(&tension_val);
  nh.spinOnce();
  delay(1);
}
