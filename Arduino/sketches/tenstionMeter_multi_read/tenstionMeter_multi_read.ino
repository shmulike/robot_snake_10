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
#define PRINT 1
#define N_joints 10
#define CLK 11      // clock pin to the load cell amp
#define DOUT1 A0    // data pin to the first lca
#define DOUT2 A1    // data pin to the second lca
#define DOUT3 A2    // data pin to the third lca
#define DOUT4 A3    // data pin to the third lca
#define DOUT5 A4    // data pin to the third lca
#define DOUT6 A5    // data pin to the third lca
#define DOUT7 A6    // data pin to the third lca
#define DOUT8 A7    // data pin to the third lca
#define DOUT9 A8    // data pin to the third lca
#define DOUT10 A9    // data pin to the third lca
#define DOUT11 A12
#define DOUT12 A13
#define DOUT13 A14
#define DOUT14 A15
#define DOUT15 A16
#define DOUT16 A17
#define DOUT17 A18
#define DOUT18 A19
#define DOUT19 A20
#define DOUT20 A21
#define BOOT_MESSAGE "MIT_ML_SCALE V0.8"
#define d_count 1000
//#define CHANNEL_COUNT sizeof(DOUTS)/sizeof(byte)

#define N_tensions 20
long count = 0;
unsigned long startTime, endTime;

//=====[ VARIABLES ]=========================================
ros::NodeHandle nh;
byte DOUTS[N_tensions] = {DOUT1, DOUT2, DOUT3, DOUT4, DOUT5, DOUT6, DOUT7, DOUT8, DOUT9, DOUT10, DOUT11, DOUT12, DOUT13, DOUT14, DOUT15, DOUT16, DOUT17, DOUT18, DOUT19, DOUT20};
//int tension_order[N_tensions] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19 ,20};
//byte DOUTS[N_tensions] = {DOUT1, DOUT2, DOUT3, DOUT4};
//int tension_order[N_tensions] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
int k;
long int result[N_tensions];
float arr[N_tensions]={0};
HX711MULTI scales(N_joints*2, DOUTS, CLK);

std_msgs::Float32MultiArray tension_val;
ros::Publisher pub_tensions("/robot_snake_1/tension_val", &tension_val);

   double P[20][3] = {{0.00000000, 3.32E-5, 2.9418},
                              {0.00000000, 3.35E-5, 4.3512},
                              {0.00000000, 3.31E-5, 5.2857},
                              {0.00000000, 3.31E-5, -6.6821},
                              {0.00000000, 3.43E-5, -1.8389},
                              {0.00000000, 3.25E-5, 4.6884},
                              {0.00000000, 3.44E-5, 7.9783},
                              {0.00000000, 3.41E-5, 0.3253},
                              {0.00000000, 3.51E-5, -0.4163},
                              {0.00000000, 3.76E-5, 11.2153},
                              {0.00000000, 3.2E-5, 0.5870},
                              {0.00000000, 3.32E-5, 2.0759},
                              {0.00000000, 3.49E-5, -0.3476},
                              {0.00000000, 3.38E-5, 0.5863},
                              {0.00000000, 3.22E-5, 9.1975},
                              {0.00000000,  3.43E-5, 1.3067},
                              {0.00000000, 3.45E-5, 11.111},
                              {0.00000000, 3.37E-5,0.0905},
                              {0.00000000, 3.12E-5,5.2910},
                              {0.00000000, 3.47E-5,5.0214}};                          
                  
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
  
  scales.readRaw(result);                // Read raw data from all the tension-meters
 
  
  // ---------- Convert RAW data to Kg
  for (int i=0; i<(N_joints*2); i++){
    //arr[i] = P[i][0]*pow(result[i],2) + P[k][1]*result[i] + P[i][2];
    arr[i] = P[i][0]*pow(result[i],2) + P[i][1]*result[i] + P[i][2];
    //Serial.print(arr[i],3);
    //Serial.print("\t");
  }


  // PRINT ALL VALUES
  if (PRINT){
    count++;
    
    Serial.print(count++);
    Serial.print(":");
    
    if (count%d_count==0){
      endTime = millis();
      //Serial.print(endTime-startTime);
      Serial.print(d_count*1000.0/(endTime-startTime));
      Serial.print("Hz");
      startTime=endTime;
    }
    Serial.print("\t");
    
    for (int i=0; i<(N_joints*2); i++){
      //Serial.print(result[k]);
      //Serial.print("t[i]:");
      Serial.print(arr[i],4);
      //Serial.print(result[i],4);
      Serial.print(" ; ");
    }
    Serial.print('\n');
    
  }


  
  Serial.print('\n');
  // Send ROS message
  tension_val.data = arr;
  pub_tensions.publish(&tension_val);
  nh.spinOnce();
  delay(10);
}
