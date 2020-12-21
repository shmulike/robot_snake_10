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
#define d_count 1000
//#define CHANNEL_COUNT sizeof(DOUTS)/sizeof(byte)

#define N_tensions 8
long count = 0;
unsigned long startTime, endTime;

//=====[ VARIABLES ]=========================================
ros::NodeHandle nh;
byte DOUTS[N_tensions] = {DOUT1, DOUT2, DOUT3, DOUT4, DOUT5, DOUT6, DOUT7, DOUT8};
int tension_order[N_tensions] = {1, 2, 3, 4, 5, 6, 7, 8};

int k;
long int result[N_tensions];
float arr[N_tensions]={0};
HX711MULTI scales(N_joints*2, DOUTS, CLK);

std_msgs::Float32MultiArray tension_val;
ros::Publisher pub_tensions("/robot_snake_1/tension_val", &tension_val);

   double P[N_tensions][3] = {{0.00000000, 3.32E-5, 2.9418},
                              {0.00000000, 3.35E-5, 4.3512},
                              {0.00000000, 3.31E-5, 5.2857},
                              {0.00000000, 3.31E-5, -6.6821},
                              {0.00000000, 3.43E-5, -1.8389},
                              {0.00000000, 3.25E-5, 4.6884},
                              {0.00000000, 3.44E-5, 7.9783},
                              {0.00000000, 3.41E-5, 0.3253}};
                     
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
    arr[i] = P[i][0]*pow(result[i],2) + P[k][1]*result[i] + P[i][2];
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
      //Serial.print("Joint ");
      Serial.print(arr[i],4);
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
