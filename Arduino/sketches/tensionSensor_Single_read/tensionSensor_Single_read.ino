//==========================================================
//Shmulik Edelman
//shmulike@post.bgu.ac.il
// This code reading form MULTI HX711 Load-cells aplifier and sending there value as ROS-message
//==========================================================

//=====[ INCULDE ]==========================================
#include "HX711.h"

//=====[ Constants ]========================================

#define CLK 11      // clock pin to the load cell amp
#define DOUT A3    // data pin to the first lca
#define calib_mat_row 0

#define BOOT_MESSAGE "MIT_ML_SCALE V0.8"
#define TARE_TIMEOUT_SECONDS 4


//=====[ VARIABLES ]=========================================


long int val_raw;
double val;
HX711 scale;

double P[8][3] ={{0.00000000, 3.32E-5, 2.9418},
                              {0.00000000, 3.35E-5, 4.3512},
                              {0.00000000, 3.31E-5, 5.2857},
                              {0.00000000, 3.31E-5, -6.6821},
                              {0.00000000, 3.43E-5, -1.8389},
                              {0.00000000, 3.25E-5, 4.6884},
                              {0.00000000, 3.44E-5, 7.9783},
                              {0.00000000, 3.41E-5, 0.3253}};
                     
//=====[ SETUP ]=============================================
void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Serial Communication started");

  scale.begin(DOUT,CLK);
  

}

//=====[ LOOP ]==============================================
void loop() {
  val_raw = scale.read();                // Read raw data from all the tension-meters
  val = P[calib_mat_row][0]*pow(val_raw, 2) + P[calib_mat_row][1]*val_raw + P[calib_mat_row][2];

  Serial.print("Raw: ");
  Serial.print(val_raw);
  Serial.print("\tKg: ");
  Serial.println(val,3);

  delay(13);
}
