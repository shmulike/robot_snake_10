//==========================================================
//Shmulik Edelman
//shmulike@post.bgu.ac.il
// This code reading form MULTI HX711 Load-cells aplifier and sending there value as ROS-message
//==========================================================

//=====[ INCULDE ]==========================================
#include "HX711.h"

//=====[ Constants ]========================================

#define CLK 11      // clock pin to the load cell amp
#define DOUT A6    // data pin to the first lca
#define calib_mat_row 0

#define BOOT_MESSAGE "MIT_ML_SCALE V0.8"
#define TARE_TIMEOUT_SECONDS 4


//=====[ VARIABLES ]=========================================


long int val_raw;
double val;
HX711 scale(DOUT, CLK);

double P[8][3] = {{0.00000000, 0.00002348, 1.47896687},
                              {0.00000000, 0.00002385, 2.88188982},
                              {0.00000000, 0.00002414, 1.27118260},
                              {0.00000000, 0.00002028,-4.84178993},
                              {0.00000000, 0.00002322, 3.38832415},
                              {0.00000000, 0.00002559, 4.95853685},
                              {0.00000000, 0.00002408, 6.59883064},
                              {0.00000000, 0.00002436, 1.13493471}};
                     
//=====[ SETUP ]=============================================
void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Serial Communication started");

}

//=====[ LOOP ]==============================================
void loop() {
  val_raw = scale.read();                // Read raw data from all the tension-meters
  val = P[calib_mat_row][0]*pow(val_raw, 2) + P[calib_mat_row][1]*val_raw + P[calib_mat_row][2];

  Serial.print("Raw: ");
  Serial.print(val_raw);
  Serial.print("\tKg: ");
  Serial.println(val,3);

  delay(2);
}
