
#include <Encoder.h>
#define E_max 80
#define PWM_MAX 255
#define PWM_MAX_HOMING 100
#define homing_error 0.004
#define count_num 30

static const float TIC_TO_MM = 94.3648;
//pin decleration//
static const int HOME_MS_PIN = 6; // back microswitch pin
static const int LIM_MS_PIN = 8;  // front microswitch pin
static const int PWM_PIN = 23;
static const int DIR_PIN = 22;// LOW=forward, HIGH=backward
//////////////////////

static Encoder myEnc(2,1);

//PID const//
static  float Ki = 0.5;
static  float Kp = 120;
static  float Kd = 4;
//HOMING PI const//
static  float Ki_hom = 0.5;
static  float Kp_hom = 45;
////////////////////
float DESIRED =650;         
float  CALC_DESIRED=0 ;      
unsigned long currentTime,previousTime=0,elapsedTime; //var for dError
double lastError=0,dError;
int dir=1;
int homing_end=10;
float pwm_val=0, E=0, Position,error,tmp_pos=0;
//func decleration//
void homing();
///////////////////

void setup() {
  
  Serial.begin(115200);
  while(!Serial);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(HOME_MS_PIN, INPUT);
  pinMode(LIM_MS_PIN, INPUT);
  homing();
  delay (500);
}

void loop() {
  
     Position = myEnc.read();
     if(CALC_DESIRED<DESIRED){
      CALC_DESIRED+=0.04;
      }
     error = CALC_DESIRED - Position/TIC_TO_MM;
    
    currentTime=millis();
    elapsedTime= currentTime-previousTime;
     
Serial.print("\nDes: ");
Serial.print(CALC_DESIRED,3);
Serial.print("\tCurrent pos: ");
Serial.print(Position/TIC_TO_MM);
Serial.print("\tError: ");
Serial.print(error, 3);
Serial.print("\tError sum: ");
Serial.print(E);
Serial.print("\tPWM: ");
Serial.print(pwm_val);

if (DESIRED<0){Serial.print("\n Distance too small \n");}

if (DESIRED>790){Serial.print("\n Distance too big \n");}

    if(digitalRead(HOME_MS_PIN)&&digitalRead(LIM_MS_PIN)&&CALC_DESIRED>=0&&DESIRED>0&&DESIRED<790){
        
      E += error;//error sum
      if (E>E_max)
        E = E_max;
      else if (E<-E_max)
        E=-E_max;
      dError= (error - lastError)/elapsedTime;
      
      pwm_val = Kp*error + Ki*E+Kd*dError;
      
      pwm_val = (pwm_val>0 ? min(pwm_val, PWM_MAX):max(pwm_val, -PWM_MAX)); // keep it below abs(255)
      
      digitalWrite(DIR_PIN, pwm_val<0 ? LOW : HIGH);
      analogWrite(PWM_PIN, abs(pwm_val));
    }
 
 else if (!digitalRead(HOME_MS_PIN)){ // if back microswitch is pressed, move forward for 1 sec
    digitalWrite(DIR_PIN,HIGH);
    analogWrite(PWM_PIN, 150);
    delay (1000);
    analogWrite(PWM_PIN, 0);
       }
    
 else if (!digitalRead(LIM_MS_PIN)){ // if front microswitch is pressed, move backward for 1 sec
    digitalWrite(DIR_PIN,LOW);
    analogWrite(PWM_PIN, 150);
    delay (1000);
    analogWrite(PWM_PIN, 0);
       }
    
 else analogWrite(PWM_PIN, 0);
 delay(1);
 
lastError = error;
previousTime = currentTime; 
}
///end loop//

////////////homing/////////////////////////////
void homing(){
  Serial.println("Start Homing");
  delay(200);
 int  counter=0;
  int pwm_val_1 = 160, pwm_val_2 = 60,i=0;
  
// move forwad for 1 sec//
  while (i<2000&&digitalRead(LIM_MS_PIN)){
  analogWrite(PWM_PIN, pwm_val_1);
  digitalWrite(DIR_PIN,HIGH);
  delay(4);
  i++;
  }
  
  //move BACK until presses
  analogWrite(PWM_PIN, pwm_val_1);
  digitalWrite(DIR_PIN, LOW);
  while(digitalRead(HOME_MS_PIN)) {
    delay(2);
  }
  
  //move FORWARD for 1 sec
  analogWrite(PWM_PIN, pwm_val_1);
  digitalWrite(DIR_PIN, HIGH);
  delay(1000);

  //move BACK until presses
  analogWrite(PWM_PIN, pwm_val_2);
  digitalWrite(DIR_PIN, LOW);
  while(digitalRead(HOME_MS_PIN)) {     
    delay(2);
  }
 analogWrite(PWM_PIN, 0);
 delay(1000);

//set new 0 point//
  myEnc.write(0);
  error = homing_end;
    while (counter<count_num){
      
    Position = myEnc.read();
     error = homing_end - Position/TIC_TO_MM;
         E += error;//error sum
      if (E>E_max)
        E = E_max;
      else if (E<-E_max)
        E=-E_max;
           
      pwm_val = Kp_hom*error + Ki_hom*E;
      
      pwm_val = (pwm_val>0 ? min(pwm_val, PWM_MAX_HOMING):max(pwm_val, -PWM_MAX_HOMING)); // keep it below abs(100)
      
      digitalWrite(DIR_PIN, pwm_val<0 ? LOW : HIGH);
      analogWrite(PWM_PIN, abs(pwm_val));
      delay(2);
      if (abs(error)<=homing_error) counter++;
      else counter=0;
      Serial.print("\tHoming end:");
      Serial.print(homing_end);
      Serial.print("\tPOS:");
      Serial.print(Position/TIC_TO_MM,6);
      Serial.print("\terror:");
      Serial.print(error,6);
      Serial.print("\tPWM: ");
      Serial.print(pwm_val);
      Serial.print("\tcounter:");
      Serial.println(counter);
      }
  
  //STOP
  analogWrite(PWM_PIN, 0);
  digitalWrite(DIR_PIN, LOW);
 
  //Set encoder to 0
  Serial.println("Done Homing");
  delay(1000);
  myEnc.write(0);
}
