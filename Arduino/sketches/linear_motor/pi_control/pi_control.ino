
#include <Encoder.h>
#define E_max 80
#define PWM_MAX 255

static const float TIC_TO_MM = 94.3648;
//pin decleration//
static const int HOME_MS_PIN = 6; // backward microswitch pin
static const int LIM_MS_PIN = 8;  // forward microswitch pin
static const int PWM_PIN = 23;
static const int DIR_PIN = 22;// LOW=forward, HIGH=backward
//////////////////////

static Encoder myEnc(2,1);

//pid const//
static  float Ki = 0.5;   //best for 1 Aterr=1.81
static  float Kp = 120;
static  float Kd = 4;
////////////////////
float DESIRED =400;         //distance from back black box to perspex in mm(min 16)
float  CALC_DESIRED=0 ;        //=DESIRED-0;
unsigned long currentTime,previousTime=0,elapsedTime; //var for dError
double lastError=0,dError;
int dir=1;
int cycle=0,counter=0;
float pwm_val=0, E=0, Position,error;
//func decleration//
void homing2();
///////////////////

void setup() {

//if(abs(DESIRED)<11){Ki=0.3;Kp=15;}
  Serial.begin(115200);
  Serial.println("Test");
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(HOME_MS_PIN, INPUT);
  pinMode(LIM_MS_PIN, INPUT);
  homing2();
  delay (2500);
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

//Serial.println(error);
//Serial.print("\tpos counter: ");
//Serial.print(counter);
//Serial.print("\tcycle: ");
//Serial.println(cycle);

if (CALC_DESIRED<0){Serial.print("\n Distance too small \n");}

    if(digitalRead(HOME_MS_PIN)&&digitalRead(LIM_MS_PIN)&&CALC_DESIRED>=0){
        
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

 else analogWrite(PWM_PIN, 0);
 delay(1);
 
lastError = error;
previousTime = currentTime; 
}

void homing2(){
  Serial.println("Start Homing");
  delay(200);
  int pwm_val_1 = 130, pwm_val_2 = 60;

  analogWrite(PWM_PIN, pwm_val_1);
  digitalWrite(DIR_PIN,HIGH);
  delay(1000);
  
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
  while(digitalRead(HOME_MS_PIN)) {     //when pressed move forward
    delay(2);
  }
  analogWrite(PWM_PIN, pwm_val_2);
  digitalWrite(DIR_PIN,HIGH);
  delay(300);

  //STOP
  analogWrite(PWM_PIN, 0);
  digitalWrite(DIR_PIN, LOW);

  //Set encoder to 0
  Serial.println("Done Homing");
  delay(1000);
   myEnc.write(0);
}
