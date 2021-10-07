///////   31.5.2021
    ///PID controller for platform movment on Lead Screw Rail.


#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
 
#include <Encoder.h>
#define E_max 80
#define PWM_MAX 255
#define PWM_MAX_HOMING 100
#define homing_error 0.004
#define count_num 30
#define maxRailLength 790
#define lastMessageOk 500 // millisec

static const float TIC_TO_MM = 94.3648;
//pin decleration//
static const int HOME_MS_PIN = 6;   // back microswitch pin.
static const int LIM_MS_PIN = 8;   // front microswitch pin.
static const int PWM_PIN = 23;
static const int DIR_PIN = 22;    // LOW=forward, HIGH=backward.


//func decleration//
void homing();
void set_linear_cmd(const std_msgs::Float32& msg);
void LED_blink();
///////////////////

//=====[ VARIABLES ]============================================================
ros::NodeHandle nh;

std_msgs::Int32 homing_status;

ros::Publisher pub("/robot_snake_10/homing_cmd", &homing_status);

ros::Subscriber<std_msgs::Float32> sub("/robot_snake_10/linear_cmd", &set_linear_cmd);
/////////

static Encoder myEnc(2,1);

//PID const//
static  float Ki = 4;
static  float Kp = 150;
static  float Kd = 7;
//HOMING PI const//
static  float Ki_hom = 0.5;
static  float Kp_hom = 45;
////////////////////
float DESIRED =0.0;      //distance from back microswitch to back robot plate. USER INPUT    
float  CALC_DESIRED=0.0 ;      
unsigned long currentTime, previousTime=0, elapsedTime; //var for dError.
double lastError=0,dError;
int dir=1;           //Direction.
int homing_end=10;
float pwm_val=0, E=0, Position,error,tmp_pos=0;
int flag=1, connectionOk = 0;
unsigned int lastMessage = 0;

void setup() {
  
  Serial.begin(115200);
  while(!Serial);


  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(HOME_MS_PIN, INPUT);
  pinMode(LIM_MS_PIN, INPUT);
  // Placing the robot at a starting point...
  // becuse we have Relative encoder and not abs encoder.
  // homing();
  // LED_blink(); 

   digitalWrite(DIR_PIN, LOW);
   analogWrite(PWM_PIN, 0);
   homing_status.data=1;
   flag=1;
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  delay (500);

}

void loop() {

        if(flag==1){
        nh.logwarn("stop motion");
        digitalWrite(DIR_PIN, LOW);
        analogWrite(PWM_PIN, 0);
        homing_status.data=1;
        nh.logwarn("----stop motion----");
  }

     
     if (nh.connected()){  

     flag=2;
     Position = myEnc.read();     // Reading of a current situation.-
     
      //char str[10];
      //dtostrf(Position/TIC_TO_MM,6,2, str);
      //nh.logwarn(str);
      /*
     if(abs(CALC_DESIRED-DESIRED)>0.1)
        if(CALC_DESIRED<DESIRED)
          CALC_DESIRED+=0.02;
        else
          CALC_DESIRED-=0.02;
     else
      CALC_DESIRED = DESIRED;
      */
        
     //error = CALC_DESIRED - Position/TIC_TO_MM;  // calc current error.
    error = DESIRED - Position/TIC_TO_MM; 
    CALC_DESIRED = DESIRED;

    currentTime=millis();
    elapsedTime= currentTime-previousTime;
   /*
    // print to serial monitor.     
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
    */
    
    // make sure the input is valid
    if (DESIRED<0){
      //Serial.print("\n Distance too small \n");
       analogWrite(PWM_PIN, 0);
       digitalWrite(DIR_PIN, LOW);
       nh.logwarn("ROS NOT CONNECTED");
       homing_status.data=1;
       flag=1;
      }
    if (DESIRED>790){
        //Serial.print("\n Distance too big \n");
             homing();  
        }
if(digitalRead(HOME_MS_PIN)&&digitalRead(LIM_MS_PIN)&&CALC_DESIRED>=0&&DESIRED>=0&&DESIRED<maxRailLength){
        
      E += error;   // value of error for integral controller.
      /////////////// Limit the value     why???
      ///////////////if (E>E_max){E = E_max;}
      ///////////////else if (E<-E_max){E=-E_max;}
      dError= (error - lastError)/elapsedTime;     //// value of error for derivative controller.
      pwm_val = Kp*error + Ki*E+Kd*dError;
      
      pwm_val = (pwm_val>0 ? min(pwm_val, PWM_MAX):max(pwm_val, -PWM_MAX));    // saturation limit
         // clamping
      if((abs(pwm_val)==PWM_MAX)&&(error*pwm_val>0)){
        E=0;
        pwm_val = Kp*error+Ki*E+Kd*dError;
        pwm_val = (pwm_val>0 ? min(pwm_val, PWM_MAX):max(pwm_val, -PWM_MAX)); // saturation limit
        }
        
      // Send PWM signal to the motor driver
      digitalWrite(DIR_PIN, pwm_val<0 ? LOW : HIGH);
      analogWrite(PWM_PIN, abs(pwm_val));
    }
 
       else if (!digitalRead(HOME_MS_PIN)){ // if back microswitch is pressed, move forward for 1 sec
          //LED_blink();
          digitalWrite(DIR_PIN,HIGH);
          analogWrite(PWM_PIN, 150);
          delay (500);
          analogWrite(PWM_PIN, 0);
             }
          
       else if (!digitalRead(LIM_MS_PIN)){ // if front microswitch is pressed, move backward for 1 sec
          //LED_blink();
          digitalWrite(DIR_PIN, LOW);
          analogWrite(PWM_PIN, 150);
          delay (500);
          analogWrite(PWM_PIN, 0);
             }
          
       else analogWrite(PWM_PIN, 0);
       delay(1);
       
       lastError = error;
       previousTime = currentTime; 
       nh.spinOnce();
     }
     else{
       nh.logwarn("Waiting For ROS");
       analogWrite(PWM_PIN, 0);
       digitalWrite(DIR_PIN, LOW);
       homing_status.data=0;
     }
  pub.publish(&homing_status);
  nh.spinOnce();
  delay(2);

}
/////////////////////end loop///////////////////

////////////homing/////////////////////////////
void homing(){
  nh.logwarn("arduino start homing");
  homing_status.data=2;
  pub.publish(&homing_status);
  nh.spinOnce();
  //Serial.println("Start Homing");
  delay(200);
 int  counter=0;
  int pwm_val_1 = 160, pwm_val_2 = 60,i=0;
  
// move forwad for 1 sec//
  while (i<500 && digitalRead(LIM_MS_PIN)){
  analogWrite(PWM_PIN, pwm_val_1);
  digitalWrite(DIR_PIN,HIGH);
  delay(2);
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
  delay(500);

  //move BACK until presses
  analogWrite(PWM_PIN, pwm_val_2);
  digitalWrite(DIR_PIN, LOW);
  while(digitalRead(HOME_MS_PIN)) {     
    delay(2);
  }
 analogWrite(PWM_PIN, 0);
 delay(200);

  ///set new 0 point//
  myEnc.write(0);
  error = homing_end;
    while (counter<count_num){
    //while (error < 0.01){
      
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
      /*
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
      */
      }
      
//STOP
  analogWrite(PWM_PIN, 0);
  digitalWrite(DIR_PIN, LOW);
 
  //Set encoder to 0
  //Serial.println("Done Homing");
  myEnc.write(0);
  
  homing_status.data=3; // DONE HOMEING
  pub.publish(&homing_status);
  delay(500);
  //char str[10];
  //dtostrf(myEnc.read()/TIC_TO_MM,6,2, str);
  //nh.logwarn(str);
  nh.logwarn("DONE HOMEING");
}

void set_linear_cmd(const std_msgs::Float32& msg){
  lastMessage = millis();
  DESIRED = msg.data;
}

void LED_blink(){
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(50);
  digitalWrite(13, LOW);
  delay(50);
}
