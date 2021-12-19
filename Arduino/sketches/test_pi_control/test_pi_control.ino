///////   31.5.2021
    ///PID controller for platform movment on Lead Screw Rail.


#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
 
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
void status_linear_homing(const std_msgs::Bool& msg);
///////////////////

//=====[ VARIABLES ]============================================================
ros::NodeHandle nh;
std_msgs::Int32 homing_status;

ros::Publisher pub("/robot_snake_10/homing_cmd", &homing_status);
ros::Subscriber<std_msgs::Float32> sub("/robot_snake_10/linear_cmd", &set_linear_cmd);
ros::Subscriber<std_msgs::Bool> sub1("/robot_snake_10/doing_linear_homing", &status_linear_homing);

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
bool linear_status = false; 

void setup() {
  
  Serial.begin(115200);
  while(!Serial);


  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(HOME_MS_PIN, INPUT);
  pinMode(LIM_MS_PIN, INPUT); 

  digitalWrite(DIR_PIN, LOW);
  analogWrite(PWM_PIN, 0);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub1);
  nh.advertise(pub);
  delay (500);

}

void loop() {
 
                 digitalWrite(DIR_PIN, LOW);
          analogWrite(PWM_PIN, 150);
          delay (500);
          analogWrite(PWM_PIN, 0);
          
          /*
                    digitalWrite(DIR_PIN,HIGH);
          analogWrite(PWM_PIN, 150);
          delay (500);
          analogWrite(PWM_PIN, 0);
          */
}
/////////////////////end loop///////////////////

////////////homing/////////////////////////////
void homing(){
  
  
  
}

void set_linear_cmd(const std_msgs::Float32& msg){
  lastMessage = millis();
  DESIRED = msg.data;
}

void status_linear_homing(const std_msgs::Bool& msg){
  linear_status = msg.data;
}
