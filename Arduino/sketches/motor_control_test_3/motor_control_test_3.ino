#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#define N_links 4
//-------------------- Methods
void set_motor_pwm(const std_msgs::Int32MultiArray& msg);
//-------------------- Global variables
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int32MultiArray> sub("/robot_snake_1/motor_cmd", &set_motor_pwm);

//juints {+1,-1,+2,-2,+3,-3,+4,-4} 
// motor num {1,2,8,5,6,7,4,3}
byte motors_PWM_pin[8] = {2,3,9,6,7,8,5,4};    
byte motors_DIR_pin[8] = {0,1,27,24,25,26,12,11};
//
//-------------------- Setup function
void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  
  //Serial.begin(115200);
  for (int i=0; i<(N_links*2); i++)
  {
    pinMode(motors_PWM_pin[i], OUTPUT);
    pinMode(motors_DIR_pin[i], OUTPUT);
  }
}

void loop() {
  //nh.logdebug("shmulik");
  nh.spinOnce();
  delay(2);
}

void set_motor_pwm(const std_msgs::Int32MultiArray& msg){
  int motor_pwm = 0;
  bool motor_dir = 0;
  for (int i=0; i<N_links*2; i++){
    motor_pwm = msg.data[i];
    if (motor_pwm>0)
      motor_dir = 1;
    else
      motor_dir = 0;
    digitalWrite(motors_DIR_pin[i], motor_dir);
    analogWrite(motors_PWM_pin[i],  abs(motor_pwm));
  }
}
