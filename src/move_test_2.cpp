// Shmulik Edelman
// shmulike@post.bgu.ac.il
//------------------------------------------------------------

// -------------------- Includer directories --------------------
#include "ros/ros.h"
#include "ros/time.h"
#include <signal.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "math.h"
#include <sys/time.h>
#include <ctime>
#include <chrono>
#include <iostream>
#include <unistd.h>

// -------------------- Define constants --------------------
#define sinAmp1 25
#define N_links 10
#define PI 3.14159265
#define step_factor 6.0
#define dtheta 360.0/50000000.0
#define max_angle 5.0
#define deg2rad  PI/180.0
#define max_move 200
#define N_steps 10000

// -------------------- Global variables --------------------

std_msgs:: Float32MultiArray joint_cmd_array;
std_msgs:: Float32 linear_move;


float arr[N_links] = {0.0};
float PWM[N_links*2] = {0.0};
ros::Publisher pub_1, pub_led, pub_2;
int homing_status=1;
int flag=1;

// -------------------- Functions defenition --------------------

void mySigintHandler(int sig);
void get_homing_status(const std_msgs::Int32::ConstPtr& msg);


// --------------------------------------------------------------
int main(int argc, char **argv)
{

    ros::init(argc, argv, "move_test_1", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    signal(SIGINT, mySigintHandler);

    // -------------------- Publishers --------------------
    pub_1 = n.advertise<std_msgs::Float32MultiArray>("/robot_snake_10/joint_cmd", 1000);
    pub_2 = n.advertise<std_msgs::Float32>("/robot_snake_10/linear_cmd", 1000);
    // -------------------- Subscribers --------------------
    ros::Subscriber sub_1 = n.subscribe("/robot_snake_10/homing_cmd",     1000, get_homing_status);
    ROS_WARN("->\tController node launched !");
    ROS_WARN("\tWaiting now for 5 seconds to start");

    using std::cout; using std::endl;
    using std::chrono::duration_cast;
    using std::chrono::milliseconds;
    using std::chrono::system_clock;
    
    joint_cmd_array.data.clear();
    linear_move.data = 0.0;

    // Send zero angel for N_links joints
    for (int joint_i=0; joint_i<N_links; joint_i++){
        joint_cmd_array.data.push_back( arr[joint_i] );    
    }
    pub_1.publish(joint_cmd_array);
    pub_2.publish(linear_move);

    // If ros NOT ok
    //while (!ros::ok());

    //ros::spinOnce();
    loop_rate.sleep();
    ros::Duration(1, 0).sleep();
    ROS_WARN("\tStarting");
    

    int joint_i = 8; //Change Start point JOINT
    auto start = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count(); // start time
    auto now = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count(); //time right now
    double Time_period = (double)120.0*pow(10,3); // time of one feriod in milisecsec
    double linear_mat[N_steps] = {0.0};
    double angle_mat[10][N_steps] = {0.0};
    double angle_final[]={-1,0,-1,0,-1,-2,-1,-2,-1,0};
    double linear_final = 150.0;
    int step = 0;

    // Creating joint and linear commands
    for (int joint=0; joint<10; joint++)
        for (int i=0; i<N_steps; i++)
        {
            double dangle = angle_final[joint]/N_steps;
            angle_mat[joint][i] = double(dangle*i);
        }
    for (int i=0; i<N_steps; i++)
        {
        double dlinear = linear_final/N_steps;
        linear_mat[i] = double(dlinear*i);
        }

    while (ros::ok())
    {
        
        if(homing_status==1){
            //ROS_INFO("\tstart homing");        
            linear_move.data= 800;
            pub_2.publish(linear_move);  
            //homing_status = 2; 
        }

        else if(homing_status==3)
        {
            if (flag==1)
            {

                flag=2;

            }
        
        
        //double cmd_linear = max_move*abs(sin(1.0/Time_period*2*PI*(now-start))); // calculate the angel from SIN function
        joint_cmd_array.data.clear();
        for (int joint=0; joint<N_links; joint++){
            joint_cmd_array.data.push_back(angle_mat[joint][step]);
        }
        linear_move.data=  linear_mat[step];
        if (step<N_steps-1)
        {
            step+=1;
            ROS_WARN("Step NO.:%d", step);
        }
        pub_1.publish(joint_cmd_array);
        pub_2.publish(linear_move);     
            
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::shutdown();
    return 0;
}

// ------------------------------------------------------------
// -------------------- Functions --------------------

void mySigintHandler(int sig){
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
    joint_cmd_array.data.clear();
    for (int joint_i=0; joint_i<N_links; joint_i++){
        joint_cmd_array.data.push_back( 0.0 );    
    }
    //linear_move.data= 0.0;
    ROS_ERROR("--> Shutting Down. Get all joints to Home-Position");
    pub_1.publish(joint_cmd_array);
    //pub_2.publish(linear_move); 
  // All the default sigint handler does is call shutdown()
    ros::shutdown();
}


void get_homing_status(const std_msgs::Int32::ConstPtr& msg){

        homing_status = int(msg->data);
    }
