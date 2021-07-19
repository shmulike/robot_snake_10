// Shmulik Edelman
// shmulike@post.bgu.ac.il
//------------------------------------------------------------

// -------------------- Includer directories --------------------
#include "ros/ros.h"
#include "ros/time.h"
#include <signal.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "math.h"
#include <sys/time.h>
#include <ctime>
#include <chrono>
#include <iostream>

// -------------------- Define constants --------------------
#define sinAmp1 25
#define N_links 6
#define PI 3.14159265
#define step 8000.0
#define step_factor 6.0
#define dtheta 360.0/50000000.0
#define max_angle 5.0
#define deg2rad  PI/180.0

// -------------------- Global variables --------------------

std_msgs:: Float32MultiArray joint_cmd_array;

float arr[N_links] = {0.0};
float PWM[N_links*2] = {0.0};
ros::Publisher pub_1, pub_led;

// -------------------- Functions defenition --------------------

void mySigintHandler(int sig);


// --------------------------------------------------------------
int main(int argc, char **argv)
{

    ros::init(argc, argv, "move_test_1", ros::init_options::NoSigintHandler);
    
    ros::NodeHandle n;
    signal(SIGINT, mySigintHandler);

    // -------------------- Publishers --------------------
    pub_1 = n.advertise<std_msgs::Float32MultiArray>("/robot_snake_10/joint_cmd", 1000);



    ros::Rate loop_rate(100);

    using std::cout; using std::endl;
    using std::chrono::duration_cast;
    using std::chrono::milliseconds;
    using std::chrono::system_clock;
    

    ROS_WARN("->\tController node launched !");
    ROS_WARN("\tWaiting now for 5 seconds to start");

/*
    arr[0] = 0;
    arr[1] = 0;
    arr[2] = 0;
    arr[3] = 0;
    arr[4] = 0;
    */
    joint_cmd_array.data.clear();

    // Send zero angel for N_links joints
    for (int joint_i=0; joint_i<N_links; joint_i++){
        joint_cmd_array.data.push_back( arr[joint_i] );    
    }

    // If ros NOT ok
    while (!ros::ok());
    pub_1.publish(joint_cmd_array);
    ros::spinOnce();
    loop_rate.sleep();
    ros::Duration(15, 0).sleep();
    ROS_WARN("\tStarting");
    

    int joint_i = 0;
    auto start = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count(); // atart time
    auto now = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count(); //time right now
    double Time_period = (double)60.0*pow(10,3); // time of one feriod in milisecsec
    
    while (ros::ok())
    {

        now = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count(); //update the time right now
        double cmd_ang = max_angle*sin(1.0/Time_period*2*PI*(now-start)); // calculate the angel from SIN function
        //ROS_INFO("now: %ld\tjoint_i: %d\tcmd:%lf", now-start, joint_i ,cmd_ang);
        joint_cmd_array.data.clear();

        
        for (int i=0; i<N_links; i++){
            if (joint_i==i)
             joint_cmd_array.data.push_back(cmd_ang);
                /*
            if(joint_i==i-1)
                joint_cmd_array.data.push_back(cmd_ang);
            if(joint_i==i-2)
                joint_cmd_array.data.push_back(cmd_ang);
                */
            else
                joint_cmd_array.data.push_back(0.0);
        }
        pub_1.publish(joint_cmd_array);

            
            if (now-start>=Time_period){
                start = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count(); //if one sine wave ended update the start time
                joint_i++; // mobe to the next JOINT
                if (joint_i==N_links){
                    for (int i=0; i<N_links; i++)
                        joint_cmd_array.data.push_back(0.0);
                    pub_1.publish(joint_cmd_array);
                    break;
                }
            }
        
        /*
        if (now-start>=Time_period) 
            for (int i = 0; i<N_links; i++){
                if (i%2) // Go on even joints 0 2 4 6
                    joint_cmd_array.data.push_back(0.0);
                else // Go on odd joints 1 3 5 7
                    joint_cmd_array.data.push_back(cmd_ang + i*0.2);
            }
        else
            for (int i=0; i<N_links; i++)
                joint_cmd_array.data.push_back(0.0);
        
        pub_1.publish(joint_cmd_array);
        */
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
    ROS_ERROR("--> Shutting Down. Get all joints to Home-Position");
    pub_1.publish(joint_cmd_array);

  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}