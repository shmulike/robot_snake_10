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
//#define sinAmp1 25
#define _Hz 25
#define N_links 10
#define N_links_def 10
#define PI 3.14159265
#define sinAmp1 5
#define step 2000.0
#define step_factor 20.0
#define deg2rad  PI/180.0

// -------------------- Global variables --------------------
std_msgs::Float32MultiArray joint_cmd_array;
ros::Publisher pub_1;

float arr[N_links_def] = {0.0};
float counter = 0;
int fase = 1;


// -------------------- Functions definition --------------------
void mySigintHandler(int sig);


// --------------------------------------------------------------


int main(int argc, char **argv) {


    ros::init(argc, argv, "circle_move", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    ros::Rate loop_rate(_Hz);
    signal(SIGINT, mySigintHandler);

    // -------------------- Publishers --------------------
    pub_1 = n.advertise<std_msgs::Float32MultiArray>("/robot_snake_10/joint_cmd", 1000);
 

    ROS_WARN("->\tController node launched !");
    ROS_WARN("\tWaiting now for 5 seconds to start");

    using std::cout; using std::endl;
    using std::chrono::duration_cast;
    using std::chrono::milliseconds;
    using std::chrono::system_clock;

    
    // Send zero angel for N_links joints
    for (int joint_i = 0; joint_i < N_links; joint_i++) {
        joint_cmd_array.data.push_back(arr[joint_i]);
    }
    pub_1.publish(joint_cmd_array);

    loop_rate.sleep();              // whu do we need sleep?
    ros::Duration(10, 0).sleep();    // why do we need duration sleep?
    ROS_WARN("\tStarting");


    while (ros::ok()) {
           
ROS_WARN("Fase : %d\tCounter : %f", fase, counter);
            if (counter < 0.25)
            {
                counter += 2.0*PI/(step*step_factor);
                fase = 1;
            }
            if (counter >= 0.25 && counter < 1.25)
            {
                counter += 2.0*PI/(step*step_factor);
                fase = 2;
            }
            if (counter >= 1.25 && counter < 1.5)
            {
                counter += 2.0*PI/(step*step_factor);
                fase = 3;
            }

            switch(fase){
                case 1:
                arr[0] = sinAmp1 * sin(counter*2*PI);
                arr[1] = 0;
                arr[2] = sinAmp1 * sin(counter*2*PI);
                arr[3] = 0;
                arr[4] = -sinAmp1 * sin(counter*2*PI);
                arr[5] = 0;
                arr[6] = -sinAmp1 * sin(counter*2*PI);
                arr[7] = 0;
                arr[8] = 0;
                arr[9] = 0;
                break;

                case 2:
                arr[0] =  sinAmp1 * sin(counter*2*PI);
                arr[1] =  sinAmp1 * cos(counter*2*PI);
                arr[2] =  sinAmp1 * sin(counter*2*PI);
                arr[3] =  sinAmp1 * cos(counter*2*PI);
                arr[4] = -sinAmp1 * sin(counter*2*PI);
                arr[5] = -sinAmp1 * cos(counter*2*PI);
                arr[6] = -sinAmp1 * sin(counter*2*PI);
                arr[7] = -sinAmp1 * cos(counter*2*PI);
                //arr[7] = 0;
                arr[8] =  0;
                arr[9] =  0;
                break;

                case 3:
                arr[0] = sinAmp1 * sin(counter*2*PI);
                arr[1] = 0;
                arr[2] = sinAmp1 * sin(counter*2*PI);
                arr[3] = 0;
                arr[4] = -sinAmp1 * sin(counter*2*PI);
                arr[5] = 0;
                arr[6] = -sinAmp1 * sin(counter*2*PI);
                arr[7] = 0;
                arr[8] = 0;
                arr[9] = 0;
                break;
            }            
            
            joint_cmd_array.data.clear();
            for (int joint_i=0; joint_i<N_links; joint_i++){
                joint_cmd_array.data.push_back( arr[joint_i] );    
            }
            
            pub_1.publish(joint_cmd_array);

        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::shutdown();
    return 0;
}

// ------------------------------------------------------------
// -------------------- Functions --------------------

void mySigintHandler(int sig) {
    // Do some custom action.
    // For example, publish a stop message to some other nodes.

    joint_cmd_array.data.clear();
    for (int joint_i = 0; joint_i < N_links; joint_i++) {
        joint_cmd_array.data.push_back(0.0);
    }
    //linear_move.data= 0.0;
    ROS_ERROR("--> Shutting Down. Get all joints to Home-Position");
    pub_1.publish(joint_cmd_array);
    //pub_2.publish(linear_move);
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

