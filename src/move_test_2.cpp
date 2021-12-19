/*
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
#include "std_srvs/SetBool.h"
#include "math.h"
#include <sys/time.h>
#include <ctime>
#include <chrono>
#include <iostream>
#include <unistd.h>
#include <rosserial_arduino/Test.h>


// -------------------- Define constants --------------------
//#define sinAmp1 25
#define _Hz 100
#define N_links 10
#define N_links_def 10
#define PI 3.14159265
//#define step_factor 6.0
#define dtheta 360.0/50000000.0
#define max_angle 5.0
#define deg2rad  PI/180.0
#define max_move 200
#define N_steps 10000
#define joint_homing_step 0.001             // deg
#define joint_homing_eps 1.0      // Epsilon value for joint-homing procedure
#define joint_joming_avg_count 50
enum JOINT_HOMING_STATUS {
    waiting, working, done
}joint_homing_status;


// -------------------- Global variables --------------------
std_msgs::Float32MultiArray joint_cmd_array;
std_msgs::Float32 linear_move;
ros::Publisher pub_1, pub_led, pub_2;

float arr[N_links_def] = {0.0};
float PWM[N_links_def * 2] = {0.0};
float joint_val[N_links_def]{0.0}, joint_val_avg[N_links_def] = {0.0};

//float joint_mat[N_links][N_steps] = {0.0}, linear_mat[N_steps] = {0.0};
int homing_status = 1;                // ???
int flag = 1;                         // ???
//bool joint_homing_done = false;     // Flag for joint-homing procedure
bool joint_master_alive = false;    // Flag for joint-master alive
//enum JOINT_HOMING_STATUS joint_homing_status = waiting;

// -------------------- Functions definition --------------------
void mySigintHandler(int sig);
void get_homing_status(const std_msgs::Int32::ConstPtr &msg);
void get_joint_val(const std_msgs::Float32MultiArray::ConstPtr &msg);
JOINT_HOMING_STATUS joint_homing_fun();
void send_current_val(float joint_val_cur[N_links]);
// --------------------------------------------------------------
int main(int argc, char **argv) {
    ros::init(argc, argv, "move_test_1", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    
    ros::ServiceClient client = n.serviceClient<std_srvs::SetBool>("linear_homing_status");
    std_srvs::SetBool srv;
    ros::Rate loop_rate(_Hz);
    signal(SIGINT, mySigintHandler);

    // -------------------- Publishers --------------------
    pub_1 = n.advertise<std_msgs::Float32MultiArray>("/robot_snake_10/joint_cmd", 1000);
    pub_2 = n.advertise<std_msgs::Float32>("/robot_snake_10/linear_cmd", 1000);
    // -------------------- Subscribers --------------------
    ros::Subscriber sub_1 = n.subscribe("/robot_snake_10/homing_cmd", 1000, get_homing_status);
    ros::Subscriber sub_2 = n.subscribe("/robot_snake_10/joint_val", 1000, get_joint_val);
    ROS_WARN("->\tController node launched !");
    ROS_WARN("\tWaiting now for 5 seconds to start");

    using std::cout; using std::endl;
    using std::chrono::duration_cast;
    using std::chrono::milliseconds;
    using std::chrono::system_clock;

    float linear_cmd_mat[N_steps] = {0.0};
    float joint_cmd_mat[N_links_def][N_steps] = {0.0};
    float joint_cmd_final[] = {-1, 0, -1, 0, -1, -2, -1, -2, -1, 0};
    float linear_final = 150.0;
    int step_counter = 0;               // initilize the step counter
    joint_homing_status = waiting;

    // Why do we need this?
    // was itwriten  before the homing function??
    joint_cmd_array.data.clear();
    linear_move.data = 0.0;
    
    // Send zero angel for N_links joints
    for (int joint_i = 0; joint_i < N_links; joint_i++) {
        joint_cmd_array.data.push_back(arr[joint_i]);
    }
    pub_1.publish(joint_cmd_array);
    pub_2.publish(linear_move);

    // If ros NOT ok
    //while (!ros::ok());

    //ros::spinOnce();
    loop_rate.sleep();              // whu do we need sleep?
    ros::Duration(1, 0).sleep();    // why do we need duration sleep?
    ROS_WARN("\tStarting");

    for (int step_i = 0; step_i < N_steps; step_i++) {
        linear_cmd_mat[step_i] = float(linear_final / (N_steps * step_i));
        for (int joint_i = 0; joint_i < N_links; joint_i++)
            joint_cmd_mat[joint_i][step_i] = float(joint_cmd_final[joint_i] / (N_steps * step_i));
    }

    while (joint_master_alive == false)               // Wait for joints master teensy to be alive
    {
        ros::spinOnce();
    }

    ROS_WARN("Joint master is alive !!!");
    joint_homing_status = working;
    ROS_WARN("Joint master is alive !!!");
    
 


    
    while (ros::ok()) {
         srv.request.data=true; 
         client.call(srv);
    ROS_WARN("------------------in loop---------------------");
      if (srv.response.success=true){
             ROS_WARN("FINISH HOMING ");
            srv.request.data=false; 
            // ROS_WARN("-------IN THE LOOP----------");
            if (!joint_homing_status) {
                // If joint homing not done
                // Creating the homing commands
                // ROS_WARN("-------stack here----------");
            } 
            
            else {

                if (joint_homing_status == working) {
                    joint_homing_status = joint_homing_fun();
                    // ROS_WARN("-------HOMING JOINTS---------- %d", joint_homing_status);
                    
                } 
                
                else if (joint_homing_status == done) {
                       //ROS_WARN("-------HOMING JOINTS DONNNNNNNE----------");
                    /*
                     * Joint-homing is done
                     * Now we can start the test sequence
                    
                    //double cmd_linear = max_move*abs(sin(1.0/Time_period*2*PI*(now-start))); // calculate the angel from SIN function
                    joint_cmd_array.data.clear();
                    for (int joint = 0; joint < N_links; joint++) {
                        joint_cmd_array.data.push_back(joint_cmd_mat[joint][step_counter]);
                    }
                    linear_move.data = linear_cmd_mat[step_counter];
                    if (step_counter < N_steps - 1) {
                        step_counter += 1;
                        ROS_WARN("Step NO.:%d/%d", step_counter, N_steps);
                    }
                }
                pub_1.publish(joint_cmd_array);
                pub_2.publish(linear_move);
            }
      }
      if (srv.response.success=false){
            ROS_WARN("NOT FINISH HOMING");
            send_current_val(arr);
            pub_1.publish(joint_cmd_array);
            ros::spinOnce();
      }
   

   ////////////////////////////////////////////////////////////////////////////////////////////
       

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

void get_homing_status(const std_msgs::Int32::ConstPtr &msg) {
    homing_status = int(msg->data);
}

void get_joint_val(const std_msgs::Float32MultiArray::ConstPtr &msg) {
    //int homing_count = 50, homing_counter = 0;                              // set value for joints value average measure
    static int homing_read_counter = 0;
    joint_master_alive = true;                          // set the flag to joint_master connection is true = joint master serial node is alive
    for (int i = 0; i < N_links; i++) {
        joint_val[i] = msg->data[i];
    }
    if (joint_homing_status == working && homing_read_counter < joint_joming_avg_count){
        homing_read_counter++;
        for (int i = 0; i < N_links; i++) {
            joint_val_avg[i] += float(msg->data[i] / joint_joming_avg_count);
        }
    }
}

JOINT_HOMING_STATUS joint_homing_fun(){
    joint_cmd_array.data.clear();
    for (int i = 0; i < N_links; i++) {
        if (joint_val_avg[i] > joint_homing_step)
            joint_val_avg[i] -= joint_homing_step;
        else
            joint_val_avg[i] = 0.0;
        joint_cmd_array.data.push_back(joint_val_avg[i]);
    }
    for (int i=0; i < N_links; i++){
        ROS_INFO("%f,%f", std::fabs(joint_val[i]), joint_homing_eps);
        if (std::fabs(joint_val[i]) > joint_homing_eps){
            return working;
        }
     }
    return done;
    
}

void send_current_val(float joint_val_cur[N_links]){
    joint_cmd_array.data.clear();
    for (int i = 0; i < N_links; i++) 
    {
        joint_val_cur[i]=joint_val[i];
        joint_cmd_array.data.push_back( joint_val_cur[i] );
        }
    
}
*/

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
//#define sinAmp1 25
#define _Hz 100
#define N_links 10
#define N_links_def 10
#define PI 3.14159265
//#define step_factor 6.0
#define dtheta 360.0/50000000.0
#define max_angle 5.0
#define deg2rad  PI/180.0
#define max_move 200
#define N_steps 5000
#define joint_homing_step 0.001             // deg
#define joint_homing_eps 1.0      // Epsilon value for joint-homing procedure
#define joint_joming_avg_count 50
enum JOINT_HOMING_STATUS {
    waiting, working, done
}joint_homing_status;


// -------------------- Global variables --------------------
std_msgs::Float32MultiArray joint_cmd_array;
std_msgs::Float32 linear_move;
ros::Publisher pub_1, pub_led, pub_2;

float arr[N_links_def] = {0.0};
float PWM[N_links_def * 2] = {0.0};
float joint_val[N_links_def]{0.0}, joint_val_avg[N_links_def] = {0.0};

//float joint_mat[N_links][N_steps] = {0.0}, linear_mat[N_steps] = {0.0};
int homing_status = 1;                // ???
int flag = 1;                         // ???
//bool joint_homing_done = false;     // Flag for joint-homing procedure
bool joint_master_alive = false;    // Flag for joint-master alive
//enum JOINT_HOMING_STATUS joint_homing_status = waiting;

// -------------------- Functions definition --------------------
void mySigintHandler(int sig);
void get_homing_status(const std_msgs::Int32::ConstPtr &msg);
void get_joint_val(const std_msgs::Float32MultiArray::ConstPtr &msg);
JOINT_HOMING_STATUS joint_homing_fun();
void send_current_val(float joint_val_cur[N_links]);
// --------------------------------------------------------------


int main(int argc, char **argv) {


    ros::init(argc, argv, "move_test_1", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    ros::Rate loop_rate(_Hz);
    signal(SIGINT, mySigintHandler);

    // -------------------- Publishers --------------------
    pub_1 = n.advertise<std_msgs::Float32MultiArray>("/robot_snake_10/joint_cmd", 1000);
    pub_2 = n.advertise<std_msgs::Float32>("/robot_snake_10/linear_cmd", 1000);
    // -------------------- Subscribers --------------------
    ros::Subscriber sub_1 = n.subscribe("/robot_snake_10/homing_cmd", 1000, get_homing_status);
    ros::Subscriber sub_2 = n.subscribe("/robot_snake_10/joint_val", 1000, get_joint_val);
    ROS_WARN("->\tController node launched !");
    ROS_WARN("\tWaiting now for 5 seconds to start");

    using std::cout; using std::endl;
    using std::chrono::duration_cast;
    using std::chrono::milliseconds;
    using std::chrono::system_clock;

    float linear_cmd_mat[N_steps] = {0.0};
    float joint_cmd_mat[N_links_def][N_steps] = {0.0};
    float joint_cmd_final[] = {5, 5, 10, 10, 0, 0, -10, 5, 5, 0};
    float linear_final = 600.0;
    int step_counter = 0;               // initilize the step counter
    joint_homing_status = waiting;

    // Why do we need this?
    // was itwriten  before the homing function??
    joint_cmd_array.data.clear();
    linear_move.data = 0.0;
    
    // Send zero angel for N_links joints
    for (int joint_i = 0; joint_i < N_links; joint_i++) {
        joint_cmd_array.data.push_back(arr[joint_i]);
    }
    pub_1.publish(joint_cmd_array);
    pub_2.publish(linear_move);

    // If ros NOT ok
    //while (!ros::ok());

    //ros::spinOnce();
    loop_rate.sleep();              // whu do we need sleep?
    ros::Duration(1, 0).sleep();    // why do we need duration sleep?
    ROS_WARN("\tStarting");

    for (int step_i = 0; step_i < N_steps; step_i++) {
        linear_cmd_mat[step_i] = float(linear_final / (N_steps) * step_i);
        for (int joint_i = 0; joint_i < N_links; joint_i++)
            joint_cmd_mat[joint_i][step_i] = float((joint_cmd_final[joint_i] / (N_steps))* step_i);
    }

    while (joint_master_alive == false)               // Wait for joints master teensy to be alive
    {
        ros::spinOnce();
    }

    ROS_WARN("Joint master is alive !!!");
    //ROS_WARN("Joint master is alive !!!");

    while (ros::ok()) {
           
    //* Joint-homing is done
    //* Now we can start the test sequence
                    
    //double cmd_linear = max_move*abs(sin(1.0/Time_period*2*PI*(now-start))); // calculate the angel from SIN function
    joint_cmd_array.data.clear();
    for (int joint = 0; joint < N_links; joint++) {
        joint_cmd_array.data.push_back(joint_cmd_mat[joint][step_counter]);
        }
    linear_move.data = linear_cmd_mat[step_counter];
    if (step_counter < N_steps - 1) {
        step_counter += 1;
        ROS_WARN("Step NO.:%d/%d", step_counter, N_steps);
    }
        
     pub_1.publish(joint_cmd_array);
     pub_2.publish(linear_move);

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

void get_homing_status(const std_msgs::Int32::ConstPtr &msg) {
    homing_status = int(msg->data);
}

void get_joint_val(const std_msgs::Float32MultiArray::ConstPtr &msg) {
    //int homing_count = 50, homing_counter = 0;                              // set value for joints value average measure
    static int homing_read_counter = 0;
    joint_master_alive = true;                          // set the flag to joint_master connection is true = joint master serial node is alive
    for (int i = 0; i < N_links; i++) {
        joint_val[i] = msg->data[i];
    }
    if (joint_homing_status == working && homing_read_counter < joint_joming_avg_count){
        homing_read_counter++;
        for (int i = 0; i < N_links; i++) {
            joint_val_avg[i] += float(msg->data[i] / joint_joming_avg_count);
        }
    }
}

JOINT_HOMING_STATUS joint_homing_fun(){
    joint_cmd_array.data.clear();
    for (int i = 0; i < N_links; i++) {
        if (joint_val_avg[i] > joint_homing_step)
            joint_val_avg[i] -= joint_homing_step;
        else
            joint_val_avg[i] = 0.0;
        joint_cmd_array.data.push_back(joint_val_avg[i]);
    }
    for (int i=0; i < N_links; i++){
        ROS_INFO("%f,%f", std::fabs(joint_val[i]), joint_homing_eps);
        if (std::fabs(joint_val[i]) > joint_homing_eps){
            return working;
        }
     }
    return done;
    
}

void send_current_val(float joint_val_cur[N_links]){
    joint_cmd_array.data.clear();
    for (int i = 0; i < N_links; i++) 
    {
        joint_val_cur[i]=joint_val[i];
        joint_cmd_array.data.push_back( joint_val_cur[i] );
        }
    
}
