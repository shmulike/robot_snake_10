// Shmulik Edelman
// shmulike@post.bgu.ac.il
//------------------------------------------------------------

#include "ros/ros.h"
#include "ros/time.h"
#include <signal.h>
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "math.h"
#include <vector>

#include <iostream>
#include <sstream>
#include <stack>
#include <ctime>
//#include "../include/my_header.h"
#include "../include/PID.h"
#include "../include/LowPassFilter-master/LowPassFilter.hpp"
//#include <chrono>
//#include <iostream>

#include <cstdio>

using namespace std;


#define ROS_rate 300
//#define N_string 2              // In single joints
#define N_links 10          // Number of controlled joints
#define N_links_def 10         // Total number of joints in the system
#define N_tensions N_links*2    // Number of strings (2 for each joints)
#define N_motors N_links*2      // Number of motors (2 for each joints)
#define eps_angle 0.06

//#define Kp_tension 100.0       //40
//#define Kd_tension 0.0
//#define Ki_tension 1.0       //1
#define eps_tension 0.03
//#define MAX_PWM_tension 60                     // 255

//-----------------------------------------------------------
#define limit_angle_error 40.0          // joint angle limit [deg]
#define limit_angle_warn 35.0
#define limit_max_tension 35.0        // String tension limit [Kg]
#define limit_min_tension 0.2        // String tension limit [Kg]
#define tension_slop 0.00001 // y=tension_slop*x
#define step_tension 20

// -------------------- Global variables --------------------



double Kp_angle[N_links_def] = {25, 35, 50, 35, 50, 50, 50, 50, 70, 50};
double Ki_angle[N_links_def] = {100, 35, 20, 4, 5, 5, 5, 5, 7, 5};
double Kd_angle[N_links_def] = {0.0, 2.0, 2.0, 3.0, 2.0, 3.0, 3.0, 3.0, 3.0, 3.0};

double Kp_tension[N_links_def] = {40, 5, 24, 5, 2, 7, 7, 7, 7, 7};
double Ki_tension[N_links_def] = {200, 0.3, 0.9, 0.5, 0.4, 0.3, 0.4, 0.3, 0.4, 0.3};
double Kd_tension[N_links_def] = {0.0, 2.0, 0.0, 0.5, 0.5, 0.0, 0.5, 0.0, 0.5, 0.0};

double MAX_PWM_angle[N_links_def] = {200, 200, 200, 200, 200, 200, 200, 200, 200, 200};
double MAX_PWM_tension[N_links_def] = {160, 160, 160, 160, 160, 160, 160, 160, 160, 160};
double MAX_PWM_tension_fix[N_links_def] = {165, 165, 165, 165, 165, 165, 165, 165, 165, 165};

double tension_cmd[N_links_def] = {0.5, 1.5, 0.5, 1.5, 0.5, 1.5, 0.5, 1.5, 0.5, 1.5};


double joint_val[N_links_def] = {0}, joint_cmd[N_links_def] = {0};

double linear_val = 0, linear_cmd = 0;
int motor_cmd[2][N_links_def] = {0}, motor_cmd_flat[N_links * 2] = {0};
double tension_val[2][N_links_def] = {0};
double Weight_Matrix[2*N_links_def][2*N_links_def]={{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                    {0.15,0,0,0,0.07,0,0.03,0,0.09,0,0,0,0.09,0,0,0,0.03,0,0,0.07},
                                                    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                    {0,0,0,0,0.15,0,0,0,0.07,0,0,0,0.09,0,0,0,0.03,0,0,0.03},
                                                    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                    {0,0,0,0,0,0,0,0,0.15,0,0,0,0.09,0,0,0,0.07,0,0,0},
                                                    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                    {0,0,0,0,0,0,0,0,0,0,0,0,0.15,0,0,0,0.09,0,0,0},
                                                    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.15,0,0,0},
                                                    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};

double pwm_temp = 0;
std_msgs::Int32MultiArray motor_cmd_PWM;
bool alive[2] = {0};        // [0] Joints -- [1] Tensions
double last_tension[2][N_links_def] = {0};
int last_PWM[2][N_links_def]={0};
int vec_mot_cmd[N_links_def*2]= {0};
double joint_val_prev[N_links_def]={0};
int count_return[N_links_def]= {0};
ros::Publisher pub_motor_cmd;

// -------------------- Functions definition --------------------
void mySigintHandler(int sig);

void get_joint_val(const std_msgs::Float32MultiArray::ConstPtr &msg);

void get_joint_cmd(const std_msgs::Float32MultiArray::ConstPtr &msg);

void get_linear_val(const std_msgs::Float32::ConstPtr &msg);

void get_linear_cmd(const std_msgs::Float32::ConstPtr &msg);

void get_tension_val(const std_msgs::Float32MultiArray::ConstPtr &msg);

double signOf(double num);

void publish_motor_cmd(int motor_cmd[][N_links_def]);

double linear_move_ten(int k);

void conv_mat_to_vec(int *m );

void mul_vec_and_mat(int *m , double b[2*N_links_def][2*N_links_def]);

void conv_1Dvec_to_2Dvec(int *m );

std::stack <clock_t> tictoc_stack;
std::clock_t temp_time = std::clock();

std::clock_t static tic() { return std::clock(); }

double static toc(std::clock_t start) {
    return ((std::clock() - start) / ((double) CLOCKS_PER_SEC)) * 1000;
}

int main(int argc, char **argv) {
    //std::vector<PID> joint_CTRLA;//(N_links, PID(0,0,0,0,0,0));
    //std::vector<PID> joint_CTRLB;//(N_links, PID(0,0,0,0,0,0));
    std::vector <std::pair<PID, PID>> joints_CTRL;

	

    // Create PID objects
    std::cout << "create PID" << std::endl;
    for (int joint_i = 0; joint_i < N_links; joint_i++) {
        joints_CTRL.push_back({PID(1.0 / ROS_rate, MAX_PWM_angle[joint_i], -MAX_PWM_angle[joint_i], Kp_angle[joint_i],
                                   Ki_angle[joint_i], Kd_angle[joint_i]),
                               PID(1.0 / ROS_rate, MAX_PWM_tension[joint_i], -MAX_PWM_tension[joint_i],
                                   Kp_tension[joint_i], Ki_tension[joint_i], Kd_tension[joint_i])});
    }

    ros::init(argc, argv, "controller_4", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    ros::Rate loop_rate(ROS_rate);
    signal(SIGINT, mySigintHandler);
    // -------------------- Publishers --------------------
    pub_motor_cmd = n.advertise<std_msgs::Int32MultiArray>("/robot_snake_4/motor_cmd", 1000);
    // -------------------- Subscribers --------------------
    ros::Subscriber sub_1 = n.subscribe("/robot_snake_10/joint_val", 1000, get_joint_val);
    ros::Subscriber sub_2 = n.subscribe("/robot_snake_10/joint_cmd", 1000, get_joint_cmd);
    ros::Subscriber sub_3 = n.subscribe("/robot_snake_1/linear_val", 1000, get_linear_val);
    ros::Subscriber sub_4 = n.subscribe("/robot_snake_1/linear_cmd", 1000, get_linear_cmd);
    ros::Subscriber sub_5 = n.subscribe("/robot_snake_1/tension_val", 1000, get_tension_val);
    ROS_WARN("-> Controller node launched !");
    ROS_WARN("--> Waiting for all nodes to publish !");

    //while (!alive[0]) ROS_INFO("teensy not working %d", alive[0]);
    //while (!alive[1]) ROS_INFO("tension not working %d", alive[1]);
    //while (!alive[0] && !alive[1]);
    //ROS_INFO("-> wait 5 sec");
    //ros::Duration(5, 0).sleep();
    int m = 1, s = 1;
    while (ros::ok()) {
        if (!alive[0] || !alive[1]) {
            if (!alive[0])
                ROS_ERROR("Waiting for teensy 'Joints' to wake up");
            if (!alive[1])
                ROS_ERROR("Waiting for teensy 'Tension sensor' to wake up");
            ros::Duration(2, 0).sleep();        // wait between checking if Teensy is alive}
        } else {
            // We are ONLINE lets get to WORK
            for (int joint_i = 0; joint_i < N_links; joint_i++) {

                if (joint_cmd[joint_i] - joint_val[joint_i] < 0 && joint_i % 2 > 0) {
                    motor_cmd[1][joint_i] =
                            -1 * joints_CTRL[joint_i].first.calculate(joint_cmd[joint_i], joint_val[joint_i]);

                    // String 2 - tension - PID controller
                    motor_cmd[0][joint_i] =
                            1 * joints_CTRL[joint_i].second.calculate(tension_cmd[joint_i], tension_val[0][joint_i]);
                } else {
                    // Run on every joint
                    // String 1 - angle - PID controller
                    //joints_CTRL[joint_i].first.resetSum();
                    motor_cmd[0][joint_i] = joints_CTRL[joint_i].first.calculate(joint_cmd[joint_i],
                                                                                 joint_val[joint_i]);

                    // String 2 - tension - PID controller
                    motor_cmd[1][joint_i] = joints_CTRL[joint_i].second.calculate(tension_cmd[joint_i],
                                                                                  tension_val[1][joint_i]);
                }
            
                
                if (fabs(joint_val[joint_i]) > limit_angle_warn) {
                    ROS_WARN("-> Joint #%d - angle got to its limit: %.2lf", joint_i, joint_val[joint_i]);
                    if (fabs(joint_val[joint_i]) > limit_angle_error) {
                        if (joint_cmd[joint_i] - joint_val[joint_i] < 0 && joint_i % 2 > 0) {
                            motor_cmd[1][joint_i] = -MAX_PWM_angle[N_links] * signOf(joint_val[joint_i]);
                            motor_cmd[0][joint_i] = 0;
                        } else {
                            motor_cmd[0][joint_i] = -MAX_PWM_angle[N_links] * signOf(joint_val[joint_i]);
                            motor_cmd[1][joint_i] = 0;
                        }
                        ROS_ERROR("-> Joint #%d - angle got to its MAX limit: %.2lf\t--> BacKi_angleng off", joint_i,
                                  joint_val[joint_i]);
                    }
                }


/*
                // For each string in joint-i
                for (int j = 0; j < 2; j++) {
                    // If tension if bigger than MAX limit -> stop pulling, release opposite string
                    if (tension_val[j][joint_i] > limit_max_tension) {
                       // if ((joint_cmd[joint_i] - joint_val[joint_i]) < 0 && joint_i % 2 > 0) {
                            //motor_cmd[1][joint_i] = 0;
                            motor_cmd[j][joint_i] += -(tension_slop*MAX_PWM_tension[joint_i]);
                       // } else {
                            //motor_cmd[0][joint_i] = 0;
                            //motor_cmd[1][joint_i] += -(tension_slop*MAX_PWM_tension[joint_i]);
                        //}
                        ROS_FATAL("J%d - String #%d - tension got to its max limit: %.2lf", joint_i, 2 * joint_i + j,
                                  tension_val[j][joint_i]);
                    }
                        // If tension if smaller than MIN limit -> pull
                    else if (tension_val[j][joint_i] < limit_min_tension) {
                        //if (joint_cmd[joint_i] - joint_val[joint_i] < 0 && joint_i % 2 > 0) {
                            motor_cmd[j][joint_i] += (tension_slop*MAX_PWM_tension[joint_i]);
                            //joints_CTRL[joint_i].first.resetSum();
                            //joints_CTRL[joint_i].second.resetSum();
                    ROS_FATAL("J%d - String #%d - tension got to its min limit: %.2lf ", joint_i, 2 * joint_i + j,
                                  tension_val[j][joint_i]);
                    }
                    else if(j==1){ // Fix motor command
                        if (joint_i==0){
                            motor_cmd[1][0] += -0.11 * motor_cmd[0][0]-0.6 * motor_cmd[0][6]-0.6 * motor_cmd[0][4]-0.4 * motor_cmd[1][9]-0.4 * motor_cmd[0][2]
                                   -0.2 * motor_cmd[1][3]-0.2 * motor_cmd[0][8];
                        }
                        if(joint_i==2)
                            motor_cmd[1][2] += -0.11 * motor_cmd[0][2]-0.6 * motor_cmd[0][6]-0.4 * motor_cmd[0][4]-0.2 * motor_cmd[1][9]-0.1 * motor_cmd[0][8];
                        if(joint_i==4)
                            motor_cmd[1][4] += -0.11 * motor_cmd[0][4]-0.6 * motor_cmd[0][6]-0.4 * motor_cmd[0][8];
                         }
                        //} else {
                          //  motor_cmd[1][joint_i] += (tension_slop*MAX_PWM_tension[joint_i]);
                        //}

                        }  // ---------- End check limits

                //ROS_INFO("Joint #%d:\tPos/cmd= %.3f/%.1f;\tPWM=[%d,%d];\tT1= %.2f;\tT2= %.2f\tE_sum=%.2f\tkP=%.2f\tE_sum=%.2f\tkP=%.2f",joint_i, joint_val[joint_i],joint_cmd[joint_i], motor_cmd[0][joint_i], motor_cmd[1][joint_i], tension_val[0][joint_i], tension_val[1][joint_i],joint_error_sum[joint_i],joint_error[joint_i]*Kp_angle[joint_i],tension_error[1][joint_i]*Kp_tension , tension_error_sum[1][joint_i]);

                ROS_INFO(
                        "Joint #%d:\tPos/cmd= %.3lf/%.3lf;\tPWM=[%d,%d];\tT1= %.2lf;\tT2= %.2lf\tIout= %.2lf\tPout= %.2lf",
                        joint_i, joint_val[joint_i], joint_cmd[joint_i], motor_cmd[0][joint_i], motor_cmd[1][joint_i],
                        tension_val[0][joint_i], tension_val[1][joint_i], joints_CTRL[joint_i].first.Iout,
                        joints_CTRL[joint_i].first.Pout);
                 */        
            } // End of links loop
            ROS_INFO("--------------------");

            //convert 2D array to 1D array by take 1 col from motor_cmd and push in the new array 
            conv_mat_to_vec(vec_mot_cmd); 
                   
            // multipy weight matrix eith the valu of the motor
            mul_vec_and_mat(vec_mot_cmd , Weight_Matrix);
            
            //convert 1D array to 2D array 
            conv_1Dvec_to_2Dvec(vec_mot_cmd);

            // Publish motor command to micro-controller
            publish_motor_cmd(motor_cmd);

        } // end of else
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

    motor_cmd_PWM.data.clear();
    for (int joint_i = 0; joint_i < N_links * 2; joint_i++) {
        motor_cmd_PWM.data.push_back(0);
    }
    ROS_ERROR("--> Shutting Down. Stopping all motors\tsig: %d", sig);
    pub_motor_cmd.publish(motor_cmd_PWM);

    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

void get_joint_val(const std_msgs::Float32MultiArray::ConstPtr &msg) {
    alive[0] = true;
    for (int i = 0; i < N_links; i++) {
        joint_val_prev[i]=joint_val[i];
        joint_val[i] = double(msg->data[i]);
        
        /**
        if(joint_val_prev[i]==joint_val[i]){
           
            count_return[i]+=1;
                if(count_return[i]== 100){
                    ROS_ERROR("THE SAME ANGEL %d , joint_val: %lf, joint_val_prev: %lf", i, joint_val[i], joint_val_prev[i]);
                    ros::shutdown();
                }    
        }
        else{
            count_return[i]=0;
        }
        **/
    }
}

void get_joint_cmd(const std_msgs::Float32MultiArray::ConstPtr &msg) {
    for (int i = 0; i < N_links; i++) {
        //joint_cmd[i] = msg->data[i];
        joint_cmd[i] = double(msg->data[i]);
    }
}

void get_linear_val(const std_msgs::Float32::ConstPtr &msg) {
    linear_val = msg->data;
}

void get_linear_cmd(const std_msgs::Float32::ConstPtr &msg) {
    linear_cmd = msg->data;
}

void get_tension_val(const std_msgs::Float32MultiArray::ConstPtr &msg) {
    alive[1] = true;
    for (int joint_i = 0; joint_i < N_links; joint_i++) {
        for (int j = 0; j < 2; j++) {
            last_tension[j][joint_i]= tension_val[j][joint_i];
            tension_val[j][joint_i] = double(fmax(msg->data[joint_i * 2 + j], 0.0));
            if(abs(last_tension[j][joint_i]-tension_val[j][joint_i])>20)
                  tension_val[j][joint_i]=last_tension[j][joint_i];
            //tension_val[j][joint_i] =  fmax(msg->data[joint_i*2+j],0.0);
            //tension_val[j][i] = tension_to_Kg (i*2+j, msg->data[i*2+j]);
        }
    }
}

void publish_motor_cmd(int motor_cmd[][N_links_def]) {
    motor_cmd_PWM.data.clear();
    for (int joint_i = 0; joint_i < N_links_def; joint_i++) {
        for (int j = 0; j < 2; j++) {
            //if(abs(last_PWM[j][joint_i]-motor_cmd[j][joint_i])>70)
                //motor_cmd[j][joint_i]=last_PWM[j][joint_i]+ 0.1* motor_cmd[j][joint_i];
              //  motor_cmd[j][joint_i]=tension_slop*MAX_PWM_tension[joint_i]/step_tension+last_PWM[j][joint_i];
            motor_cmd_PWM.data.push_back(motor_cmd[j][joint_i]);
            last_PWM[j][joint_i]=motor_cmd[j][joint_i];
        }
    }
    
    pub_motor_cmd.publish(motor_cmd_PWM);
}

double signOf(double num) {
    if (num >= 0)
        return 1;
    return -1;
}

double linear_move_ten(int k){
    static int tension_PWM=0;
    double cur_PWM;
    if (tension_PWM <(MAX_PWM_tension[k]/tension_slop))
        cur_PWM=tension_slop*tension_PWM;
    tension_PWM +=1;
return(cur_PWM);
}

void conv_mat_to_vec(int *m ){
      
int k=0;
    for(int i=0; i<N_links_def; i++){
        for(int j=0; j<2;j++){
             m[k]=motor_cmd[j][i];
             k+=1;
        }
    }
}

void mul_vec_and_mat(int *m , double b[2*N_links_def][2*N_links_def]){

    for(int i=0; i<N_links_def*2; i++){
      for(int j=0; j<N_links_def*2;j++){
             m[i]+=(int)(m[j]*b[i][j]);
        }
    }
}

void conv_1Dvec_to_2Dvec(int *m ){

int k=0;
    for(int i=0; i<N_links_def; i++){
        for(int j=0; j<2;j++){

            // If tension if bigger than MAX limit -> stop pulling, release opposite string
            if (tension_val[j][i] > limit_max_tension) {
                 motor_cmd[j][i] += -(tension_slop*MAX_PWM_tension[i]);
                 ROS_FATAL("J%d - String #%d - tension got to its max limit: %.2lf", i, 2 * i + j,tension_val[j][i]);
                    }
            
            // If tension if smaller than MIN limit -> pull
            else if (tension_val[j][i] < limit_min_tension) {
                motor_cmd[j][i] += (tension_slop*MAX_PWM_tension[i]);
                ROS_FATAL("J%d - String #%d - tension got to its min limit: %.2lf ", i, 2 * i + j,tension_val[j][i]);
            }
            else{
            motor_cmd[j][i]= m[k];
            }
            k++;
        }
            ROS_INFO(
            "Joint #%d:\tPos/cmd= %.3lf/%.3lf;\tPWM=[%d,%d];\tT1= %.2lf;\tT2= %.2lf",
            i, joint_val[i], joint_cmd[i], motor_cmd[0][i], motor_cmd[1][i],
            tension_val[0][i], tension_val[1][i]);
    }
}


