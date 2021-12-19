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

#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"

int A;

bool add(std_srvs::SetBool::Request  &req,
         std_srvs::SetBool::Response &res)
{
    if (req.data==0){
        A=1;
    }
    else
      A=0;
    
    if(A==1){
      res.success=true;
        //return true;
    }
    else
      res.success=false;


      
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("empty2", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}