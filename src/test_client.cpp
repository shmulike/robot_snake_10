#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<std_srvs::SetBool>("empty2");
  std_srvs::SetBool srv;
 

 

    if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    //return 1;
  }

  srv.request.data=false;

  std::cout<<client.call(srv)<<std::endl;


  

  return 0;
}