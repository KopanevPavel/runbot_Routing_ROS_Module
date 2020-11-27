#include "ros/ros.h"
#include "runbot_routing_machine/ParseWptsService.h"
#include <iostream>
#include <ctype.h>
#include <cstring>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <unistd.h>
#include <sstream>
#include <fstream>
#include <string.h>
#include <vector>
#include <ros/console.h>

using namespace std;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "routing_machine_client");
  if (argc != 2)
  {
    ROS_INFO("usage: routing_machine_client X");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<runbot_routing_machine::ParseWptsService>("runbot_routing_machine/get_wpts");
  runbot_routing_machine::ParseWptsService srv;
  srv.request.get_wpts = atoll(argv[1]);
  // std::cout << atoll(argv[1]) << std::endl;
  if (client.call(srv))
  {
    ROS_INFO("Service was called");
  }
  else
  {
    ROS_ERROR("Failed to call service get_wpts");
    return 1;
  }

  return 0;
}