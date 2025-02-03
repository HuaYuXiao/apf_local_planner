#include <ros/ros.h>
#include "local_planning.h"

using namespace apf_local_planner;

int main(int argc, char** argv){
  ros::init(argc, argv, "apf_local_planner");
  ros::NodeHandle nh("~");

  Local_Planner apf_local_planner;
  apf_local_planner.init(nh);

  ros::spin();

  return 0;
}
