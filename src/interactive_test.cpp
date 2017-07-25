#include <ros/ros.h>

#include "giskard_sim/scenario_instance.h"

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM( feedback->marker_name << " is now at "
      << feedback->pose.position.x << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z );
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_marker");

  giskard_sim::ScenarioInstance scenario;

  giskard_sim::SWorldObject worldObject;
  worldObject.name = "meh";
  worldObject.parent = "odom_combined";

  scenario.addSceneObject(&worldObject);

  // start the ROS main loop
  ros::spin();
}
// %Tag(fullSource)%
