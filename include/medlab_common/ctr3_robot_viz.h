#pragma once
#ifndef CTR3_ROBOT_VIZ__H
#define CTR3_ROBOT_VIZ__H

#include <medlab_common/ctr3_robot.h>
#include <ros/ros.h>


// Q_DECLARE_METATYPE(CTR3Robot*)

class CTR3RobotViz
{
  //Q_OBJECT

public:
  CTR3RobotViz();
  void init(std::string name_space); // advertises topic to rViz

  void updateViz(CTR3Robot& robot); // creates marker array and publishes to rViz

private:
  ros::NodeHandle nh_;
  std::string name_space_;
  ros::Publisher  pubViz_;
  int prevBackbonePts_;
  bool init_flag_;
};

#endif // CTR3_ROBOT_VIZ__H
