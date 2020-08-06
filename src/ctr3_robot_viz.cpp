#include <medlab_common/ctr3_robot_viz.h>
#include <medlab_common/ctr3_robot.h>
#include <medlab_common/medlab_types.h>
#include <medlab_common/robotics_math.h>

#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>


CTR3RobotViz::CTR3RobotViz():
  prevBackbonePts_(0),
  init_flag_(false)
{
}

void CTR3RobotViz::init(std::string name_space)
{
  name_space_ = name_space;
  pubViz_ = nh_.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );

  init_flag_ = true;
}


void CTR3RobotViz::updateViz(CTR3Robot &robot)
{
  // ensure we've initialized
  if(!init_flag_){
    return;
  }

  medlab::CTR3RobotParams robotParams = robot.GetCurRobotParams();
  RoboticsMath::Vector6d robotCurQVec = robot.GetCurrQVec();
  Eigen::Vector3d robotCurBeta = robotCurQVec.bottomRows(3);

  // interpolate robot backbone
  medlab::InterpRet robot_backbone = robot.GetInterpolatedBackbone();
  int backbonePts = robot.GetNPts() + robot.GetNInterp();

  // marker template
  visualization_msgs::Marker marker;
  marker.ns = name_space_;
  marker.header.stamp = ros::Time();
  marker.header.frame_id = "/world";
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.color.a = 1.0f;

  // marker_array allocation
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.reserve(250);

  // create rViz marker for each backbone point
  for (int j=0; j<(backbonePts); j++)
  {
    marker.id = j;

    double px = robot_backbone.p(0,j);
    double py = robot_backbone.p(1,j);
    double pz = robot_backbone.p(2,j);
    double qw = robot_backbone.q(0,j);
    double qx = robot_backbone.q(1,j);
    double qy = robot_backbone.q(2,j);
    double qz = robot_backbone.q(3,j);

    if(j>prevBackbonePts_)
    {
      // delete if we have less backbone points than last time
        marker.action = visualization_msgs::Marker::DELETE;
    }
    else
    {
      // Set marker action. ADD/DELETE/DELETEALL
      marker.action = visualization_msgs::Marker::ADD;

      // Calulate the distance between this and the next marker
      double gap = 0.0;
      if (j < (backbonePts-1) )
      {
        double px_next = robot_backbone.p(0,j+1);
        double py_next = robot_backbone.p(1,j+1);
        double pz_next = robot_backbone.p(2,j+1);
        gap = sqrt( (px-px_next)*(px-px_next) + (py-py_next)*(py-py_next) + (pz-pz_next)*(pz-pz_next) );
      }

      if (gap<0.00001)
      {
          gap=0.00001;
      }

      // Set the pose of the marker
      // Since the cylinder marker "grows" in both direction, need to displace the first marker
      if (j==0){
          // basically (0,0,1) rotated by the quaternion then times half the gap is the displaced amount
          marker.pose.position.x = px+gap/2*(2*qw*qy-2*qz*qx);
          marker.pose.position.y = py+gap/2*(2*qw*qx+2*qy*qz);
          marker.pose.position.z = pz+gap/2*(1+2*qx*qx-2*qy*qy);
      }
      else
      {
          marker.pose.position.x = px;
          marker.pose.position.y = py;
          marker.pose.position.z = pz;
      }

      marker.pose.orientation.w = qw; // convention wxyz
      marker.pose.orientation.x = qx;
      marker.pose.orientation.y = qy;
      marker.pose.orientation.z = qz;


      // generally make the length of the cylinders twice the gaps
      // for the first cylinder, the length should equal to the gap
      // make the length of the last marker arbitrarily small
      if (j == backbonePts)
      {
          marker.scale.z = 0.00000005;
      }
      else if (j == 1)
      {
          marker.scale.z = gap;
      }
      else
      {
          marker.color.r = 0.0f; // green
          marker.color.g = 1.0f;
          marker.color.b = 0.0f;

          marker.scale.x = 1.168e-3; // width of inner tube
          marker.scale.y = 1.168e-3;
          marker.scale.z = gap*2;
      }

      // Set Color and Scale
      if (robotCurBeta[1] > robot_backbone.s[j] ||
          robotParams.L2+robotCurBeta[1] < robot_backbone.s[j])
      {
        // INNER TUBE
        marker.color.r = 0.0f; // green
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;

        marker.scale.x = 1.168e-3; // width of inner tube
        marker.scale.y = 1.168e-3;
      }
      else if ((robotCurBeta[1] <= robot_backbone.s[j] && robotCurBeta[2] > robot_backbone.s[j]) ||
               (robotParams.L3 + robotCurBeta[2] < robot_backbone.s[j] && robotParams.L2 + robotCurBeta[1] >= robot_backbone.s[j]))
      {
        // MIDDLE TUBE
        marker.color.r = 1.0f; // red
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;

        marker.scale.x = 1.684e-3; // width of middle tube
        marker.scale.y = 1.684e-3;
      }
      else
      {
        // OUTER TUBE
        marker.color.r = 0.0f; // blue
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;

        marker.scale.x = 2.324e-3; // width of outer tube
        marker.scale.y = 2.324e-3;
      }

    }

  // Add to the marker_array
  marker_array.markers.push_back(marker);

  } // ! create rViz marker for each backbone point

  // publish marker_array
  pubViz_.publish(marker_array);

  prevBackbonePts_ = backbonePts;
}


//for (int i=0; i <prevLength; i++)
//{
//    visualization_msgs::Marker marker = ComputeMarkerPoseAndColor(i);

//    if (marker.pose.position.z > 0) // only publish markers in front of plate
//    {
//      // Publish marker
//      //std::cout<<"published"<<std::endl;
//      shape_pub.publish(marker);
//      ID=ID+1;
//      if (ID==length)
//      {
//          ID=0;
//          length=0;
//          new_message=false;
//      }
//    }
//}



