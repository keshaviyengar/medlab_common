//
// Created by keshav on 2020-07-29.
//

#include <ros/ros.h>
#include <medlab_common/ctr3_robot_viz.h>
#include <medlab_common/ctr3_robot.h>
#include <medlab_common/medlab_types.h>
#include <medlab_common/robotics_math.h>
#include <geometry_msgs/Twist.h>

#include <sensor_msgs/JointState.h>

class JointListener
{
public:
    void init(RoboticsMath::Vector6d& initial_q);
    RoboticsMath::Vector6d q;

    void callback(const sensor_msgs::JointStateConstPtr &msg);
};

void JointListener::init(RoboticsMath::Vector6d& initial_q) {
    q = initial_q;
}
void JointListener::callback(const sensor_msgs::JointStateConstPtr &msg) {
    q[0] = msg->position[0];
    q[1] = msg->position[1];
    q[2] = msg->position[2];

    q[3] = msg->position[3];
    q[4] = msg->position[4];
    q[5] = msg->position[5];
}


int main(int argc, char *argv[])
{
    ros::init(argc,argv, "kinematics");
    ros::NodeHandle node;

    // Get robot parameters
    medlab::CTR3RobotParams robot_params;
    // Material properties
    robot_params.E = 60e9;
    robot_params.G = 60e9 / 2.0 / 1.33;

    // Tube 1 geometry
    robot_params.L1 = 222.5e-3;
    robot_params.Lt1 = robot_params.L1 - 42.2e-3;
    robot_params.OD1 = 1.165e-3;
    robot_params.ID1 = 1.067e-3;
    robot_params.k1 = 1.0 / 63.5e-3;

    // Tube 2 geometry
    robot_params.L2 = 163e-3;
    robot_params.Lt2 = robot_params.L2 - 38.2e-3;
    robot_params.OD2 = 2.0574e-3;
    robot_params.ID2 = 1.6002e-3;
    robot_params.k2 = 1.0 / 51.2e-3;

    // Tube 3 geometry
    robot_params.L3 = 104.4e-3;
    robot_params.Lt3 = robot_params.L3 - 21.4e-3;
    robot_params.OD3 = 2.540e-3;
    robot_params.ID3 = 2.2479e-3;
    robot_params.k3 = 1.0 / 71.4e-3;

    // Initialize joint positions
    robot_params.qHome << 0, 0, 0, -160e-3, -127.2e-3, -86.4e-3;

    // Starting configuration
    auto cannula = CTR3Robot::createCannula3(robot_params);

    CTR3Robot ctr_robot(cannula);
    ctr_robot.init(robot_params);
    CTR3RobotViz robot_viz;
    robot_viz.init("ctr_namespace");

    // Initialize subscriber
    JointListener joint_listener;
    joint_listener.init(robot_params.qHome);
    ros::Subscriber sub = node.subscribe<sensor_msgs::JointState>("/ctm/joint_state",1,&JointListener::callback, &joint_listener);

    // Set ros rate
    double rosLoopRate = 200.0;
    ros::Rate ra(rosLoopRate);

    while (ros::ok())
    {
        // update joints from callback
        ctr_robot.SetCurrQVec(joint_listener.q);

        medlab::CTR3KinematicsInputVector new_kinematic_input_vector;
        new_kinematic_input_vector.PsiL = ctr_robot.GetCurrQVec().head(3);
        new_kinematic_input_vector.Beta = ctr_robot.GetCurrQVec().tail(3);
        new_kinematic_input_vector.Ftip = Eigen::Vector3d::Zero();
        new_kinematic_input_vector.Ttip = Eigen::Vector3d::Zero();

        // Compute kinematics
        ctr_robot.callKinematicsWithDenseOutput(new_kinematic_input_vector);

         // Publish
         robot_viz.updateViz(ctr_robot);

         ros::spinOnce();
         ra.sleep();
    }
}

