#pragma once

#include <eigen3/Eigen/Dense>
#include <medlab_common/robotics_math.h>
#include <medlab_common/medlab_types.h>
#include <memory>
#include <utility>

class CTR3Robot
{

public:
  static medlab::Cannula3 createCannula3(medlab::CTR3RobotParams params);

  CTR3Robot(medlab::Cannula3 cannula);
  ~CTR3Robot();
  bool init(medlab::CTR3RobotParams params);
//  bool init(medlab::CTR3RobotParams params, RoboticsMath::Vector6d qHome, Eigen::Matrix4d baseFrame);

//  medlab::Cannula3* GetCannula() {return cannula_;}
  medlab::CTR3RobotParams GetCurRobotParams() {return currCannulaParams_;}
  medlab::CTR3KinematicsInputVector GetCurrKinematicsInputVector() {return currKinematicsInputVector_;}
  RoboticsMath::Vector6d GetCurrQVec() {return currQVec_;}
  void SetCurrQVec(RoboticsMath::Vector6d qVec) {currQVec_ = qVec;}
  RoboticsMath::Vector6d GetQRelative();
  RoboticsMath::Vector6d GetQHome() {return currCannulaParams_.qHome;}
  medlab::InterpRet GetInterpolatedBackbone() {return currInterpolatedBackbone_;}
  int GetNPts() {return nPts_;}
  int GetNInterp() {return nInterp_;}

  void callKinematicsWithDenseOutput(medlab::CTR3KinematicsInputVector newKinematicInput);
//  medlab::KinOut callKinematicsWithDenseOutput(medlab::CTR3KinematicsInputVector newKinematicsInput);

//  Eigen::MatrixXd transformToBaseFrame(CTR::KinRetDense< CTR::State< std::tuple_size<medlab::Cannula3>::type::value, medlab::OType >> kinret);

  void interpolateBackbone(Eigen::VectorXd sRef, Eigen::MatrixXd poseDataRef);
  void computeStabilityWeightingMatrix(double sThreshold, double alphaS);
  medlab::KinOut currKinematics;  // kinout from kinematics call, transformed into base frame & interpolated
  RoboticsMath::Matrix6d WStability;
  RoboticsMath::Vector6d vS;
//  Eigen::Matrix4d BaseFrame_WORLD;

private:
  medlab::Cannula3 cannula_;  // cannula tuple object fed to Hunter's kinematics
  medlab::CTR3RobotParams currCannulaParams_; // params that define the cannula3
  medlab::CTR3KinematicsInputVector currKinematicsInputVector_; // full input vector fed to kinematics call
  RoboticsMath::Vector6d currQVec_; // condensed kinematics input vector [psiL, beta]
//  RoboticsMath::Vector6d qHome_; // home configuration (joint space [psiL, beta])
  medlab::InterpRet currInterpolatedBackbone_; // interpolated cannula [sxn,pxn,qxn]
  int nPts_; // number of points along arclength returned by kinematics
  int nInterp_; // number of points to interpolate on backbone
  Eigen::Matrix<double, 8, Eigen::Dynamic> markersOut_; // Matrix for storing marker output to rviz (used in endonasal_teleop)

};
