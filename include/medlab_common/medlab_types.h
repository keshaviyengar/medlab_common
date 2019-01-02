#pragma once

#include <eigen3/Eigen/Dense>
//#include <Eigen/Dense>
#include <vector>
#include <tuple>

#include <Tube.h>
#include <BasicFunctions.h>
#include <Options.h>
#include <Utility.h>
#include "spline.h"
#include "robotics_math.h"

namespace medlab
{
using CurvFun  = CTR::Functions::constant_fun< Eigen::Vector2d >;
using TubeType = CTR::Tube< CurvFun >;
using Cannula3 = std::tuple<TubeType, TubeType, TubeType>; // CTR3Robot architecture
using OType = CTR::DeclareOptions < CTR::Option::ComputeJacobian,
                                    CTR::Option::ComputeGeometry,
                                    CTR::Option::ComputeStability,
                                    CTR::Option::ComputeCompliance >::options;

struct CTR3RobotParams {

  // Material Properties
  double E;
  double G;

  // Tube 1 Geometry
  double L1;  // [m]   total tube length
  double Lt1; // [m]   straight length
  double k1;  // [1/m] curvature
  double OD1; // [m]   outer diameter
  double ID1; // [m]   inner diameter

  // Tube 2 Geometry
  double L2;
  double Lt2;
  double k2;
  double OD2;
  double ID2;

  // Tube 3 Geometry
  double L3;
  double Lt3;
  double k3;
  double OD3;
  double ID3;
};

struct CTR3KinematicsInputVector { // format of the input vector fed into Kinematics_with_dense_output()
  Eigen::Vector3d PsiL;
  Eigen::Vector3d Beta;
  Eigen::Vector3d Ftip;
  Eigen::Vector3d Ttip;
};

struct InterpRet {      // Interpolated CTR3 Backbone
  Eigen::VectorXd s;
  Eigen::MatrixXd p;
  Eigen::MatrixXd q;
};

struct WeightingRet {
  RoboticsMath::Matrix6d W;
  Eigen::VectorXd dh;
};

struct KinOut {
  Eigen::Vector3d Ptip;
  Eigen::Vector4d Qtip;
  Eigen::Vector3d Alpha;
  RoboticsMath::Matrix6d Jbody;
  RoboticsMath::Matrix6d Jhybrid;
  double Stability;
};


}
