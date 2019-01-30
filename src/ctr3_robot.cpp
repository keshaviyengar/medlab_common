#include <medlab_common/ctr3_robot.h>
#include <medlab_common/robotics_math.h>
#include <medlab_common/medlab_types.h>
#include <medlab_common/spline.h>

#include <vector>
#include <tuple>
#include <memory>
#include <cmath>
#include <utility>
#include <eigen3/Eigen/Dense>

#include <Options.h>
#include <Utility.h>
#include <Kinematics.h>
#include <BasicFunctions.h>
#include <Tube.h>

//#include <ros/console.h>

medlab::Cannula3 CTR3Robot::createCannula3(medlab::CTR3RobotParams params)
{
  // Curvature of each tube
  CTR::Functions::constant_fun< CTR::Vector<2>::type > k_fun1( (params.k1)*Eigen::Vector2d::UnitX() );
  CTR::Functions::constant_fun< CTR::Vector<2>::type > k_fun2( (params.k2)*Eigen::Vector2d::UnitX() );
  CTR::Functions::constant_fun< CTR::Vector<2>::type > k_fun3( (params.k3)*Eigen::Vector2d::UnitX() );

  // Define tubes
  medlab::TubeType T1 = CTR::make_annular_tube( params.L1, params.Lt1, params.OD1, params.ID1, k_fun1, params.E, params.G );
  medlab::TubeType T2 = CTR::make_annular_tube( params.L2, params.Lt2, params.OD2, params.ID2, k_fun2, params.E, params.G );
  medlab::TubeType T3 = CTR::make_annular_tube( params.L3, params.Lt3, params.OD3, params.ID3, k_fun3, params.E, params.G );

  // Assemble cannula
  medlab::Cannula3 cannula = std::make_tuple( T1, T2, T3 );

  return cannula;
}

CTR3Robot::CTR3Robot(medlab::Cannula3 cannula):
  WStability(RoboticsMath::Matrix6d::Identity()),
//  BaseFrame_WORLD(Eigen::Matrix4d::Identity()),
  cannula_(cannula),
//  qHome_(RoboticsMath::Vector6d::Zero()),
  nInterp_(250)
{
}

CTR3Robot::~CTR3Robot()
{
}

bool CTR3Robot::init(medlab::CTR3RobotParams params)
{
  currCannulaParams_ = params;

  currKinematicsInputVector_.PsiL = params.qHome.head(3);
  currKinematicsInputVector_.Beta = params.qHome.tail(3);
  currKinematicsInputVector_.Ftip = Eigen::Vector3d::Zero();
  currKinematicsInputVector_.Ttip = Eigen::Vector3d::Zero();
  currQVec_ = params.qHome;

  callKinematicsWithDenseOutput(currKinematicsInputVector_); // currInterpolatedBackbone_ set in here

  return true;
}

//bool CTR3Robot::init(medlab::CTR3RobotParams params, RoboticsMath::Vector6d qHome, Eigen::Matrix4d baseFrame)
//{
//  currCannulaParams_ = params;
//  qHome_ = qHome;
//  BaseFrame_WORLD = baseFrame;

//  currKinematicsInputVector_.PsiL = qHome_.head(3);
//  currKinematicsInputVector_.Beta = qHome_.tail(3);
//  currKinematicsInputVector_.Ftip = Eigen::Vector3d::Zero();
//  currKinematicsInputVector_.Ttip = Eigen::Vector3d::Zero();
//  currQVec_ = qHome_;

//  callKinematicsWithDenseOutput(currKinematicsInputVector_); // currInterpolatedBackbone_ set in here

//  return true;
//}

RoboticsMath::Vector6d CTR3Robot::GetQRelative()
{
  RoboticsMath::Vector6d qRelative;
  qRelative.topRows(3) = currKinematics.Alpha;
//  qRelative.bottomRows(3) = currQVec_.bottomRows(3) - GetQHome().bottomRows(3);

  qRelative[5] = currQVec_[5] - GetQHome()[5];
  qRelative[4] = currQVec_[4] - GetQHome()[4] - qRelative[5];
  qRelative[3] = currQVec_[3] - GetQHome()[3] - qRelative[5];
//  std::cout << "QVec[4] = " << currQVec_[4] << ", QRelative[4] = " << qRelative[4] << std::endl;
//  std::cout << "QVec = " << currQVec_.bottomRows(3).transpose() << std::endl;

  return qRelative;
}

void CTR3Robot::callKinematicsWithDenseOutput(medlab::CTR3KinematicsInputVector newKinematicsInput)
{
  CTR::KinRetDense< CTR::State< std::tuple_size<medlab::Cannula3>::type::value, medlab::OType >> ret1 = CTR::Kinematics_with_dense_output(cannula_, newKinematicsInput, medlab::OType());

  RoboticsMath::Matrix6d Jbody;
  Jbody = CTR::GetTipJacobianForTube1(ret1.y_final);

  double Stability;
  Stability = CTR::GetStability(ret1.y_final);
//  ROS_INFO_STREAM("Stability: " << Stability);
  ////////////////////
  /// Transform To Base Frame
  ////////////////////
//  poseData = CTR3Robot::transformToBaseFrame(ret1);

    // Pick out arc length points
    nPts_ = ret1.arc_length_points.size();

//    ROS_INFO_STREAM("nPts_ = " << nPts_);

    double* ptr = &ret1.arc_length_points[0];
    Eigen::Map<Eigen::VectorXd> s(ptr, nPts_);

    double zeroIndex;
    Eigen::VectorXd zeroIndexVec;
    Eigen::Vector3d pZero;
    Eigen::Vector4d qZero;
    Eigen::Matrix4d gZero;
    Eigen::Matrix4d gStarZero;
    Eigen::Matrix4d gStarL;

    int count = 0;
    for (int i=0; i < s.size(); ++i)
    {
      if (std::abs(s(i)) < 1.0e-7) {
        zeroIndexVec.resize(count+1);
        zeroIndexVec(count) = (double) i;
        count ++;
      }
    }

    zeroIndex = zeroIndexVec(count-1);

    pZero = ret1.dense_state_output.at( zeroIndex ).p;
    qZero = ret1.dense_state_output.at( zeroIndex ).q;
    gZero = RoboticsMath::assembleTransformation(RoboticsMath::quat2rotm(qZero),pZero);
    gStarZero = RoboticsMath::assembleTransformation(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
    gStarL = gStarZero*RoboticsMath::inverseTransform(gZero);

    Eigen::Matrix<double, 7, Eigen::Dynamic> poseData;
    poseData.resize(7, nPts_);
    for (int i =0; i < nPts_; ++i)
    {
      Eigen::Vector3d pi = ret1.dense_state_output.at( i ).p;
      Eigen::Vector4d qi = ret1.dense_state_output.at( i ).q;
      Eigen::Matrix4d gi = RoboticsMath::assembleTransformation(RoboticsMath::quat2rotm(qi),pi);
      Eigen::Matrix4d gStari = gStarL*gi;
      RoboticsMath::Vector7d xi;
      xi = RoboticsMath::collapseTransform(gStari);
      poseData.col(i) = xi;
    }

//  // Pick out arc length points
//  int nPts_ = ret1.arc_length_points.size();
//  double* ptr = &ret1.arc_length_points[0];
//  Eigen::Map<Eigen::VectorXd> s_temp(ptr, nPts_);
//  Eigen::VectorXd s;
//  s.resize(nPts_);
//  s = s_temp;

//  ROS_INFO_STREAM("nPts_ = " << nPts_);
//  ROS_INFO_STREAM("poseData is [" << poseData.rows() << " x " << poseData.cols() << "]");
//  ROS_INFO_STREAM("s is [" << s.rows() << " x " << s.cols() << "]");
  interpolateBackbone(s, poseData);

  /////////////////////
  /////////////////////

  Eigen::Vector3d pTip;
  Eigen::Vector4d qBishop;
  Eigen::Matrix3d RBishop;
  Eigen::Matrix3d Rtip;
  Eigen::Vector4d qTip;

  pTip = ret1.pTip;
  qBishop = ret1.qTip;

//  ROS_INFO_STREAM("qBishop = " << qBishop(0) <<  ", " << qBishop(1) << ", " << qBishop(2) << ", " << qBishop(3));

  RBishop = RoboticsMath::quat2rotm(qBishop);
  Eigen::Matrix3d rotate_psiL = Eigen::Matrix3d::Identity();
  rotate_psiL(0,0) = cos(newKinematicsInput.PsiL(0));
  rotate_psiL(0,1) = -sin(newKinematicsInput.PsiL(0));
  rotate_psiL(1,0) = sin(newKinematicsInput.PsiL(0));
  rotate_psiL(1,1) = cos(newKinematicsInput.PsiL(0));
  Rtip = RBishop*rotate_psiL;
  qTip = RoboticsMath::rotm2quat(Rtip);

  RoboticsMath::Matrix6d Jhybrid = RoboticsMath::Matrix6d::Zero();
  RoboticsMath::Matrix6d Rr = RoboticsMath::Matrix6d::Identity();
  Rr.topLeftCorner(3,3) = RBishop;
  Rr.bottomRightCorner(3,3) = RBishop;
  Jhybrid = Rr*Jbody;

  // Parse kinret into currKinematics_
  currKinematics.Ptip[0] = pTip[0];
  currKinematics.Ptip[1] = pTip[1];
  currKinematics.Ptip[2] = pTip[2];
  currKinematics.Qtip[0] = qTip[0];
  currKinematics.Qtip[1] = qTip[1];
  currKinematics.Qtip[2] = qTip[2];
  currKinematics.Qtip[3] = qTip[3];
  currKinematics.Alpha[0] = ret1.y_final.Psi[0];
  currKinematics.Alpha[1] = ret1.y_final.Psi[1];
  currKinematics.Alpha[2] = ret1.y_final.Psi[2];
  currKinematics.Stability = Stability;
  currKinematics.Jbody = Jbody;
  currKinematics.Jhybrid = Jhybrid;  
}

//Eigen::MatrixXd CTR3Robot::transformToBaseFrame(CTR::KinRetDense<CTR::State<std::tuple_size<medlab::Cannula3>::type::value, medlab::OType> > kin)
//{

//  // Pick out arc length points
//  int nPts = kin.arc_length_points.size();
//  double* ptr = &kin.arc_length_points[0];
//  Eigen::Map<Eigen::VectorXd> s(ptr, nPts);

//  double zeroIndex;
//  Eigen::VectorXd zeroIndexVec;
//  Eigen::Vector3d pZero;
//  Eigen::Vector4d qZero;
//  Eigen::Matrix4d gZero;
//  Eigen::Matrix4d gStarZero;
//  Eigen::Matrix4d gStarL;

//  int count = 0;
//  for (int i=0; i < s.size(); ++i)
//  {
//    if (s(i) == 0) {
//      zeroIndexVec.resize(count+1);
//      zeroIndexVec(count) = (double) i;
//      count ++;
//    }
//  }

//  zeroIndex = zeroIndexVec(count-1);
//  pZero = kin.dense_state_output.at( zeroIndex ).p;
//  qZero = kin.dense_state_output.at( zeroIndex ).q;
//  gZero = RoboticsMath::assembleTransformation(RoboticsMath::quat2rotm(qZero),pZero);
//  gStarZero = RoboticsMath::assembleTransformation(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
//  gStarL = gStarZero*RoboticsMath::inverseTransform(gZero);

//  Eigen::MatrixXd poseData(7,nPts);
//  for (int i =0; i < nPts; ++i)
//  {
//    Eigen::Vector3d pi = kin.dense_state_output.at( i ).p;
//    Eigen::Vector4d qi = kin.dense_state_output.at( i ).q;
//    Eigen::Matrix4d gi = RoboticsMath::assembleTransformation(RoboticsMath::quat2rotm(qi),pi);
//    Eigen::Matrix4d gStari = gStarL*gi;
//    RoboticsMath::Vector7d xi;
//    xi = RoboticsMath::collapseTransform(gStari);
//    poseData.col(i) = xi;
//  }

//  return poseData;
//}

void CTR3Robot::interpolateBackbone(Eigen::VectorXd sRef, Eigen::MatrixXd poseDataRef)
{
//  Eigen::Matrix<double, 4, Eigen::Dynamic> qRef;
  Eigen::Matrix<double, 4, Eigen::Dynamic, 0, 4, 50> qRef;

  qRef = poseDataRef.bottomRows(4);

//  ROS_INFO_STREAM("qRef is [" << qRef.rows() << " x " << qRef.cols() << "]");
//  ROS_INFO_STREAM("sRef is " << sRef.size());

  // Create a zero to one list for ref arc lengths
  int nRef = sRef.size();
  double totalArcLength = sRef(0) - sRef(nRef - 1);
  Eigen::VectorXd sRef0Vec(nRef);
  sRef0Vec.fill(sRef(nRef-1));
  Eigen::VectorXd zeroToOne = (1 / totalArcLength)*(sRef - sRef0Vec);

  // Create a zero to one vector including ref arc lengths & interp arc lengths (evenly spaced)
  int nPtsTotal = nInterp_ + nRef;

  Eigen::VectorXd xxLinspace(nInterp_);

  xxLinspace.fill(0.0);
  xxLinspace.setLinSpaced(nInterp_, 1.0, 0.0);
  Eigen::VectorXd xxUnsorted(nPtsTotal);
  xxUnsorted << xxLinspace, zeroToOne;
  std::sort(xxUnsorted.data(), xxUnsorted.data() + xxUnsorted.size());
  Eigen::VectorXd xx = xxUnsorted.reverse(); //Rich's interpolation functions call for descending order

  // List of return arc lengths in the original scaling/offset
  Eigen::VectorXd xxSRef0Vec(nPtsTotal);
  xxSRef0Vec.fill(sRef(nRef-1));
  Eigen::VectorXd sInterp = totalArcLength*xx + xxSRef0Vec;

  // Interpolate to find list of return quaternions
  Eigen::MatrixXd qInterp = RoboticsMath::quatInterp(qRef, zeroToOne, xx);

  // Interpolate to find list of return positions
  Eigen::VectorXd sInterpSpline = sInterp.reverse(); // spline requires ascending order

  std::vector<double> sVec;
  sVec.resize(sRef.size());
  Eigen::VectorXd::Map(&sVec[0], sRef.size()) = sRef.reverse();

  Eigen::VectorXd x = poseDataRef.row(0).reverse(); // interp x
  std::vector<double> xVec;
  xVec.resize(x.size());
  Eigen::VectorXd::Map(&xVec[0], x.size()) = x;
  tk::spline Sx;
  Sx.set_points(sVec, xVec);
  Eigen::VectorXd xInterp(nPtsTotal);

  xInterp.fill(0);
  for (int i = 0; i < nPtsTotal; i++)
  {
    xInterp(i) = Sx(sInterpSpline(i));
  }
  xInterp = xInterp.reverse().eval();

  Eigen::VectorXd y = poseDataRef.row(1).reverse(); // interp y
  std::vector<double> yVec;
  yVec.resize(y.size());
  Eigen::VectorXd::Map(&yVec[0], y.size()) = y;
  tk::spline Sy;
  Sy.set_points(sVec, yVec);
  Eigen::VectorXd yInterp(nPtsTotal);
  yInterp.fill(0);
  for (int i = 0; i < nPtsTotal; i++)
  {
    yInterp(i) = Sy(sInterpSpline(i));
  }
  yInterp = yInterp.reverse().eval();

  Eigen::VectorXd z = poseDataRef.row(2).reverse(); // interp z
  std::vector<double> zVec;
  zVec.resize(z.size());
  Eigen::VectorXd::Map(&zVec[0], z.size()) = z;
  tk::spline Sz;
  Sz.set_points(sVec, zVec);
  Eigen::VectorXd zInterp(nPtsTotal);
  for (int i = 0; i < nPtsTotal; i++)
  {
    zInterp(i) = Sz(sInterpSpline(i));
  }
  zInterp = zInterp.reverse().eval();

  Eigen::MatrixXd pInterp(3, nPtsTotal);
  pInterp.fill(0);
  pInterp.row(0) = xInterp.transpose();
  pInterp.row(1) = yInterp.transpose();
  pInterp.row(2) = zInterp.transpose();

  medlab::InterpRet interpResults;
  interpResults.s = sInterp;
  interpResults.p = pInterp;
  interpResults.q = qInterp;

  currInterpolatedBackbone_ = interpResults;
}

void CTR3Robot::computeStabilityWeightingMatrix(double sThreshold, double alphaS)
{
  // computes WStability and vS

  WStability = (exp(1.0 / (currKinematics.Stability - sThreshold)) - 1.0) * RoboticsMath::Matrix6d::Identity();

  RoboticsMath::Vector6d dSdq = RoboticsMath::Vector6d::Zero();

  double rotationalStep = 0.05*M_PI / 180.0;
  double translationalStep = 1.0E-5;

  RoboticsMath::Vector6d qFD;
  qFD.block<3,1>(0,0) = currQVec_.block<3,1>(0,0);
  qFD.block<3,1>(3,0) = currQVec_.block<3,1>(3,0);

  medlab::CTR3KinematicsInputVector qKinematicsUpper;
  medlab::CTR3KinematicsInputVector qKinematicsLower;

  for (int i=0; i < 6; i++)
  {
    RoboticsMath::Vector6d direction = RoboticsMath::Vector6d::Zero();
//    direction(i) = 1.0;

    double step = (i<3) ? rotationalStep : translationalStep;
    direction(i) = step;

//    if (i < 3)
//    {
//      step = rotationalStep;
//    }
//    else
//    {
//      step = translationalStep;
//    }

//    RoboticsMath::Vector6d qFDUpper = qFD + step*direction;
//    RoboticsMath::Vector6d qFDLower = qFD - step*direction;

    RoboticsMath::Vector6d qFDUpper = qFD + direction;
    RoboticsMath::Vector6d qFDLower = qFD - direction;

    qKinematicsUpper.PsiL = qFDUpper.block<3,1>(0,0);
    qKinematicsUpper.Beta = qFDUpper.block<3,1>(3,0);
    qKinematicsUpper.Ftip << 0.0, 0.0, 0.0;
    qKinematicsUpper.Ttip << 0.0, 0.0, 0.0;

    qKinematicsLower.PsiL = qFDLower.block<3,1>(0,0);
    qKinematicsLower.Beta = qFDLower.block<3,1>(3,0);
    qKinematicsLower.Ftip << 0.0, 0.0, 0.0;
    qKinematicsLower.Ttip << 0.0, 0.0, 0.0;

    auto retUpper = CTR::Kinematics_with_dense_output(cannula_, qKinematicsUpper, medlab::OType());
    auto retLower = CTR::Kinematics_with_dense_output(cannula_, qKinematicsLower, medlab::OType());

    double SUpper = CTR::GetStability(retUpper.y_final);
    double SLower = CTR::GetStability(retLower.y_final);

    dSdq(i) = (SUpper - SLower) / (2*step);

  }

  vS = alphaS*dSdq;
}
