#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <Eigen/Dense>
#include <vector>

namespace RoboticsMath 
{
  // define custom data types
  using Matrix6d = Eigen::Matrix<double, 6, 6>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Vector8d = Eigen::Matrix<double, 8, 1>;

  template <typename Derived1, typename Derived2>
  void SetRotation(Eigen::DenseBase<Derived1>& g, Eigen::DenseBase<Derived2>& R)
  {
      assert(g.rows() == 4);
      assert(g.cols() == 4);
      assert(R.rows() == 3);
      assert(R.cols() == 3);
      g.block(0,0,3,3) = R;
  }

  template <typename Derived>
  Eigen::MatrixXd GetRotation(Eigen::DenseBase<Derived> const& in)
  {
      Eigen::MatrixXd R = in.block(0,0,3,3);
      return R;
  }

  template <typename Derived>
  Eigen::MatrixXd GetTranslation(Eigen::DenseBase<Derived> const& in)
  {
      Eigen::MatrixXd p = in.block(0,3,3,1);
      return p;
  }

  template <typename Derived1, typename Derived2>
  void SetTranslation(Eigen::DenseBase<Derived1>& g, Eigen::DenseBase<Derived2>& p)
  {
      assert(g.rows() == 4);
      assert(g.cols() == 4);
      assert(p.rows() == 3);
      assert(p.cols() == 1);
      g.block(0,3,3,1) = p;
  }

  template <typename Derived>
  Eigen::MatrixXd Inverse(Eigen::DenseBase<Derived> const& g)
  {
      Eigen::MatrixXd ginv = Eigen::MatrixXd::Zero(4,4);
      ginv.block(0,0,3,3) = g.block(0,0,3,3).transpose(); //R^T
      ginv.block(0,3,3,1) = -g.block(0,0,3,3).transpose()*g.block(0,3,3,1); //-R^T*p
      ginv(3,3) = 1.0;
      return ginv;
  }

  double deg2rad(double degrees);
  double sgn(double x);
  double vectornorm(Eigen::Vector3d v);
  Eigen::Matrix3d orthonormalize(Eigen::Matrix3d R);
  Eigen::Matrix4d assembleTransformation(Eigen::Matrix3d Rot, Eigen::Vector3d Trans);
  RoboticsMath::Vector7d collapseTransform(Eigen::Matrix4d T);
  Eigen::Matrix3d quat2rotm(Eigen::Vector4d Quat);
  Eigen::Vector4d rotm2quat(Eigen::Matrix3d R);

  void getCofactor(double A[6][6], double temp[6][6], int p, int q, int n);
  double determinant(double A[6][6], int n);
  void adjoint(double A[6][6], double adj[6][6]);
  void inverse(double A[6][6], double inverse[6][6]);

  Eigen::Matrix3d hat3(Eigen::Vector3d v);
  RoboticsMath::Matrix6d Adjoint_p_q(Eigen::Vector3d p, Eigen::Vector4d q);
  Eigen::Matrix4d inverseTransform(Eigen::Matrix4d T);
  Eigen::Vector4d slerp(Eigen::Vector4d qa, Eigen::Vector4d qb, double t);
  Eigen::Matrix<double,4,Eigen::Dynamic> quatInterp(Eigen::Matrix<double, 4,Eigen::Dynamic> refQuat, Eigen::VectorXd refArcLengths, Eigen::VectorXd interpArcLengths);
}
