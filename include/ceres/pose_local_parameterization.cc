//
// Created by lixin04 on 19年11月22日.
//

#include "pose_local_parameterization.h"

template <typename Derived>
static Eigen::Quaternion<typename Derived::Scalar> deltaQ(
    const Eigen::MatrixBase<Derived>& theta) {
  typedef typename Derived::Scalar Scalar_t;

  Eigen::Quaternion<Scalar_t> dq;
  Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
  half_theta /= static_cast<Scalar_t>(2.0);
  dq.w() = static_cast<Scalar_t>(1.0);
  dq.x() = half_theta.x();
  dq.y() = half_theta.y();
  dq.z() = half_theta.z();
  return dq;
}

static Eigen::Matrix3f skew_symmetric(Eigen::Vector3f v) {
    Eigen::Matrix3f S;
    S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
    return S;
}

bool LocalQuaternParameterization::Plus(const double* x,
                                        const double* delta,
                                        double* x_plus_delta) const {
  Eigen::Map<const Eigen::Quaterniond> _q(x);

  Eigen::Quaterniond dq = deltaQ(Eigen::Map<const Eigen::Vector3d>(delta));

  Eigen::Map<Eigen::Quaterniond> q(x_plus_delta);

  q = (_q * dq).normalized();
  return true;
}

bool LocalQuaternParameterization::ComputeJacobian(const double* x,
                                                   double* jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> j(jacobian);
  j.topRows<3>().setIdentity();
  j.bottomRows<1>().setZero();

  return true;
}

bool PoseLocalParameterization::Plus(const double* x,
                                     const double* delta,
                                     double* x_plus_delta) const {
  Eigen::Map<const Eigen::Vector3d> _p(x);
  Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

  Eigen::Map<const Eigen::Vector3d> dp(delta);

  Eigen::Quaterniond dq = deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

  Eigen::Map<Eigen::Vector3d> p(x_plus_delta);  // double 指针 转为eigen数组
  Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

  p = _p + dp;
  q = (_q * dq).normalized();

  return true;
}
bool PoseLocalParameterization::ComputeJacobian(const double* x,
                                                double* jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
  j.topRows<6>().setIdentity();
  j.bottomRows<1>().setZero();

  return true;
}

bool PoseLocalEularParameterization::Plus(const double* x,
                                          const double* delta,
                                          double* x_plus_delta) const {
  Eigen::Map<const Eigen::Vector3d> _p(x);
  Eigen::Map<const Eigen::Vector3d> _theta(x + 3);

  double s1 = sin(_theta[0]);
  double c1 = cos(_theta[0]);
  double s2 = sin(_theta[1]);
  double c2 = cos(_theta[1]);
  double s3 = sin(_theta[2]);
  double c3 = cos(_theta[2]);
  Eigen::Matrix3d _R;
  _R << c2 * c3, s1 * s2 * c3 - c1 * s3, c1 * s2 * c3 + s1 * s3, c2 * s3,
      s1 * s2 * s3 + c1 * c3, c1 * s2 * s3 - s1 * c3, -s2, s1 * c2, c1 * c2;

  Eigen::Map<const Eigen::Vector3d> dp(delta);
  Eigen::Map<const Eigen::Vector3d> delta_theta(delta + 3);
  Eigen::Matrix3d Rz;
  Rz << cos(delta_theta(2)), -sin(delta_theta(2)), 0, sin(delta_theta(2)),
      cos(delta_theta(2)), 0, 0, 0, 1;

  Eigen::Matrix3d Ry;
  Ry << cos(delta_theta(1)), 0., sin(delta_theta(1)), 0., 1., 0.,
      -sin(delta_theta(1)), 0., cos(delta_theta(1));

  Eigen::Matrix3d Rx;
  Rx << 1., 0., 0., 0., cos(delta_theta(0)), -sin(delta_theta(0)), 0.,
      sin(delta_theta(0)), cos(delta_theta(0));
  _R = _R * Rx * Ry * Rz;
  //    _R = _R * Rz * Ry * Rx;

  Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
  Eigen::Map<Eigen::Vector3d> theta(x_plus_delta + 3);

  p = _p + dp;
  Eigen::Vector3d u1 = _R.col(0);
  Eigen::Vector3d u2 = _R.col(1);
  Eigen::Vector3d u3 = _R.col(2);
  theta[0] = atan2(u2(2), u3(2));
  theta[1] = asin(-u1(2));
  theta[2] = atan2(u1(1), u1(0));

  return true;
}
bool PoseLocalEularParameterization::ComputeJacobian(const double* x,
                                                     double* jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> j(jacobian);
  j.setIdentity();

  return true;
}

Eigen::Matrix3d PointProjectionFactor::sqrt_info;
double PointProjectionFactor::sum_t;

PointProjectionFactor::PointProjectionFactor(const Eigen::Vector3d &_obs_i)
    : obs_i(_obs_i){};

/*
  parameters[0]:  Twc
  parameters[1]:  Point3d
*/
bool PointProjectionFactor::Evaluate(double const *const *parameters,
                                     double *residuals,
                                     double **jacobians) const
{
  Eigen::Vector3d twc(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond qwc(parameters[0][6], parameters[0][3], parameters[0][4],
                         parameters[0][5]);

  Eigen::Vector3d point_3d(parameters[1][0], parameters[1][1],
                           parameters[1][2]);

  Eigen::Matrix3d Rwc(qwc);
  Eigen::Vector3d point_c = qwc.inverse() * (point_3d - twc);

  Eigen::Map<Eigen::Vector2d> residual(residuals);

  residual = point_c - obs_i;

  sqrt_info.setIdentity();  // FIXME: check
  residual = sqrt_info * residual;

  if (jacobians)
  {
    Eigen::Matrix<double, 2, 3> reduce(2, 3);

    reduce << 1. / point_c(2), 0, -point_c(0) / (point_c(2) * point_c(2)), 0,
        1. / point_c(2), -point_c(1) / (point_c(2) * point_c(2));

    if (jacobians[0])
    {
      // std::cout <<"jacobian_ex_pose"<<"\n";
      Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_pose(
          jacobians[0]);

      Eigen::Matrix<double, 3, 6> jaco_pintc_pose;
      jaco_pintc_pose.setZero();
      jaco_pintc_pose.block(0, 0, 3, 3) = -Rwc.inverse(); // Lc_t
      jaco_pintc_pose.block(0, 3, 3, 3) = skew_symmetric(point_3d - twc);

      jacobian_pose.leftCols<6>() = reduce * jaco_pintc_pose;
      jacobian_pose.rightCols<1>().setZero();
    }
    if (jacobians[1])
    {
      Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_Point(
          jacobians[1]);

      jacobian_Point = reduce * Rwc.inverse();
    }
  }

  // check jacobian
  // std::cout << "---------- check jacobian ----------\n";
  // if (jacobians[0])
  //   std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(
  //                    jacobians[0])
  //             << std::endl
  //             << std::endl;
  // if (jacobians[1])
  //   std::cout << Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>>(
  //                    jacobians[1])
  //             << std::endl
  //             << std::endl;

  if (jacobians)
  {
    const double eps = 1e-6;
    Eigen::Matrix<double, 3, 9> num_jacobian;
    for (int k = 0; k < 9; k++)
    {
      Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
      Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4],
                            parameters[0][5]);

      Eigen::Vector3d point_3d(parameters[1][0], parameters[1][1],
                               parameters[1][2]);

      int a = k / 3, b = k % 3;
      Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

      if (a == 0)
        Pi += delta;
      else if (a == 1)
        Qi = Qi * deltaQ(delta);
      else if (a == 2)
        point_3d += delta;

      Eigen::Vector3d point_c = Qi.inverse() * (point_3d - Pi);

      double dep_j = point_c.z();

      Eigen::Vector3d tmp_residual;
      tmp_residual = point_c - obs_i; // 误差 = 坐标重投影误差
      tmp_residual = sqrt_info * tmp_residual;

      num_jacobian.col(k) = (tmp_residual - residual) / eps;
    }

    if (jacobians[0])
    {
      Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_pose(
          jacobians[0]);
      jacobian_pose.leftCols<6>() = num_jacobian.leftCols<6>();
      jacobian_pose.rightCols<1>().setZero();
    }
    if (jacobians[1])
    {
      Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_point(jacobians[1]);
      jacobian_point = num_jacobian.rightCols<3>(); //.rightCols<2>();
    }
  }

  // std::cout << "num_jacobian pose:\n"
  //           << num_jacobian.block(0, 0, 2, 6) << "\n"
  //           << std::endl;

  // std::cout << "num_jacobian point:\n"
  //           << num_jacobian.block(0, 6, 2, 3) << "\n"
  //           << std::endl;
  return true;
}