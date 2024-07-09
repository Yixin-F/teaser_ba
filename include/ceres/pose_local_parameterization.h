#pragma once

#include <ceres/ceres.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "../Utility.h"

class PoseLocalParameterization : public ceres::LocalParameterization {
  virtual bool Plus(const double* x,
                    const double* delta,
                    double* x_plus_delta) const;
  virtual bool ComputeJacobian(const double* x, double* jacobian) const;
  virtual int GlobalSize() const { return 7; };
  virtual int LocalSize() const { return 6; };
};

class LocalQuaternParameterization : public ceres::LocalParameterization {
  virtual bool Plus(const double* x,
                    const double* delta,
                    double* x_plus_delta) const;
  virtual bool ComputeJacobian(const double* x, double* jacobian) const;
  virtual int GlobalSize() const { return 4; };
  virtual int LocalSize() const { return 3; };
};

class PoseLocalEularParameterization : public ceres::LocalParameterization {
  virtual bool Plus(const double* x,
                    const double* delta,
                    double* x_plus_delta) const;
  virtual bool ComputeJacobian(const double* x, double* jacobian) const;
  virtual int GlobalSize() const { return 6; };
  virtual int LocalSize() const { return 6; };
};

class PointProjectionFactor : public ceres::SizedCostFunction<3, 7, 3> {
  public:
    PointProjectionFactor(const Eigen::Vector3d &_pts_i);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector3d obs_i;
    static Eigen::Matrix3d sqrt_info;
    static double sum_t;
};
