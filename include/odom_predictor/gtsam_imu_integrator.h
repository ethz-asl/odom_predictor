//
// Created by mpantic on 20.03.19.
//
#ifndef ODOM_PREDICTOR_GTSAM_IMU_INTEGRATOR_H
#define ODOM_PREDICTOR_GTSAM_IMU_INTEGRATOR_H

#include <odom_predictor/imu_integrator.h>
#include <Eigen/Dense>
#include <kindr/minimal/quat-transformation.h>
#include <boost/shared_ptr.hpp>

class GTSAMImuIntegrator : ImuIntegrator {
 public:
  GTSAMImuIntegrator() :
      kGravity(0.0, 0.0, -9.81) {
  }

  void addMeasurement(const double dt,
                      const Vector3& imu_linear_acceleration,
                      const Vector3& imu_angular_velocity) {

  }

  void getState(Transformation* transform,
                Vector3* linear_velocity,
                Vector3* angular_velocity) {

  }

  void resetState(const Transformation& transform,
                  const Vector3& linear_velocity,
                  const Vector3& angular_velocity) {

  }

  void resetBias(const Vector3& imu_angular_velocity_bias,
                 const Vector3& imu_linear_acceleration_bias) {

  }

 private:
  const Vector3 kGravity;
};

#endif //ODOM_PREDICTOR_GTSAM_IMU_INTEGRATOR_H
