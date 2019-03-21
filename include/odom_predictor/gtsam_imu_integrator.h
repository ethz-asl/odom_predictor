//
// Created by mpantic on 20.03.19.
//
#ifndef ODOM_PREDICTOR_GTSAM_IMU_INTEGRATOR_H
#define ODOM_PREDICTOR_GTSAM_IMU_INTEGRATOR_H

#include <odom_predictor/imu_integrator.h>
#include <Eigen/Dense>
#include <kindr/minimal/quat-transformation.h>
#include <boost/shared_ptr.hpp>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/PreintegrationBase.h>

class GTSAMImuIntegrator : ImuIntegrator {
 public:
  GTSAMImuIntegrator() :
      kGravity(0.0, 0.0, -9.81) {

    // create default integrator
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>
        params = boost::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(kGravity);
    integrator_ = boost::make_shared<gtsam::PreintegratedCombinedMeasurements>(params);
  }

  void addMeasurement(const double dt,
                      const Vector3& imu_linear_acceleration,
                      const Vector3& imu_angular_velocity) {
    // luckily gtsam Vector3 and kindr Vector3 are the same typedef :-)
    integrator_->integrateMeasurement(imu_linear_acceleration,
                                      imu_angular_velocity,
                                      dt);

  }

  void getState(Transformation* transform,
                Vector3* linear_velocity,
                Vector3* angular_velocity) {
    // predict new pose
    gtsam::NavState predicted_pose = integrator_->predict(start_pose_, biases_);

    // convert to output formats
    *linear_velocity = predicted_pose.v();
    *angular_velocity = angular_velocity_;
    transform->getPosition() = predicted_pose.t();
    transform->getRotation() = Rotation(transform->getEigenQuaternion());
  }

  void resetState(const Transformation& transform,
                  const Vector3& linear_velocity,
                  const Vector3& angular_velocity) {
    // set navstate
    start_pose_ = gtsam::NavState::Create(gtsam::Rot3(transform.getRotation().getRotationMatrix()),
                                          transform.getPosition(),
                                          linear_velocity,
                                          boost::none,
                                          boost::none,
                                          boost::none);
    angular_velocity_ = angular_velocity;
  }

  void resetBias(const Vector3& imu_angular_velocity_bias,
                 const Vector3& imu_linear_acceleration_bias) {
    // set biases
    biases_ = gtsam::imuBias::ConstantBias(imu_linear_acceleration_bias,
                                           imu_angular_velocity_bias);
  }

 private:
  const gtsam::Vector3 kGravity;
  gtsam::Vector3 angular_velocity_;
  gtsam::NavState start_pose_;
  gtsam::imuBias::ConstantBias biases_;
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements> integrator_;
};

#endif //ODOM_PREDICTOR_GTSAM_IMU_INTEGRATOR_H
