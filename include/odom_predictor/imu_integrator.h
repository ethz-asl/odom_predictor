#include <Eigen/Dense>
#include <kindr/minimal/quat-transformation.h>
#include <boost/shared_ptr.hpp>

typedef kindr::minimal::QuatTransformation Transformation;
typedef kindr::minimal::RotationQuaternion Rotation;
typedef Transformation::Vector3 Vector3;

#ifndef ODOM_PREDICTOR_IMU_INTEGRATOR_H
#define ODOM_PREDICTOR_IMU_INTEGRATOR_H

class ImuIntegrator {

 public:
  virtual void resetState(const Transformation& transform = Transformation(),
                          const Vector3& linear_velocity = Vector3::Zero(),
                          const Vector3& angular_velocity = Vector3::Zero()) = 0;

  virtual void resetBias(const Vector3& imu_angular_velocity_bias = Vector3::Zero(),
                         const Vector3& imu_linear_acceleration_bias = Vector3::Zero()) = 0;

  virtual void addMeasurement(const double dt,
                              const Vector3& imu_linear_acceleration,
                              const Vector3& imu_angular_velocity) = 0;

  virtual void getState(Transformation* transform,
                        Vector3* linear_velocity,
                        Vector3* angular_velocity) = 0;

};

class ZecImuIntegrator : ImuIntegrator {
 public:
  ZecImuIntegrator() :
      kGravity(0.0, 0.0, -9.81),
      linear_velocity_(Vector3::Zero()),
      angular_velocity_(Vector3::Zero()),
      imu_angular_velocity_bias_(Vector3::Zero()),
      imu_linear_acceleration_bias_(Vector3::Zero()) {

  }

  void addMeasurement(const double dt,
                      const Vector3& imu_linear_acceleration,
                      const Vector3& imu_angular_velocity) {

    // find changes in angular velocity and rotation delta
    const Vector3 final_angular_velocity =
        (imu_angular_velocity - imu_angular_velocity_bias_);
    const Vector3 delta_angle =
        dt * (final_angular_velocity + angular_velocity_) / 2.0;
    angular_velocity_ = final_angular_velocity;

    // apply half of the rotation delta
    const Rotation half_delta_rotation = Rotation::exp(delta_angle / 2.0);
    transform_.getRotation() = transform_.getRotation() * half_delta_rotation;

    // find changes in linear velocity and position
    const Vector3 delta_linear_velocity =
        dt * (imu_linear_acceleration +
            transform_.getRotation().inverse().rotate(kGravity) -
            imu_linear_acceleration_bias_);
    transform_.getPosition() =
        transform_.getPosition() +
            transform_.getRotation().rotate(
                dt * (linear_velocity_ + delta_linear_velocity / 2.0));
    linear_velocity_ += delta_linear_velocity;

    // apply the other half of the rotation delta
    transform_.getRotation() = transform_.getRotation() * half_delta_rotation;
  }

  void getState(Transformation* transform,
                Vector3* linear_velocity,
                Vector3* angular_velocity) {
    *transform = transform_;
    *linear_velocity = linear_velocity_;
    *angular_velocity = angular_velocity_;
  }

  void resetState(const Transformation& transform,
                  const Vector3& linear_velocity,
                  const Vector3& angular_velocity) {
    transform_ = transform;
    linear_velocity_ = linear_velocity;
    angular_velocity_ = angular_velocity;
  }

  void resetBias(const Vector3& imu_angular_velocity_bias,
                 const Vector3& imu_linear_acceleration_bias) {
    imu_linear_acceleration_bias_ = imu_linear_acceleration_bias;
    imu_angular_velocity_bias_ = imu_angular_velocity_bias;
  }

 private:
  const Vector3 kGravity;
  Vector3 linear_velocity_;
  Vector3 angular_velocity_;
  Vector3 imu_angular_velocity_bias_;
  Vector3 imu_linear_acceleration_bias_;
  Transformation transform_;

};

#endif //ODOM_PREDICTOR_IMU_INTEGRATOR_H
