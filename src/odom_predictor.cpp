#include "odom_predictor/odom_predictor.h"

OdomPredictor::OdomPredictor(const ros::NodeHandle& nh,
                             const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      seq_(0),
      have_odom_(false),
      have_bias_(false) {
  nh_private.param("max_imu_queue_length", max_imu_queue_length_, 1000);

  constexpr size_t kROSQueueLength = 100;
  imu_sub_ =
      nh_.subscribe("imu", kROSQueueLength, &OdomPredictor::imuCallback, this);
  imu_bias_sub_ = nh_.subscribe("imu_bias", kROSQueueLength,
                                &OdomPredictor::imuBiasCallback, this);
  odometry_sub_ = nh_.subscribe("odometry", kROSQueueLength,
                                &OdomPredictor::odometryCallback, this);

  odom_pub_ = nh_private_.advertise<nav_msgs::Odometry>("predicted_odometry",
                                                        kROSQueueLength);
  odom2_pub_ = nh_private_.advertise<nav_msgs::Odometry>("predicted_odometry_compare",
                                                         kROSQueueLength);

  transform_pub_ = nh_private_.advertise<geometry_msgs::TransformStamped>(
      "predicted_transform", kROSQueueLength);

  integrator_.reset((ImuIntegrator*) new ZecImuIntegrator());
  integrator2_.reset((ImuIntegrator*) new GTSAMImuIntegrator());
}

void OdomPredictor::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  if (!have_bias_) {
    return;
  }

  // clear old IMU measurements
  while (!imu_queue_.empty() &&
      imu_queue_.front().header.stamp < msg->header.stamp) {
    imu_queue_.pop_front();
  }

  // extract useful information from message
  Transformation transform;
  Vector3 linear_velocity, angular_velocity;
  tf::poseMsgToKindr(msg->pose.pose, &transform);
  tf::vectorMsgToKindr(msg->twist.twist.linear, &linear_velocity);
  tf::vectorMsgToKindr(msg->twist.twist.angular, &angular_velocity);
  // set integrator to this state
  integrator_->resetState(transform, linear_velocity, angular_velocity);
  integrator2_->resetState(transform, linear_velocity, angular_velocity);

  pose_covariance_ = msg->pose.covariance;
  twist_covariance_ = msg->twist.covariance;
  frame_id_ = msg->header.frame_id;
  child_frame_id_ = msg->child_frame_id;

  // reintegrate IMU messages
  estimate_timestamp_ = msg->header.stamp;

  try {
    for (const sensor_msgs::Imu& imu_msg : imu_queue_) {
      integrateIMUData(imu_msg);
    }
  } catch (std::exception& e) {
    ROS_ERROR_STREAM(
        "IMU INTEGRATION FAILED, RESETING EVERYTHING: " << e.what());
    have_bias_ = false;
    have_odom_ = false;
    imu_queue_.clear();
    return;
  }

  have_odom_ = true;
}

void OdomPredictor::imuCallback(const sensor_msgs::ImuConstPtr& msg) {
  if (msg->header.stamp < imu_queue_.back().header.stamp) {
    ROS_ERROR_STREAM("Latest IMU message occured at time: "
                         << msg->header.stamp
                         << ". This is before the previously received IMU "
                            "message that ouccured at: "
                         << imu_queue_.back().header.stamp
                         << ". The current imu queue will be reset.");
    imu_queue_.clear();
  }
  if (imu_queue_.size() > max_imu_queue_length_) {
    ROS_WARN_STREAM_THROTTLE(
        10, "There has been over "
        << max_imu_queue_length_
        << " IMU messages since the last odometry update. The oldest "
           "measurement will be forgotten. This message is printed "
           "once every 10 seconds");
    imu_queue_.pop_front();
  }

  imu_queue_.push_back(*msg);

  if (!have_bias_ || !have_odom_) {
    return;
  }

  try {
    integrateIMUData(*msg);
  } catch (std::exception& e) {
    ROS_ERROR_STREAM(
        "IMU INTEGRATION FAILED, RESETING EVERYTHING: " << e.what());
    have_bias_ = false;
    have_odom_ = false;
    imu_queue_.clear();
    return;
  }

  publishOdometry();
  publishTF();
  ++seq_;
}

void OdomPredictor::imuBiasCallback(const sensor_msgs::ImuConstPtr& msg) {
  Vector3 imu_linear_acceleration_bias, imu_angular_velocity_bias;
  tf::vectorMsgToKindr(msg->linear_acceleration,
                       &imu_linear_acceleration_bias);
  tf::vectorMsgToKindr(msg->angular_velocity, &imu_angular_velocity_bias);

  integrator_->resetBias(imu_angular_velocity_bias,
                         imu_linear_acceleration_bias);
  integrator2_->resetBias(imu_angular_velocity_bias,
                          imu_linear_acceleration_bias);

  have_bias_ = true;
}

void OdomPredictor::integrateIMUData(const sensor_msgs::Imu& msg) {
  const double delta_time = (msg.header.stamp - estimate_timestamp_).toSec();

  Vector3 imu_linear_acceleration, imu_angular_velocity;
  tf::vectorMsgToKindr(msg.linear_acceleration, &imu_linear_acceleration);
  tf::vectorMsgToKindr(msg.angular_velocity, &imu_angular_velocity);
  integrator_->addMeasurement(delta_time, imu_linear_acceleration,
                              imu_angular_velocity);
  integrator2_->addMeasurement(delta_time, imu_linear_acceleration,
                               imu_angular_velocity);

  estimate_timestamp_ = msg.header.stamp;
}

void OdomPredictor::publishOdometry() {
  if (!have_odom_) {
    return;
  }

  nav_msgs::Odometry msg;

  msg.header.frame_id = frame_id_;
  msg.header.seq = seq_;
  msg.header.stamp = estimate_timestamp_;
  msg.child_frame_id = child_frame_id_;

  Transformation transform;
  Vector3 linear_velocity, angular_velocity;
  integrator_->getState(&transform,
                        &linear_velocity,
                        &angular_velocity);

  tf::poseKindrToMsg(transform, &msg.pose.pose);
  msg.pose.covariance = pose_covariance_;

  tf::vectorKindrToMsg(linear_velocity, &msg.twist.twist.linear);
  tf::vectorKindrToMsg(angular_velocity, &msg.twist.twist.angular);
  msg.twist.covariance = twist_covariance_;
  odom_pub_.publish(msg);

  // Publish result of second integrator
  integrator2_->getState(&transform,
                         &linear_velocity,
                         &angular_velocity);
  tf::poseKindrToMsg(transform, &msg.pose.pose);
  tf::vectorKindrToMsg(linear_velocity, &msg.twist.twist.linear);
  tf::vectorKindrToMsg(angular_velocity, &msg.twist.twist.angular);
  odom2_pub_.publish(msg);
}

void OdomPredictor::publishTF() {
  if (!have_odom_) {
    return;
  }

  geometry_msgs::TransformStamped msg;

  msg.header.frame_id = frame_id_;
  msg.header.seq = seq_;
  msg.header.stamp = estimate_timestamp_;
  msg.child_frame_id = child_frame_id_;

  Transformation transform;
  Vector3 linear_velocity, angular_velocity;
  integrator_->getState(&transform,
                        &linear_velocity,
                        &angular_velocity);

  tf::transformKindrToMsg(transform, &msg.transform);

  transform_pub_.publish(msg);
  br_.sendTransform(msg);
}
