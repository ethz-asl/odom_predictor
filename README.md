# odom_predictor
A simple ROS node that integrates an IMU to predict future odometry readings

### Parameters
| Parameter | Description | Default |
| --------------------  |:-----------:| :-------:|
| `max_imu_queue_length` |  Maximum number of IMU measurements to store, this limits how far foward in time the system can predict. | 1000 |
