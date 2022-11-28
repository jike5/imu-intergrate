# imu integrate

This repo is for integrate imu msgdata to get odometry.

## How to use

The ros node will subscribe "/imu" topic, and publish "predicted_odometry" topic.


```bash
catkin_make im_ws
source devel/setup.bash
rosrun imu_integrate estimator_node
```
