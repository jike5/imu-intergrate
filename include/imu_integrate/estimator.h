#ifndef ESTIMATOR_H_
#define ESTIMATOR_H_

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <list>
#include <Eigen/Dense>
#include <Eigen/Core>

typedef Eigen::Matrix4d Transformation;
typedef Eigen::Matrix3d Rotation;
typedef Eigen::Vector3d Vector3;
typedef Eigen::Quaterniond Quaternion;

class Estimator
{
public:
    Estimator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    void imuCallback(const sensor_msgs::ImuConstPtr& msg);

private:
    void integrateIMUData(const sensor_msgs::Imu& msg);
    void publishOdometry();
    void publishTF();

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber imu_sub_;
    
    ros::Publisher odom_pub_;
    ros::Publisher transform_pub_;

    int max_imu_queue_length_;
    std::list<sensor_msgs::Imu> imu_queue_;

    int seq_;
    std::string frame_id_;
    std::string child_frame_id_;

    bool isInit_; // first imu frame
    ros::Time estimate_timestamp_; // last imu_msg time
    
    Vector3 linear_velocity_;  // last 
    Vector3 angular_velocity_; // last gyro
    Vector3 linear_acceleration_; // last gyro
    
    Transformation transform_;

};

#endif

