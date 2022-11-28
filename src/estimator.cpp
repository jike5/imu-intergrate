#include "imu_integrate/estimator.h"
#include <cmath>

Estimator::Estimator(const ros::NodeHandle& nh, 
                     const ros::NodeHandle& nh_private)
                     : nh_(nh), nh_private_(nh_private), isInit_(false)
{
    nh_private.param("max_imu_queue_length", max_imu_queue_length_, 1000);

    constexpr size_t kROSQueueLength = 100;
    imu_sub_ =
        nh_.subscribe("/imu", kROSQueueLength, &Estimator::imuCallback, this);

    odom_pub_ = nh_private_.advertise<nav_msgs::Odometry>("predicted_odometry",
                                                        kROSQueueLength);
                                                        
    transform_pub_ = nh_private_.advertise<geometry_msgs::TransformStamped>(
        "predicted_transform", kROSQueueLength);
}


void Estimator::imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
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
    try {
    integrateIMUData(*msg);
    } catch (std::exception& e) {
    ROS_ERROR_STREAM(
        "IMU INTEGRATION FAILED, RESETING EVERYTHING: " << e.what());
    imu_queue_.clear();
    return;
    }

    publishOdometry();
    // publishTF();
    ++seq_;

}

void Estimator::integrateIMUData(const sensor_msgs::Imu& msg)
{
    ROS_INFO("/n------------integrate imu data-------------");

    Vector3 imu_linear_acceleration, imu_angular_velocity;
    imu_linear_acceleration[0] = msg.linear_acceleration.x;
    imu_linear_acceleration[1] = msg.linear_acceleration.y;
    imu_linear_acceleration[2] = msg.linear_acceleration.z;
    imu_angular_velocity[0] = msg.angular_velocity.x;
    imu_angular_velocity[1] = msg.angular_velocity.y;
    imu_angular_velocity[2] = msg.angular_velocity.z;

    if (!isInit_)
    {
        ROS_INFO("Init frame");
        estimate_timestamp_ = msg.header.stamp;
        linear_velocity_ = Vector3::Zero();
        angular_velocity_ = imu_angular_velocity;
        linear_acceleration_ = imu_linear_acceleration;
        transform_ = Transformation::Identity();
        isInit_ = true;
        return;
    }

    const double delta_time = (msg.header.stamp - estimate_timestamp_).toSec();
    if(delta_time < 1e-5)
    {
        return;
    }
    Vector3 kGravity(0.0, 0.0, -9.81);
    Vector3 delta_angle = delta_time * (imu_angular_velocity + angular_velocity_) / 2.0;
    angular_velocity_ = imu_angular_velocity;

    ROS_INFO_STREAM(std::setprecision(5) << std::fixed << "delta_time = " << delta_time << "\ndelta_angle = " << delta_angle);

    double phi = delta_angle.norm();
    double sin_phi = std::sin(phi);
    double cos_phi = std::cos(phi);
    Rotation phi_skew = Rotation::Zero();
    //  0  -z   y
    //  z   0  -x
    // -y   x   0
    phi_skew(2,1) =  delta_angle[0];
    phi_skew(1,2) = -delta_angle[0];
    phi_skew(2,0) = -delta_angle[1];
    phi_skew(0,2) =  delta_angle[1];
    phi_skew(1,0) =  delta_angle[2];
    phi_skew(0,1) = -delta_angle[2];

    Rotation delta_rotation = Rotation::Identity() 
                        + sin_phi / phi * phi_skew 
                        + (1 - cos_phi) / pow(phi,2) * phi_skew * phi_skew;

            
    Rotation R_k_1 = transform_.block(0,0,3,3);
    transform_.block(0,0,3,3) = transform_.block(0,0,3,3) * delta_rotation;

    Rotation R_k = transform_.block(0,0,3,3);

    ROS_INFO_STREAM("phi = " << phi << "\nphi_skew = " << phi_skew << "\ndelta_rotation = " << delta_rotation << 
        "\nR_k_1 = " << R_k_1 << "\nR_k = " << R_k);

    Vector3 linear_velocity_cur = linear_velocity_ 
                    + ((R_k * imu_linear_acceleration + R_k_1 * linear_acceleration_) / 2.0 - kGravity) * delta_time;
    linear_acceleration_ = imu_linear_acceleration;

    ROS_INFO_STREAM("t_K_1 = " << transform_.block(0,3,3,1));
    transform_.block(0,3,3,1) = transform_.block(0,3,3,1) + (linear_velocity_cur + linear_velocity_) / 2.0 * delta_time;
    ROS_INFO_STREAM("t_K_1 = " << transform_.block(0,3,3,1));
    
    ROS_INFO_STREAM("\nlinear_acceleration = " << linear_acceleration_ << "\nlinear_velocity_k_1 = " << linear_velocity_ << 
        "\nlinear_velocity_k = " << linear_velocity_cur);

    linear_velocity_ = linear_velocity_cur;
    estimate_timestamp_ = msg.header.stamp;
}

void Estimator::publishOdometry()
{
    nav_msgs::Odometry msg;

    msg.header.frame_id = "world";
    msg.header.seq = seq_;
    msg.header.stamp = estimate_timestamp_;
    msg.child_frame_id = "est_local_ned";
    Rotation R = transform_.block(0,0,3,3);
    Quaternion q = Quaternion(R);
    msg.pose.pose.orientation.w = q.w();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.x = q.x();

    msg.pose.pose.position.x = transform_(0,3);
    msg.pose.pose.position.y = transform_(1,3);
    msg.pose.pose.position.z = transform_(2,3);
    // linear velocity
    msg.twist.twist.linear.x = linear_velocity_(0);
    msg.twist.twist.linear.y = linear_velocity_(1);
    msg.twist.twist.linear.z = linear_velocity_(2);
    // angular velocity
    msg.twist.twist.angular.x = angular_velocity_(0);
    msg.twist.twist.angular.y = angular_velocity_(1);
    msg.twist.twist.angular.z = angular_velocity_(2);

    odom_pub_.publish(msg);
    ROS_INFO("Publish Estimated Odometry");
}


void Estimator::publishTF()
{

}