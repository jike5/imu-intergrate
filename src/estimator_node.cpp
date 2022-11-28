#include "ros/ros.h"
#include "imu_integrate/estimator.h"


int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"estimator_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    Estimator estimator(nh, nh_private);
    ros::spin();

    return 0;
}