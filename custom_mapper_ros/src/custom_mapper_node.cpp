#include "custom_mapper_ros/custom_mapper_ros.h"
#include <ros/ros.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "custom_mapper_ros");
    custom_mapper_ros::CustomMapperRos custom_mapper_node;
    ROS_INFO_STREAM("[main] start loop");
    custom_mapper_node.run();

    return 0;
}


