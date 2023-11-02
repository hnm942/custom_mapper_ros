#ifndef CUSTOM_MAPPER_ROS_H
#define CUSTOM_MAPPER_ROS_H

#include <memory>
#include <string>
#include <vector>
#include <cv_bridge/cv_bridge.h>


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>


namespace custom_mapper_ros
{
    enum ProcessArgs
    {
        NOMINAL = 0,
        NO_POSE = 1,
        NO_GOAL = 2,
        NO_DEPTH_IMAGE = 3
    };

    class CustomMapperRos
    {
    public:
        CustomMapperRos();
        void run();

    private:
        void getParams();
        void initSubcribers();
        void initPublishers();
        void depthImageCallback(const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg);
        void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_ptr);
        // variable
        tf2_ros::Buffer tf_buffer_;
        image_transport::CameraSubscriber depth_sub_;
        ros::Subscriber odom_sub_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;
        std::unique_ptr<image_transport::ImageTransport> it_ptr_; 
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        bool got_depth_image_;
    };
}

#endif