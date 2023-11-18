#include "custom_mapper_ros/custom_mapper_ros.h"

namespace custom_mapper_ros
{
    CustomMapperRos::CustomMapperRos() : nh_(), pnh_("~"), got_depth_image_(false)
    {
        it_ptr_ = std::unique_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(pnh_));
        tf_listener_ptr_ = std::unique_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(tf_buffer_));
    }

    void CustomMapperRos::getParams()
    {
        
    }

    void CustomMapperRos::initSubcribers()
    {
        depth_sub_ = it_ptr_->subscribeCamera("depth_image_topic", 1, &CustomMapperRos::depthImageCallback, this);
        odom_sub_ = pnh_.subscribe("pose_topic", 1, &CustomMapperRos::odomCallback, this);
    }

    void CustomMapperRos::initPublishers()
    {
        
    }
    void CustomMapperRos::depthImageCallback(const sensor_msgs::Image::ConstPtr &image_msg, const sensor_msgs::CameraInfo::ConstPtr &camera_info_msg)
    {
        ROS_INFO_STREAM("[Mapper::DepthImage] depth image received");
        if (!got_depth_image_)
        {
            got_depth_image_ = true;
        }
        float fx, fy, cx, cy;
        int binning_x = std::max<uint32_t>(camera_info_msg->binning_x, 1);
        int binning_y = std::max<uint32_t>(camera_info_msg->binning_y, 1);
        fx = camera_info_msg->K[0] / binning_x;
        fy = camera_info_msg->K[4] / binning_y;
        cx = camera_info_msg->K[2] / binning_x;
        cy = camera_info_msg->K[5] / binning_y;
        // get image
        cv_bridge::CvImageConstPtr depth_ptr = cv_bridge::toCvCopy(image_msg, "32FC1");
        cv::Mat1f depthmap(depth_ptr->image);
        if(image_msg->encoding == "16UC1")
        {
            depthmap /= 1000;
        }
        int height = depthmap.rows;
        int width = depthmap.cols;
        std::cout << "height: " << height << ", width: " << width << std::endl;
        
    }

    void CustomMapperRos::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_ptr)
    {
        return;
    }

    void CustomMapperRos::run()
    {
        getParams();
        initSubcribers();
        initPublishers();
        ros::Rate sleep_rate(20);
        while(ros::ok())
        {
            ros::spinOnce();
            sleep_rate.sleep();
        }
        // start mapping thread

        // ros callback

    }
}