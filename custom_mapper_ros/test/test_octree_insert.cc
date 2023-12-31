#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>("/home/hnm942/workspace/src/custom_mapper_ros/custom_mapper_ros/data/rgbd_dataset_freiburg1_360.bag", *cloud))
    {
        std::cout << "Couldn't read file " << std::endl;
    }
    return 0;
    
}