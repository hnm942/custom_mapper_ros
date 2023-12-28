#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>


int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPLYFile<pcl::PointXYZ>("/home/huynm942/workspace/ws/src/custom_mapper/point_cloud.ply", *cloud))
    {
        std::cout << "Couldn't read file "<< std::endl;
    }
    return 0;

}