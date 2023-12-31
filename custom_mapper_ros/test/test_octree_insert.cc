#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <string>
#include "custom_mapper/octree_grid.h"
int main()
{
    std::string file_path = "/home/hnm942/workspace/src/custom_mapper_ros/custom_mapper_ros/data/rgbd_dataset_freiburg1_360/point_cloud_1305031790.ply";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(file_path, *cloud))
    {
        std::cout << "Couldn't read file " << std::endl;
    }
    // test file:
    std::cout <<"read point cloud from: " << file_path << std::endl;
    int grid_dimensions[3] = {100, 100, 100};
    double world_resolution = 20;
    double grid_resolution = 0.05;
    int max_depth = 10;
    std::unique_ptr<octree_grid::Octree> octree_ptr = std::make_unique<octree_grid::Octree>(grid_dimensions, world_resolution, grid_resolution, max_depth);
    // create loop to insert pcl to octree
    int i = 0;
    pcl::PointXYZ point = cloud->points[i];

    float x = cloud->points[i].x;
    float y = cloud->points[i].y;
    float z = cloud->points[i].z;
    float xyz[3] = {x, y ,z};
    octree_ptr->insertPoint(xyz);    
    for(int i = 0; i < cloud->points.size(); i++)
    {
            octree_ptr->insertPoint(xyz);    
    }
    return 0;
    
}