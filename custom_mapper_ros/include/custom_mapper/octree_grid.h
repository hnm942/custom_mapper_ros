#ifndef OCTREE_GRID_H
#define OCTREE_GRID_H

#include <iostream>
#include <vector>
#include <cmath>

#include "custom_mapper/octree_types.h"

namespace octree_grid
{
    class Octree
    {
    public:
        Octree();
        Octree(const int grid_dimensions[3], const double world_resolution, const int grid_resolution);
        Octree(const int grid_dimensions[3], const double world_resolution, const int max_depth);
        ~Octree();
        // insert octreeNode
        bool insertPoint(const double point[3]);
        bool insertPoint(const int point[3]);
        bool insertPoint(const Point point);
        // remove Node
        bool deletePoint(const Point point);

        // search
        OctreeNode *search(const int point[3], int depth = 0);
        OctreeNode *search(const Point& point, int depth = 0);

        // print
        void printPoint();

    private:
        bool insertNode(OctreeNode* node, const Point point, int depth = 0);
        OctreeNode *searchNode(OctreeNode* node, const Point& point,const int depth = 0);
        OctreeNode *root;
        int grid_dimensions_[3];
        double world_resolution_;
        int max_depth_;
        int grid_resolution_;
    };

}

#endif