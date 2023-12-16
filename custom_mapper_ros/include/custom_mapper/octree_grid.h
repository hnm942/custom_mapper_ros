#ifndef OCTREE_GRID_H
#define OCTREE_GRID_H

#include <iostream>
#include <vector>
#include <cmath>
namespace octree_grid
{
    struct OctreeConfig
    {
    };
    struct Point
    {
        Point(){};
        Point(const int &x, const int &y, const int &z){};
        int x;
        int y;
        int z;
    };

    struct OctreeNode
    {
        int center[3];
        int min_distance[3];
        int max_distance[3];
        int size;
        bool is_leaf;
        std::vector<OctreeNode *> children;
        OctreeNode(int x, int y, int z, int size)
        {
            // get center
            center[0] = x;
            center[1] = y;
            center[2] = z;
            // calc min distance
            for (int i = 0; i < 3; i++)
            {
                min_distance[i] = center[i] - size / 2.;
                max_distance[i] = center[i] + size / 2.;
            }
            for (int i = 0; i < 8; i++)
            {
                children.push_back(nullptr);
            }
        }

        void split()
        {
            /* Split area to 8 part (each dimension split for 2)
            Return pointer to children part:
            x y z
            - - -
            + - -
            - + -
            + + -
            - - +
            + - +
            - + +
            + + +
            */
            int half_size = size / 2.;
            children[0] = new OctreeNode(center[0] - half_size, center[1] - half_size, center[2] - half_size, half_size);
            children[1] = new OctreeNode(center[0] + half_size, center[1] - half_size, center[2] - half_size, half_size);
            children[2] = new OctreeNode(center[0] - half_size, center[1] + half_size, center[2] - half_size, half_size);
            children[3] = new OctreeNode(center[0] + half_size, center[1] + half_size, center[2] - half_size, half_size);
            children[4] = new OctreeNode(center[0] - half_size, center[1] - half_size, center[2] + half_size, half_size);
            children[5] = new OctreeNode(center[0] + half_size, center[1] - half_size, center[2] + half_size, half_size);
            children[6] = new OctreeNode(center[0] - half_size, center[1] + half_size, center[2] + half_size, half_size);
            children[7] = new OctreeNode(center[0] + half_size, center[1] + half_size, center[2] + half_size, half_size);
        }

        bool insert(const Point &point)
        {
        }

        bool isPointInside(const Point &target)
        {
            if (
                target.x >= min_distance[0] && target.x < max_distance[0] &&
                target.y >= min_distance[1] && target.y < max_distance[1] &&
                target.z >= min_distance[2] && target.z < max_distance[2])
            {
                return true;
            }
            return false;
        }

        bool isOctreeChild(const OctreeNode &child)
        {
            if (
                child.center[0] >= min_distance[0] && child.center[0] < max_distance[0] &&
                child.center[1] >= min_distance[1] && child.center[1] < max_distance[1] &&
                child.center[2] >= min_distance[2] && child.center[2] < max_distance[2])
            {
                return true;
            }
            return false;
        }

        bool searchOctree

        bool checkPointInside(const Point &point)
        {
            /* check point inside node
            parameters
            ----------
            T& pointer T
                point need to check
            return
            ------
            bool
                point inside node or not
            */

            return (point.x >= min_distance[0] && point.x <= max_distance[0] &&
                    point.y >= min_distance[1] && point.y <= max_distance[1] &&
                    point.z >= min_distance[2] && point.z <= max_distance[2]);
        }

        bool checkLeaf()
        {
            for (int i = 0; i < 8; i++)
            {
                if (children[i] != nullptr)
                {
                    is_leaf = true;
                }
            }
            is_leaf = false;
        }
        bool isLeaf()
        {
            return is_leaf;
        }

        bool setLeaf(const bool &value)
        {
            is_leaf = value;
        }
    }; // end octree node

    class Octree
    {
    public:
        Octree()
        {
            for (int i = 0; i < 3; i++)
            {
                grid_dimensions_[i] = 0;
            }
            resolution_ = 0;
        }
        Octree(const int grid_dimensions[3], double resolution)
        {
            for (int i = 0; i < 3; i++)
            {
                grid_dimensions_[i] = grid_dimensions[i];
            }
            resolution_ = resolution;
        }
        ~Octree()
        {
        }
        // insert octreeNode
        bool insertPoint(const double point[3])
        {
            int voxels[3];
            for (int i = 0; i < 3; i++)
            {
                double float_voxels = std::floor(point[i] / resolution_ + 1e-6);
                voxels[i] = static_cast<int>(float_voxels);
            }
            return insertPoint(voxels);
        }
        bool insertPoint(const int point[3])
        {
            Point point(point[0], point[1], point[2]);
            return insertPoint(point);
        }

        bool insertPoint(const Point point)
        {
            return root->insert(point);
        }
        // remove Node
        bool deletePoint(const Point point)
        {
        }

        // search
        Point *searchPoint()
        {
        }
        Point *searchPoints()
        {
        }

        // print
        void printPoint()
        {
        }

    private:
        OctreeNode *root;
        int grid_dimensions_[3];
        double resolution_;
    };

}

#endif