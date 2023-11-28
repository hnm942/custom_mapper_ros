#ifndef OCTREE_GRID_H
#define OCTREE_GRID_H

#include <iostream>
#include <vector>

namespace octree_grid
{
    struct OctreeConfig
    {
    };
    template <typename T>
    struct Point
    {
        Point();
        Point(const T &x, connnnnst T &y, const T &z);
        T x;
        T y;
        T z;
    };

    template <typename T>
    struct OctreeNode
    {
        T center[3];
        T min_distance[3];
        T max_distance[3];
        T size;
        bool is_leaf;
        std::vector<OctreeNode *> children;
        OctreeNode(T x, T y, T z, T size)
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
            T half_size = size / 2.;
            children[0] = new OctreeNode(center[0] - half_size, center[1] - half_size, center[2] - half_size, half_size);
            children[1] = new OctreeNode(center[0] + half_size, center[1] - half_size, center[2] - half_size, half_size);
            children[2] = new OctreeNode(center[0] - half_size, center[1] + half_size, center[2] - half_size, half_size);
            children[3] = new OctreeNode(center[0] + half_size, center[1] + half_size, center[2] - half_size, half_size);
            children[4] = new OctreeNode(center[0] - half_size, center[1] - half_size, center[2] + half_size, half_size);
            children[5] = new OctreeNode(center[0] + half_size, center[1] - half_size, center[2] + half_size, half_size);
            children[6] = new OctreeNode(center[0] - half_size, center[1] + half_size, center[2] + half_size, half_size);
            children[7] = new OctreeNode(center[0] + half_size, center[1] + half_size, center[2] + half_size, half_size);
        }

        void insert(const Point<T> &point)
        {
            /* Check if point is exists in node
            if true: insert in current node
            else:
            */
        }

        bool checkPointInside(const Point<T> &point)
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

            return (point[0] >= min_distance[0] && point[0] <= max_distance[0] &&
                    point[1] >= min_distance[1] && point[1] <= max_distance[1] &&
                    point[2] >= min_distance[2] && point[2] <= max_distance[2]);
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

    template <typename T>
    class Octree
    {
        private:
            OctreeNode<T>* root;

        public:
            Octree();
            ~Octree();
    }; // end Octree
}

#endif