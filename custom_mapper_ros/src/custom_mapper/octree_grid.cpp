#include "custom_mapper/octree_grid.h"

namespace octree_grid
{

    Octree::Octree()
    {
        for (int i = 0; i < 3; i++)
        {
            grid_dimensions_[i] = 0;
        }
        world_resolution_ = 0;
        grid_resolution_ = 0;
        max_depth_ = 0;
        root = new OctreeNode(0, 0, 0, grid_dimensions_[0], 0);
    }

    Octree::Octree(const int grid_dimensions[3], const double world_resolution, const int grid_resolution) : world_resolution_(world_resolution), grid_resolution_(grid_resolution)
    {
        for (int i = 0; i < 3; i++)
        {
            grid_dimensions_[i] = grid_dimensions[i];
        }
        // split ratio by x
        double ratio = static_cast<double>(grid_dimensions[0]) / grid_resolution;
        max_depth_ = static_cast<int>(log2(ratio));
        root = new OctreeNode(0, 0, 0, grid_dimensions_[0], 0);
    }

    Octree::Octree(const int grid_dimensions[3], const double world_resolution, const int grid_resolution, const int max_depth) : world_resolution_(world_resolution), grid_resolution_(grid_resolution), max_depth_(max_depth)
    {
        for (int i = 0; i < 3; i++)
        {
            grid_dimensions_[i] = grid_dimensions[i];
        }
        grid_resolution_ = static_cast<int>(grid_dimensions[0] / std::pow(2, max_depth_));
        root = new OctreeNode(0, 0, 0, grid_dimensions_[0], 0);
    }

    Octree::~Octree()
    {
        delete root;
    }
    // insert octreeNode
    bool Octree::insertPoint(const double point[3])
    {
        int voxels[3];
        for (int i = 0; i < 3; i++)
        {
            double float_voxels = std::floor(point[i] / world_resolution_ + 1e-6);
            voxels[i] = static_cast<int>(float_voxels);
        }
        return insertPoint(voxels);
    }
    bool Octree::insertPoint(const int xyz[3])
    {
        Point point(xyz[0], xyz[1], xyz[2]);
        return insertPoint(point);
    }

    bool Octree::insertPoint(const Point point)
    {
        return insertNode(root, point, max_depth_);
    }

    bool Octree::insertNode(OctreeNode *node, const Point point, const int depth)
    {
        if (node->depth == depth)
        {
            node->is_leaf = true;
            return true;
        }
        node->split();
        for (int i = 0; i < 8; i++)
        {
            if (node->children[i]->isPointInside(point))
            {
                insertNode(node->children[i], point, depth);
            }
        }
    }

    // remove Node
    bool Octree::deletePoint(const int xyz[3], const int depth)
    {
        Point point(xyz[0], xyz[1], xyz[2]);
        return deletePoint(point, depth);
    }

    bool Octree::deletePoint(const Point &point, const int depth)
    {
        // find leaf
        OctreeNode *node = search(point, depth);
        return deleteNode(node);
    }

    bool Octree::deleteNode(OctreeNode *node)
    {
        for (int i = 0; i < 8; i++)
        {
            if (node->children[i] != nullptr)
            {
                deleteNode(node->children[i]);
                node->children[i] = nullptr;
            }
        }
        delete node;
        return true;
    }

    void Octree::printPoint()
    {
    }
    // search
    OctreeNode *Octree::search(const int xyz[3], int depth)
    {
        Point point(xyz[0], xyz[1], xyz[2]);
    }
    OctreeNode *Octree::search(const Point &point, int depth)
    {
        return searchNode(root, point, depth);
    }

    OctreeNode *Octree::searchNode(OctreeNode *node, const Point &point, int depth)
    {
        if (node->is_leaf)
        {
            return node;
        }
        if (node->depth == depth)
        {
            return node;
        }
        for (int i = 0; i < 8; i++)
        {
            if (node->children[i]->isPointInside(point))
            {
                return searchNode(node->children[i], point, depth);
            }
        }
    }

    // print

} // namespace octree_grid