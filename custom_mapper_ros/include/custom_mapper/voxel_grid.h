// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

#include <vector>
#include <cstring>
#include <iostream>

namespace voxel_grid
{
template <typename T>
class VoxelGrid
{
public:
  VoxelGrid();
  VoxelGrid(const double origin[3], const double world_dimensions[3], const double resolution);
  VoxelGrid(const double origin[3], const int grid_dimensions[3], const double resolution, const std::vector<T>& data);
  void GetGridDimensions(int dimensions[3]) const
  {
    memcpy(dimensions, grid_dimensions_, 3 * sizeof(int));
  }
  void WorldToGrid(const double xyz[3], int ixyz[3]) const;
  void GridToWorld(const int ixyz[3], double* xyz) const;
  bool IsInMap(int index) const;
  bool IsInMap(const int ixyz[3]) const;
  bool IsInMap(const double xyz[3]) const;
  std::vector<int> UpdateOrigin(const double xyz[3]);
  std::vector<int> UpdateOrigin(const double x, const double y, const double z);
  int GridToIndex(const int ixyz[3]) const;
  int WorldToIndex(const double xyz[3]) const;
  void IndexToGrid(int ind, int ixyz[3]) const;
  void IndexToWorld(int ind, double xyz[3]) const;
  void SnapToGrid(const double xyz[3], double grid_xyz[3]) const;
  virtual void PreShiftOrigin(const std::vector<int>& slice_indexes)
  {
  }
  virtual void PostShiftOrigin(const std::vector<int>& slice_indexes)
  {
  }
  void WorldToVoxels(const double xyz[3], int voxels[3]) const;
  int GetNumCells() const
  {
    return num_cells_;
  }
  std::vector<T> GetData() const
  {
    return data_;
  }
  double GetResolution() const
  {
    return resolution_;
  }
  void GetOrigin(double xyz[3]) const;
  void PrintIndexes() const;
  void PrintValues() const;
  void PrintWorldX() const;
  void PrintWorldZ() const;
  void PrintState() const;
  void ComputeShiftVoxels(const int new_lower_left_voxels[3], int voxel_shift[3]) const;
  void Reset(T reset_value);
  T ReadValue(const int ind) const;
  T ReadValue(const int ixyz[3]) const;
  T ReadValue(const double xyz[3]) const;
  T ReadValue(const double x, const double y, const double z) const;
  void WriteValue(const int ixyz[3], T value);
  void WriteValue(const double xyz[3], T value);
  void WriteValue(const double x, const double y, const double z, T value);
  void WriteValue(const int ind, T value);

  T& GetReference(const int ind);

private:
  void ShiftOrigin(const int new_lower_left_voxels[3]);
  void ComputeLowerLeftVoxels(const double new_grid_center_xyz[3], int new_lower_left_voxels[3]) const;
  void GetSlice(const int i, const int width, const int dimension, std::vector<int>* slice_indexes) const;
  std::vector<int> GetSliceIndexes(const int new_lower_left_voxels[3]) const;

  void InitializeData()
  {
    num_cells_ = grid_dimensions_[0] * grid_dimensions_[1] * grid_dimensions_[2];
    for (int i = 0; i < num_cells_; i++)
    {
      data_.push_back(T());
    }
  }

  void InitializeData(const std::vector<T>& data)
  {
    num_cells_ = grid_dimensions_[0] * grid_dimensions_[1] * grid_dimensions_[2];
    data_ = data;
  }

  void InitializeOrigin(const double origin[3])
  {
    int new_lower_left_voxels[3] = { 0 };
    ComputeLowerLeftVoxels(origin, new_lower_left_voxels);
    ShiftOrigin(new_lower_left_voxels);
  }

  int lower_left_voxels_[3];
  double resolution_;
  int grid_dimensions_[3];  // map bounds in grid coordinates
  int num_cells_;
  std::vector<T> data_;
};

}  // namespace voxel_grid

