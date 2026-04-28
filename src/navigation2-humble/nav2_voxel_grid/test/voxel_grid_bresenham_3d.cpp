

#include <nav2_voxel_grid/voxel_grid.hpp>
#include <gtest/gtest.h>

class TestVoxel
{
public:
  explicit TestVoxel(uint32_t * data, int sz_x, int sz_y)
  : data_(data)
  {
    size_ = sz_x * sz_y;
  }
  inline void operator()(unsigned int off, unsigned int val)
  {
    ASSERT_TRUE(off < size_);
    data_[off] = val;
  }
  inline unsigned int operator()(unsigned int off)
  {
    return data_[off];
  }

private:
  uint32_t * data_;
  unsigned int size_;
};

TEST(voxel_grid, bresenham3DBoundariesCheck)
{
  const int sz_x = 60;
  const int sz_y = 60;
  const int sz_z = 2;
  const unsigned int max_length = 60;
  const unsigned int min_length = 6;
  nav2_voxel_grid::VoxelGrid vg(sz_x, sz_y, sz_z);
  TestVoxel tv(vg.getData(), sz_x, sz_y);


  const double x0 = 2.2;
  const double y0 = 3.8;
  const double z0 = 0.4;

  const double z1 = 0.5;

  double x1, y1;


  const double epsilon = 0.02;


  y1 = 0.0;
  for (int i = 0; i <= sz_x; i++) {
    if (i != sz_x) {
      x1 = i;
    } else {
      x1 = i - epsilon;
    }
    vg.raytraceLine(tv, x0, y0, z0, x1, y1, z1, max_length, min_length);
  }


  y1 = sz_y - epsilon;
  for (int i = 0; i <= sz_x; i++) {
    if (i != sz_x) {
      x1 = i;
    } else {
      x1 = i - epsilon;
    }
    vg.raytraceLine(tv, x0, y0, z0, x1, y1, z1, max_length, min_length);
  }


  x1 = 0.0;
  for (int j = 0; j <= sz_y; j++) {
    if (j != sz_y) {
      y1 = j;
    } else {
      y1 = j - epsilon;
    }
    vg.raytraceLine(tv, x0, y0, z0, x1, y1, z1, max_length, min_length);
  }


  x1 = sz_x - epsilon;
  for (int j = 0; j <= sz_y; j++) {
    if (j != sz_y) {
      y1 = j;
    } else {
      y1 = j - epsilon;
    }
    vg.raytraceLine(tv, x0, y0, z0, x1, y1, z1, max_length, min_length);
  }
}

TEST(voxel_grid, bresenham3DSamePoint)
{
  const int sz_x = 60;
  const int sz_y = 60;
  const int sz_z = 2;
  const unsigned int max_length = 60;
  const unsigned int min_length = 0;
  nav2_voxel_grid::VoxelGrid vg(sz_x, sz_y, sz_z);
  TestVoxel tv(vg.getData(), sz_x, sz_y);


  const double x0 = 2.2;
  const double y0 = 3.8;
  const double z0 = 0.4;

  unsigned int offset = static_cast<int>(y0) * sz_x + static_cast<int>(x0);
  unsigned int val_before = tv(offset);

  vg.raytraceLine(tv, x0, y0, z0, x0, y0, z0, max_length, min_length);
  unsigned int val_after = tv(offset);
  ASSERT_FALSE(val_before == val_after);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
