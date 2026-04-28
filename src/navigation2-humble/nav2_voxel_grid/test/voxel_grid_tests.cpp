

#include <nav2_voxel_grid/voxel_grid.hpp>
#include <gtest/gtest.h>

TEST(voxel_grid, basicMarkingAndClearing) {
  int size_x = 50, size_y = 10, size_z = 16;
  nav2_voxel_grid::VoxelGrid vg(size_x, size_y, size_z);


  int table_z = 12;
  int table_x_min = 5, table_x_max = 15;
  int table_y_min = 0, table_y_max = 3;
  for (int x = table_x_min; x <= table_x_max; x++) {
    vg.markVoxelLine(x, table_y_min, table_z, x, table_y_max, table_z);
  }

  for (int i = table_x_min; i <= table_x_max; ++i) {
    for (int j = table_y_min; j <= table_y_max; ++j) {

      ASSERT_EQ(nav2_voxel_grid::MARKED, vg.getVoxel(i, j, table_z));
    }
  }

  int mark_count = 0;
  unsigned int unknown_count = 0;

  for (unsigned int i = 0; i < vg.sizeX(); ++i) {
    for (unsigned int j = 0; j < vg.sizeY(); ++j) {
      for (unsigned int k = 0; k < vg.sizeZ(); ++k) {
        if (vg.getVoxel(i, j, k) == nav2_voxel_grid::MARKED) {
          mark_count++;
        } else if (vg.getVoxel(i, j, k) == nav2_voxel_grid::UNKNOWN) {
          unknown_count++;
        }
      }
    }
  }
  ASSERT_EQ(mark_count, 44);


  ASSERT_EQ(unknown_count, vg.sizeX() * vg.sizeY() * vg.sizeZ() - 44);


  vg.clearVoxelLine(table_x_min, table_y_min, table_z, table_x_max, table_y_min, table_z);

  mark_count = 0;
  unknown_count = 0;
  int free_count = 0;

  for (unsigned int i = 0; i < vg.sizeX(); ++i) {
    for (unsigned int j = 0; j < vg.sizeY(); ++j) {
      for (unsigned int k = 0; k < vg.sizeZ(); ++k) {
        if (vg.getVoxel(i, j, k) == nav2_voxel_grid::MARKED) {
          mark_count++;
        } else if (vg.getVoxel(i, j, k) == nav2_voxel_grid::FREE) {
          free_count++;
        } else if (vg.getVoxel(i, j, k) == nav2_voxel_grid::UNKNOWN) {
          unknown_count++;
        }
      }
    }
  }


  ASSERT_EQ(mark_count, 33);


  ASSERT_EQ(free_count, 11);


  ASSERT_EQ(unknown_count, vg.sizeX() * vg.sizeY() * vg.sizeZ() - 44);


  for (unsigned int i = 0; i < vg.sizeZ(); ++i) {
    vg.markVoxel(0, 0, i);
    ASSERT_EQ(vg.getVoxel(0, 0, i), nav2_voxel_grid::MARKED);
  }

  vg.printColumnGrid();
  vg.printVoxelGrid();


  vg.clearVoxelLine(0, 0, 0, 0, 0, vg.sizeZ() - 1);

  for (unsigned int i = 0; i < vg.sizeZ(); ++i) {
    ASSERT_EQ(vg.getVoxel(0, 0, i), nav2_voxel_grid::FREE);
  }

  mark_count = 0;


  

}

TEST(voxel_grid, InvalidSize) {
  int size_x = 50, size_y = 10, size_z = 17;
  int test_z = 16;
  nav2_voxel_grid::VoxelGrid vg(size_x, size_y, size_z);
  vg.resize(size_x, size_y, test_z);
  vg.resize(size_x, size_y, size_z);
  EXPECT_TRUE(vg.getVoxelColumn(51, 10, 0, 0) == nav2_voxel_grid::VoxelStatus::UNKNOWN);
  EXPECT_TRUE(vg.getVoxelColumn(50, 11, 0, 0) == nav2_voxel_grid::VoxelStatus::UNKNOWN);
}

TEST(voxel_grid, MarkAndClear) {
  int size_x = 10, size_y = 10, size_z = 10;
  nav2_voxel_grid::VoxelGrid vg(size_x, size_y, size_z);
  vg.markVoxelInMap(5, 5, 5, 0);
  EXPECT_EQ(vg.getVoxel(5, 5, 5), nav2_voxel_grid::MARKED);
  vg.clearVoxelColumn(55);
  EXPECT_EQ(vg.getVoxel(5, 5, 5), nav2_voxel_grid::FREE);
}

TEST(voxel_grid, clearVoxelLineInMap) {
  int size_x = 10, size_y = 10, size_z = 10;
  nav2_voxel_grid::VoxelGrid vg(size_x, size_y, size_z);
  vg.markVoxelInMap(0, 0, 5, 0);
  EXPECT_EQ(vg.getVoxel(0, 0, 5), nav2_voxel_grid::MARKED);

  unsigned char * map_2d = new unsigned char[100];
  map_2d[0] = 254;

  vg.clearVoxelLineInMap(0, 0, 0, 0, 0, 9, map_2d, 16, 0);

  EXPECT_EQ(map_2d[0], 0);

  vg.markVoxelInMap(0, 0, 5, 0);
  vg.clearVoxelLineInMap(0, 0, 0, 0, 0, 9, nullptr, 16, 0);
  EXPECT_EQ(vg.getVoxel(0, 0, 5), nav2_voxel_grid::FREE);


  vg.markVoxelInMap(0, 0, 5, 0);
  vg.markVoxelInMap(0, 0, 7, 0);
  vg.clearVoxelLineInMap(
    0, 0, 0, 0, 0, 9, nullptr, 16, 0, (unsigned char)'\000',
    (unsigned char)'\377', UINT_MAX, 6);
  EXPECT_EQ(vg.getVoxel(0, 0, 5), nav2_voxel_grid::MARKED);
  EXPECT_EQ(vg.getVoxel(0, 0, 7), nav2_voxel_grid::FREE);

  delete[] map_2d;
}

TEST(voxel_grid, GetVoxelData) {
  uint32_t * data = new uint32_t[9];
  data[4] = 255;
  data[0] = 0;
  EXPECT_EQ(
    nav2_voxel_grid::VoxelGrid::getVoxel(1, 1, 1, 3, 3, 3, data), nav2_voxel_grid::UNKNOWN);

  EXPECT_EQ(
    nav2_voxel_grid::VoxelGrid::getVoxel(0, 0, 0, 3, 3, 3, data), nav2_voxel_grid::FREE);
  delete[] data;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
