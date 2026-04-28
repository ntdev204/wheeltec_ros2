

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <gtest/gtest.h>

class CostmapAction
{
public:
  explicit CostmapAction(
    unsigned char * costmap, unsigned int size, unsigned char mark_val = 128)
  : costmap_(costmap), size_(size), mark_val_(mark_val)
  {
  }

  inline void operator()(unsigned int off)
  {
    ASSERT_TRUE(off < size_);
    costmap_[off] = mark_val_;
  }

  inline unsigned int get(unsigned int off)
  {
    return costmap_[off];
  }

private:
  unsigned char * costmap_;
  unsigned int size_;
  unsigned char mark_val_;
};

class CostmapTest : public nav2_costmap_2d::Costmap2D
{
public:
  CostmapTest(
    unsigned int size_x, unsigned int size_y, double resolution,
    double origin_x, double origin_y, unsigned char default_val = 0)
  : nav2_costmap_2d::Costmap2D(size_x, size_y, resolution, origin_x, origin_y, default_val)
  {
  }

  unsigned char * getCostmap()
  {
    return costmap_;
  }

  unsigned int getSize()
  {
    return size_x_ * size_y_;
  }

  void raytraceLine(
    CostmapAction ca, unsigned int x0, unsigned int y0, unsigned int x1,
    unsigned int y1,
    unsigned int max_length = UINT_MAX, unsigned int min_length = 0)
  {
    nav2_costmap_2d::Costmap2D::raytraceLine(ca, x0, y0, x1, y1, max_length, min_length);
  }
};

TEST(costmap_2d, bresenham2DBoundariesCheck)
{
  const unsigned int sz_x = 60;
  const unsigned int sz_y = 60;
  const unsigned int max_length = 60;
  const unsigned int min_length = 6;
  CostmapTest ct(sz_x, sz_y, 0.1, 0.0, 0.0);
  CostmapAction ca(ct.getCostmap(), ct.getSize());


  const unsigned int x0 = 2;
  const unsigned int y0 = 4;

  unsigned int x1, y1;


  y1 = 0;
  for (x1 = 0; x1 < sz_x; x1++) {
    ct.raytraceLine(ca, x0, y0, x1, y1, max_length, min_length);
  }


  y1 = sz_y - 1;
  for (x1 = 0; x1 < sz_x; x1++) {
    ct.raytraceLine(ca, x0, y0, x1, y1, max_length, min_length);
  }


  x1 = 0;
  for (y1 = 0; y1 < sz_y; y1++) {
    ct.raytraceLine(ca, x0, y0, x1, y1, max_length, min_length);
  }


  x1 = sz_x - 1;
  for (y1 = 0; y1 < sz_y; y1++) {
    ct.raytraceLine(ca, x0, y0, x1, y1, max_length, min_length);
  }
}

TEST(costmap_2d, bresenham2DSamePoint)
{
  const unsigned int sz_x = 60;
  const unsigned int sz_y = 60;
  const unsigned int max_length = 60;
  const unsigned int min_length = 0;
  CostmapTest ct(sz_x, sz_y, 0.1, 0.0, 0.0);
  CostmapAction ca(ct.getCostmap(), ct.getSize());


  const double x0 = 2;
  const double y0 = 4;

  unsigned int offset = y0 * sz_x + x0;
  unsigned char val_before = ca.get(offset);

  ct.raytraceLine(ca, x0, y0, x0, y0, max_length, min_length);
  unsigned char val_after = ca.get(offset);
  ASSERT_FALSE(val_before == val_after);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
