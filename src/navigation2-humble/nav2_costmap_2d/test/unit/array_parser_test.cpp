


#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/array_parser.hpp"

TEST(array_parser, basic_operation)
{
  std::string error;
  std::vector<std::vector<float>> vvf;
  vvf = nav2_costmap_2d::parseVVF("[[1, 2.2], [.3, -4e4]]", error);
  EXPECT_EQ(2u, vvf.size() );
  EXPECT_EQ(2u, vvf[0].size() );
  EXPECT_EQ(2u, vvf[1].size() );
  EXPECT_EQ(1.0f, vvf[0][0]);
  EXPECT_EQ(2.2f, vvf[0][1]);
  EXPECT_EQ(0.3f, vvf[1][0]);
  EXPECT_EQ(-40000.0f, vvf[1][1]);
  EXPECT_EQ("", error);
}

TEST(array_parser, missing_open)
{
  std::string error;
  std::vector<std::vector<float>> vvf;
  vvf = nav2_costmap_2d::parseVVF("[1, 2.2], [.3, -4e4]]", error);
  EXPECT_NE(error, "");
}

TEST(array_parser, missing_close)
{
  std::string error;
  std::vector<std::vector<float>> vvf;
  vvf = nav2_costmap_2d::parseVVF("[[1, 2.2], [.3, -4e4]", error);
  EXPECT_NE(error, "");
}

TEST(array_parser, wrong_depth)
{
  std::string error;
  std::vector<std::vector<float>> vvf;
  vvf = nav2_costmap_2d::parseVVF("[1, 2.2], [.3, -4e4]", error);
  EXPECT_NE(error, "");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
