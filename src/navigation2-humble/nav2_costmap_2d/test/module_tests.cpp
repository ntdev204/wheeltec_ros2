




#include <gtest/gtest.h>
#include <set>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"

const unsigned char MAP_10_BY_10_CHAR[] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 200, 200, 200,
  0, 0, 0, 0, 100, 0, 0, 200, 200, 200,
  0, 0, 0, 0, 100, 0, 0, 200, 200, 200,
  70, 70, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 200, 200, 200, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 255, 255, 255,
  0, 0, 0, 0, 0, 0, 0, 255, 255, 255
};

const unsigned char MAP_5_BY_5_CHAR[] = {
  0, 0, 0, 0, 0,
  0, 0, 0, 0, 0,
  0, 0, 0, 0, 0,
  0, 0, 0, 0, 0,
  0, 0, 0, 0, 0,
};

std::vector<unsigned char> MAP_5_BY_5;
std::vector<unsigned char> MAP_10_BY_10;
std::vector<unsigned char> EMPTY_10_BY_10;
std::vector<unsigned char> EMPTY_100_BY_100;

const unsigned int GRID_WIDTH(10);
const unsigned int GRID_HEIGHT(10);
const double RESOLUTION(1);
const double WINDOW_LENGTH(10);
const unsigned char THRESHOLD(100);
const double MAX_Z(1.0);
const double RAYTRACE_MAX_RANGE(20.0);
const double RAYTRACE_MIN_RANGE(3.0);
const double OBSTACLE_MAX_RANGE(20.0);
const double OBSTACLE_MIN_RANGE(0.0);
const double ROBOT_RADIUS(1.0);

bool find(const std::vector<unsigned int> & l, unsigned int n)
{
  for (std::vector<unsigned int>::const_iterator it = l.begin(); it != l.end(); ++it) {
    if (*it == n) {
      return true;
    }
  }

  return false;
}



TEST(costmap, testResetForStaticMap) {

  std::vector<unsigned char> staticMap;
  for (unsigned int i = 0; i < 10; i++) {
    for (unsigned int j = 0; j < 10; j++) {
      staticMap.push_back(nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }


  nav2_costmap_2d::Costmap2D map(10, 10, RESOLUTION, 0.0, 0.0, 3, 3, 3,
    OBSTACLE_MAX_RANGE, OBSTACLE_MIN_RANGE, MAX_Z, RAYTRACE_MAX_RANGE, RAYTRACE_MIN_RANGE, 25,
    staticMap, THRESHOLD);


  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.resize(40);


  unsigned int ind = 0;
  for (unsigned int i = 0; i < 10; i++) {

    cloud.points[ind].x = 0;
    cloud.points[ind].y = i;
    cloud.points[ind].z = MAX_Z;
    ind++;


    cloud.points[ind].x = i;
    cloud.points[ind].y = 0;
    cloud.points[ind].z = MAX_Z;
    ind++;


    cloud.points[ind].x = 9;
    cloud.points[ind].y = i;
    cloud.points[ind].z = MAX_Z;
    ind++;


    cloud.points[ind].x = i;
    cloud.points[ind].y = 9;
    cloud.points[ind].z = MAX_Z;
    ind++;
  }

  double wx = 5.0, wy = 5.0;
  geometry_msgs::Point p;
  p.x = wx;
  p.y = wy;
  p.z = MAX_Z;
  nav2_costmap_2d::Observation obs(p, cloud, OBSTACLE_MAX_RANGE, OBSTACLE_MIN_RANGE,
    RAYTRACE_MAX_RANGE,
    RAYTRACE_MIN_RANGE);
  std::vector<nav2_costmap_2d::Observation> obsBuf;
  obsBuf.push_back(obs);


  map.updateWorld(wx, wy, obsBuf, obsBuf);



  int hitCount = 0;
  for (unsigned int i = 0; i < 10; ++i) {
    for (unsigned int j = 0; j < 10; ++j) {
      if (map.getCost(i, j) == nav2_costmap_2d::LETHAL_OBSTACLE) {
        hitCount++;
      }
    }
  }
  ASSERT_EQ(hitCount, 36);


  hitCount = 0;
  for (unsigned int i = 0; i < 10; ++i) {
    for (unsigned int j = 0; j < 10; ++j) {
      if (map.getCost(i, j) != nav2_costmap_2d::LETHAL_OBSTACLE) {
        hitCount++;
      }
    }
  }
  ASSERT_EQ(hitCount, 64);


  map.resetMapOutsideWindow(wx, wy, 0.0, 0.0);


  hitCount = 0;
  for (unsigned int i = 0; i < 10; ++i) {
    for (unsigned int j = 0; j < 10; ++j) {
      if (map.getCost(i, j) == nav2_costmap_2d::LETHAL_OBSTACLE) {
        hitCount++;
      }
    }
  }
  ASSERT_EQ(hitCount, 100);
}



TEST(costmap, testCostFunctionCorrectness) {
  nav2_costmap_2d::Costmap2D map(100, 100, RESOLUTION, 0.0, 0.0,
    ROBOT_RADIUS * 5.0, ROBOT_RADIUS * 8.0, ROBOT_RADIUS * 10.5,
    100.0, MAX_Z, 100.0, 25, EMPTY_100_BY_100, THRESHOLD);


  unsigned char c = map.computeCost((ROBOT_RADIUS * 8.0 / RESOLUTION));
  ASSERT_EQ(map.getCircumscribedCost(), c);


  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.resize(1);
  cloud.points[0].x = 50;
  cloud.points[0].y = 50;
  cloud.points[0].z = MAX_Z;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  nav2_costmap_2d::Observation obs(p, cloud, 100.0, 100.0);
  std::vector<nav2_costmap_2d::Observation> obsBuf;
  obsBuf.push_back(obs);

  map.updateWorld(0, 0, obsBuf, obsBuf);

  for (unsigned int i = 0; i <= (unsigned int)ceil(ROBOT_RADIUS * 5.0); i++) {

    ASSERT_EQ(map.getCost(50 + i, 50) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    ASSERT_EQ(map.getCost(50 + i, 50) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);

    ASSERT_EQ(map.getCost(50 - i, 50) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    ASSERT_EQ(map.getCost(50 - i, 50) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);

    ASSERT_EQ(map.getCost(50, 50 + i) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    ASSERT_EQ(map.getCost(50, 50 + i) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);

    ASSERT_EQ(map.getCost(50, 50 - i) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    ASSERT_EQ(map.getCost(50, 50 - i) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
  }


  for (unsigned int i = (unsigned int)(ceil(ROBOT_RADIUS * 5.0) + 1);
    i <= (unsigned int)ceil(ROBOT_RADIUS * 10.5); i++)
  {
    unsigned char expectedValue = map.computeCost(i / RESOLUTION);
    ASSERT_EQ(map.getCost(50 + i, 50), expectedValue);
  }


  map.resetMapOutsideWindow(0, 0, 0.0, 0.0);
  cloud.points.resize(0);

  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  nav2_costmap_2d::Observation obs2(p, cloud, 100.0, 100.0);
  std::vector<nav2_costmap_2d::Observation> obsBuf2;
  obsBuf2.push_back(obs2);

  map.updateWorld(0, 0, obsBuf2, obsBuf2);

  for (unsigned int i = 0; i < 100; i++) {
    for (unsigned int j = 0; j < 100; j++) {
      ASSERT_EQ(map.getCost(i, j), nav2_costmap_2d::FREE_SPACE);
    }
  }
}

char printableCost(unsigned char cost)
{
  switch (cost) {
    case nav2_costmap_2d::NO_INFORMATION: return '?';
    case nav2_costmap_2d::LETHAL_OBSTACLE: return 'L';
    case nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE: return 'I';
    case nav2_costmap_2d::FREE_SPACE: return '.';
    default: return '0' + (unsigned char) (10 * cost / 255);
  }
}



TEST(costmap, testWaveInterference) {

  nav2_costmap_2d::Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0,
    ROBOT_RADIUS, ROBOT_RADIUS * 2, ROBOT_RADIUS * 3.01,
    10.0, MAX_Z * 2, 10.0, 1, EMPTY_10_BY_10, THRESHOLD);


  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.resize(3);
  cloud.points[0].x = 3;
  cloud.points[0].y = 3;
  cloud.points[0].z = MAX_Z;
  cloud.points[1].x = 5;
  cloud.points[1].y = 5;
  cloud.points[1].z = MAX_Z;
  cloud.points[2].x = 7;
  cloud.points[2].y = 7;
  cloud.points[2].z = MAX_Z;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  nav2_costmap_2d::Observation obs(p, cloud, 100.0, 100.0);
  std::vector<nav2_costmap_2d::Observation> obsBuf;
  obsBuf.push_back(obs);

  map.updateWorld(0, 0, obsBuf, obsBuf);

  int update_count = 0;


  printf("map:\n");
  for (unsigned int i = 0; i < 10; ++i) {
    for (unsigned int j = 0; j < 10; ++j) {
      if (map.getCost(i, j) != nav2_costmap_2d::FREE_SPACE) {
        update_count++;
      }
      printf("%c", printableCost(map.getCost(i, j)));
    }
    printf("\n");
  }

  ASSERT_EQ(update_count, 79);
}


TEST(costmap, testWindowCopy) {
  nav2_costmap_2d::Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0,
    ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS,
    10.0, MAX_Z, 10.0, 25, MAP_10_BY_10, THRESHOLD);

  


  nav2_costmap_2d::Costmap2D windowCopy;


  windowCopy.copyCostmapWindow(map, 2.0, 2.0, 6.0, 12.0);
  ASSERT_EQ(windowCopy.getSizeInCellsX(), (unsigned int)0);
  ASSERT_EQ(windowCopy.getSizeInCellsY(), (unsigned int)0);


  map.copyCostmapWindow(map, 2.0, 2.0, 6.0, 6.0);
  ASSERT_EQ(map.getSizeInCellsX(), (unsigned int)10);
  ASSERT_EQ(map.getSizeInCellsY(), (unsigned int)10);


  windowCopy.copyCostmapWindow(map, 2.0, 2.0, 6.0, 6.0);
  ASSERT_EQ(windowCopy.getSizeInCellsX(), (unsigned int)6);
  ASSERT_EQ(windowCopy.getSizeInCellsY(), (unsigned int)6);


  for (unsigned int i = 0; i < windowCopy.getSizeInCellsX(); ++i) {
    for (unsigned int j = 0; j < windowCopy.getSizeInCellsY(); ++j) {
      ASSERT_EQ(windowCopy.getCost(i, j), map.getCost(i + 2, j + 2));

    }

  }
}


TEST(costmap, testFullyContainedStaticMapUpdate) {
  nav2_costmap_2d::Costmap2D map(5, 5, RESOLUTION, 0.0, 0.0,
    ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS,
    10.0, MAX_Z, 10.0, 25, MAP_5_BY_5, THRESHOLD);

  nav2_costmap_2d::Costmap2D static_map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0,
    ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS,
    10.0, MAX_Z, 10.0, 25, MAP_10_BY_10, THRESHOLD);

  map.updateStaticMapWindow(0, 0, 10, 10, MAP_10_BY_10);

  for (unsigned int i = 0; i < map.getSizeInCellsX(); ++i) {
    for (unsigned int j = 0; j < map.getSizeInCellsY(); ++j) {
      ASSERT_EQ(map.getCost(i, j), static_map.getCost(i, j));
    }
  }
}

TEST(costmap, testOverlapStaticMapUpdate) {
  nav2_costmap_2d::Costmap2D map(5, 5, RESOLUTION, 0.0, 0.0,
    ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS,
    10.0, MAX_Z, 10.0, 25, MAP_5_BY_5, THRESHOLD);

  nav2_costmap_2d::Costmap2D static_map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0,
    ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS,
    10.0, MAX_Z, 10.0, 25, MAP_10_BY_10, THRESHOLD);

  map.updateStaticMapWindow(-10, -10, 10, 10, MAP_10_BY_10);

  ASSERT_FLOAT_EQ(map.getOriginX(), -10);
  ASSERT_FLOAT_EQ(map.getOriginX(), -10);
  ASSERT_EQ(map.getSizeInCellsX(), (unsigned int)15);
  ASSERT_EQ(map.getSizeInCellsY(), (unsigned int)15);
  for (unsigned int i = 0; i < 10; ++i) {
    for (unsigned int j = 0; j < 10; ++j) {
      ASSERT_EQ(map.getCost(i, j), static_map.getCost(i, j));
    }
  }

  std::vector<unsigned char> blank(100);


  map.updateStaticMapWindow(-10, -10, 10, 10, blank);

  for (unsigned int i = 0; i < map.getSizeInCellsX(); ++i) {
    for (unsigned int j = 0; j < map.getSizeInCellsY(); ++j) {
      ASSERT_EQ(map.getCost(i, j), 0);
    }
  }

  std::vector<unsigned char> fully_contained(25);
  fully_contained[0] = 254;
  fully_contained[4] = 254;
  fully_contained[5] = 254;
  fully_contained[9] = 254;

  nav2_costmap_2d::Costmap2D small_static_map(5, 5, RESOLUTION, 0.0, 0.0,
    ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS,
    10.0, MAX_Z, 10.0, 25, fully_contained, THRESHOLD);

  map.updateStaticMapWindow(0, 0, 5, 5, fully_contained);

  ASSERT_FLOAT_EQ(map.getOriginX(), -10);
  ASSERT_FLOAT_EQ(map.getOriginX(), -10);
  ASSERT_EQ(map.getSizeInCellsX(), (unsigned int)15);
  ASSERT_EQ(map.getSizeInCellsY(), (unsigned int)15);
  for (unsigned int j = 0; j < 5; ++j) {
    for (unsigned int i = 0; i < 5; ++i) {
      ASSERT_EQ(map.getCost(i + 10, j + 10), small_static_map.getCost(i, j));
    }
  }
}



TEST(costmap, testRaytracing) {
  nav2_costmap_2d::Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0,
    ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS,
    10.0, MAX_Z, 10.0, 25, MAP_10_BY_10, THRESHOLD);


  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.resize(1);
  cloud.points[0].x = 0;
  cloud.points[0].y = 0;
  cloud.points[0].z = MAX_Z;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  nav2_costmap_2d::Observation obs(p, cloud, 100.0, 100.0);
  std::vector<nav2_costmap_2d::Observation> obsBuf;
  obsBuf.push_back(obs);

  map.updateWorld(0, 0, obsBuf, obsBuf);

  int lethal_count = 0;

  for (unsigned int i = 0; i < 10; ++i) {
    for (unsigned int j = 0; j < 10; ++j) {
      if (map.getCost(i, j) == nav2_costmap_2d::LETHAL_OBSTACLE) {
        lethal_count++;
      }
    }
  }


  ASSERT_EQ(lethal_count, 21);
}

TEST(costmap, testAdjacentToObstacleCanStillMove) {
  nav2_costmap_2d::Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0, 2.1, 3.1, 4.1,
    10.0, MAX_Z, 10.0, 25, MAP_10_BY_10, THRESHOLD);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.resize(1);
  cloud.points[0].x = 0;
  cloud.points[0].y = 0;
  cloud.points[0].z = MAX_Z;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  nav2_costmap_2d::Observation obs(p, cloud, 100.0, 100.0);
  std::vector<nav2_costmap_2d::Observation> obsBuf;
  obsBuf.push_back(obs);

  map.updateWorld(9, 9, obsBuf, obsBuf);

  EXPECT_EQ(nav2_costmap_2d::LETHAL_OBSTACLE, map.getCost(0, 0));
  EXPECT_EQ(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, map.getCost(1, 0));
  EXPECT_EQ(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, map.getCost(2, 0));
  EXPECT_TRUE(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE > map.getCost(3, 0));
  EXPECT_TRUE(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE > map.getCost(2, 1));
  EXPECT_EQ(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, map.getCost(1, 1));
}

TEST(costmap, testInflationShouldNotCreateUnknowns) {
  nav2_costmap_2d::Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0, 2.1, 3.1, 4.1,
    10.0, MAX_Z, 10.0, 25, MAP_10_BY_10, THRESHOLD);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.resize(1);
  cloud.points[0].x = 0;
  cloud.points[0].y = 0;
  cloud.points[0].z = MAX_Z;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  nav2_costmap_2d::Observation obs(p, cloud, 100.0, 100.0);
  std::vector<nav2_costmap_2d::Observation> obsBuf;
  obsBuf.push_back(obs);

  map.updateWorld(9, 9, obsBuf, obsBuf);

  int unknown_count = 0;

  for (unsigned int i = 0; i < 10; ++i) {
    for (unsigned int j = 0; j < 10; ++j) {
      if (map.getCost(i, j) == nav2_costmap_2d::NO_INFORMATION) {
        unknown_count++;
      }
    }
  }
  EXPECT_EQ(0, unknown_count);
}

unsigned int worldToIndex(nav2_costmap_2d::Costmap2D & map, double wx, double wy)
{
  unsigned int mx, my;
  map.worldToMap(wx, wy, mx, my);
  return map.getIndex(mx, my);
}

void indexToWorld(nav2_costmap_2d::Costmap2D & map, unsigned int index, double & wx, double & wy)
{
  unsigned int mx, my;
  map.indexToCells(index, mx, my);
  map.mapToWorld(mx, my, wx, wy);
}

TEST(costmap, testStaticMap) {
  nav2_costmap_2d::Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0,
    ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS,
    10.0, MAX_Z, 10.0, 25, MAP_10_BY_10, THRESHOLD);

  ASSERT_EQ(map.getSizeInCellsX(), (unsigned int)10);
  ASSERT_EQ(map.getSizeInCellsY(), (unsigned int)10);


  std::vector<unsigned int> occupiedCells;

  for (unsigned int i = 0; i < 10; ++i) {
    for (unsigned int j = 0; j < 10; ++j) {
      if (map.getCost(i, j) == nav2_costmap_2d::LETHAL_OBSTACLE) {
        occupiedCells.push_back(map.getIndex(i, j));
      }
    }
  }

  ASSERT_EQ(occupiedCells.size(), (unsigned int)20);


  for (std::vector<unsigned int>::const_iterator it = occupiedCells.begin();
    it != occupiedCells.end(); ++it)
  {
    unsigned int ind = *it;
    unsigned int x, y;
    map.indexToCells(ind, x, y);
    ASSERT_EQ(find(occupiedCells, map.getIndex(x, y)), true);
    ASSERT_EQ(MAP_10_BY_10[ind] >= 100, true);
    ASSERT_EQ(map.getCost(x, y) >= 100, true);
  }


  ASSERT_EQ(find(occupiedCells, map.getIndex(7, 2)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(8, 2)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(9, 2)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(7, 3)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(8, 3)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(9, 3)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(7, 4)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(8, 4)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(9, 4)), true);


  ASSERT_EQ(find(occupiedCells, map.getIndex(4, 3)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(4, 4)), true);


  ASSERT_EQ(find(occupiedCells, map.getIndex(3, 7)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(4, 7)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(5, 7)), true);



  ASSERT_EQ(worldToIndex(map, 0.0, 0.0), (unsigned int)0);
  ASSERT_EQ(worldToIndex(map, 0.0, 0.99), (unsigned int)0);
  ASSERT_EQ(worldToIndex(map, 0.0, 1.0), (unsigned int)10);
  ASSERT_EQ(worldToIndex(map, 1.0, 0.99), (unsigned int)1);
  ASSERT_EQ(worldToIndex(map, 9.99, 9.99), (unsigned int)99);
  ASSERT_EQ(worldToIndex(map, 8.2, 3.4), (unsigned int)38);


  double wx, wy;
  indexToWorld(map, 99, wx, wy);
  ASSERT_EQ(wx, 9.5);
  ASSERT_EQ(wy, 9.5);
}





TEST(costmap, testDynamicObstacles) {
  nav2_costmap_2d::Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0,
    ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS,
    10.0, MAX_Z, 10.0, 25, MAP_10_BY_10, THRESHOLD);


  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.resize(3);
  cloud.points[0].x = 0;
  cloud.points[0].y = 0;
  cloud.points[1].x = 0;
  cloud.points[1].y = 0;
  cloud.points[2].x = 0;
  cloud.points[2].y = 0;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  nav2_costmap_2d::Observation obs(p, cloud, 100.0, 100.0);
  std::vector<nav2_costmap_2d::Observation> obsBuf;
  obsBuf.push_back(obs);

  map.updateWorld(0, 0, obsBuf, obsBuf);

  std::vector<unsigned int> ids;

  for (unsigned int i = 0; i < 10; ++i) {
    for (unsigned int j = 0; j < 10; ++j) {
      if (map.getCost(i, j) == nav2_costmap_2d::LETHAL_OBSTACLE) {
        ids.push_back(map.getIndex(i, j));
      }
    }
  }


  ASSERT_EQ(ids.size(), (unsigned int)21);


  map.updateWorld(0, 0, obsBuf, obsBuf);
  ASSERT_EQ(ids.size(), (unsigned int)21);
}



TEST(costmap, testMultipleAdditions) {
  nav2_costmap_2d::Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0,
    ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS,
    10.0, MAX_Z, 10.0, 25, MAP_10_BY_10, THRESHOLD);


  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.resize(1);
  cloud.points[0].x = 7;
  cloud.points[0].y = 2;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  nav2_costmap_2d::Observation obs(p, cloud, 100.0, 100.0);
  std::vector<nav2_costmap_2d::Observation> obsBuf;
  obsBuf.push_back(obs);

  map.updateWorld(0, 0, obsBuf, obsBuf);

  std::vector<unsigned int> ids;

  for (unsigned int i = 0; i < 10; ++i) {
    for (unsigned int j = 0; j < 10; ++j) {
      if (map.getCost(i, j) == nav2_costmap_2d::LETHAL_OBSTACLE) {
        ids.push_back(map.getIndex(i, j));
      }
    }
  }

  ASSERT_EQ(ids.size(), (unsigned int)20);
}



TEST(costmap, testZThreshold) {
  nav2_costmap_2d::Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0,
    ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS,
    10.0, MAX_Z, 10.0, 25, MAP_10_BY_10, THRESHOLD);


  pcl::PointCloud<pcl::PointXYZ> c0;
  c0.points.resize(2);
  c0.points[0].x = 0;
  c0.points[0].y = 5;
  c0.points[0].z = 0.4;
  c0.points[1].x = 1;
  c0.points[1].y = 5;
  c0.points[1].z = 1.2;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  nav2_costmap_2d::Observation obs(p, c0, 100.0, 100.0);
  std::vector<nav2_costmap_2d::Observation> obsBuf;
  obsBuf.push_back(obs);

  map.updateWorld(0, 0, obsBuf, obsBuf);

  std::vector<unsigned int> ids;

  for (unsigned int i = 0; i < 10; ++i) {
    for (unsigned int j = 0; j < 10; ++j) {
      if (map.getCost(i, j) == nav2_costmap_2d::LETHAL_OBSTACLE) {
        ids.push_back(map.getIndex(i, j));
      }
    }
  }
  ASSERT_EQ(ids.size(), (unsigned int)21);
}




TEST(costmap, testInflation) {
  nav2_costmap_2d::Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0,
    ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS,
    10.0, MAX_Z, 10.0, 25, MAP_10_BY_10, THRESHOLD);


  std::vector<unsigned int> occupiedCells;

  for (unsigned int i = 0; i < 10; ++i) {
    for (unsigned int j = 0; j < 10; ++j) {
      if (map.getCost(i, j) == nav2_costmap_2d::LETHAL_OBSTACLE ||
        map.getCost(i, j) == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        occupiedCells.push_back(map.getIndex(i, j));
      }
    }
  }


  std::set<unsigned int> setOfCells;
  for (unsigned int i = 0; i < occupiedCells.size(); i++) {
    setOfCells.insert(i);
  }

  ASSERT_EQ(setOfCells.size(), occupiedCells.size());
  ASSERT_EQ(setOfCells.size(), (unsigned int)48);


  for (std::vector<unsigned int>::const_iterator it = occupiedCells.begin();
    it != occupiedCells.end(); ++it)
  {
    unsigned int ind = *it;
    unsigned int x, y;
    map.indexToCells(ind, x, y);
    ASSERT_EQ(find(occupiedCells, map.getIndex(x, y)), true);
    ASSERT_EQ(
      map.getCost(x, y) == nav2_costmap_2d::LETHAL_OBSTACLE ||
      map.getCost(x, y) == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
  }


  pcl::PointCloud<pcl::PointXYZ> c0;
  c0.points.resize(1);
  c0.points[0].x = 0;
  c0.points[0].y = 0;
  c0.points[0].z = 0.4;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  nav2_costmap_2d::Observation obs(p, c0, 100.0, 100.0);
  std::vector<nav2_costmap_2d::Observation> obsBuf, empty;
  obsBuf.push_back(obs);

  map.updateWorld(0, 0, obsBuf, empty);

  occupiedCells.clear();
  for (unsigned int i = 0; i < 10; ++i) {
    for (unsigned int j = 0; j < 10; ++j) {
      if (map.getCost(i, j) == nav2_costmap_2d::LETHAL_OBSTACLE ||
        map.getCost(i, j) == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        occupiedCells.push_back(map.getIndex(i, j));
      }
    }
  }


  ASSERT_EQ(occupiedCells.size(), (unsigned int)51);



  pcl::PointCloud<pcl::PointXYZ> c1;
  c1.points.resize(1);
  c1.points[0].x = 2;
  c1.points[0].y = 0;
  c1.points[0].z = 0.0;

  geometry_msgs::Point p1;
  p1.x = 0.0;
  p1.y = 0.0;
  p1.z = MAX_Z;

  nav2_costmap_2d::Observation obs1(p1, c1, 100.0, 100.0);
  std::vector<nav2_costmap_2d::Observation> obsBuf1;
  obsBuf1.push_back(obs1);

  map.updateWorld(0, 0, obsBuf1, empty);

  occupiedCells.clear();
  for (unsigned int i = 0; i < 10; ++i) {
    for (unsigned int j = 0; j < 10; ++j) {
      if (map.getCost(i, j) == nav2_costmap_2d::LETHAL_OBSTACLE ||
        map.getCost(i, j) == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        occupiedCells.push_back(map.getIndex(i, j));
      }
    }
  }





  ASSERT_EQ(occupiedCells.size(), (unsigned int)54);



  pcl::PointCloud<pcl::PointXYZ> c2;
  c2.points.resize(1);
  c2.points[0].x = 1;
  c2.points[0].y = 9;
  c2.points[0].z = 0.0;

  geometry_msgs::Point p2;
  p2.x = 0.0;
  p2.y = 0.0;
  p2.z = MAX_Z;

  nav2_costmap_2d::Observation obs2(p2, c2, 100.0, 100.0);
  std::vector<nav2_costmap_2d::Observation> obsBuf2;
  obsBuf2.push_back(obs2);

  map.updateWorld(0, 0, obsBuf2, empty);

  ASSERT_EQ(map.getCost(1, 9), nav2_costmap_2d::LETHAL_OBSTACLE);
  ASSERT_EQ(map.getCost(0, 9), nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  ASSERT_EQ(map.getCost(2, 9), nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);


  pcl::PointCloud<pcl::PointXYZ> c3;
  c3.points.resize(1);
  c3.points[0].x = 0;
  c3.points[0].y = 9;
  c3.points[0].z = 0.0;

  geometry_msgs::Point p3;
  p3.x = 0.0;
  p3.y = 0.0;
  p3.z = MAX_Z;

  nav2_costmap_2d::Observation obs3(p3, c3, 100.0, 100.0);
  std::vector<nav2_costmap_2d::Observation> obsBuf3;
  obsBuf3.push_back(obs3);

  map.updateWorld(0, 0, obsBuf3, empty);

  ASSERT_EQ(map.getCost(0, 9), nav2_costmap_2d::LETHAL_OBSTACLE);
}



TEST(costmap, testInflation2) {
  nav2_costmap_2d::Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0,
    ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS,
    10.0, MAX_Z, 10.0, 25, MAP_10_BY_10, THRESHOLD);


  pcl::PointCloud<pcl::PointXYZ> c0;
  c0.points.resize(3);
  c0.points[0].x = 1;
  c0.points[0].y = 1;
  c0.points[0].z = MAX_Z;
  c0.points[1].x = 1;
  c0.points[1].y = 2;
  c0.points[1].z = MAX_Z;
  c0.points[2].x = 2;
  c0.points[2].y = 2;
  c0.points[2].z = MAX_Z;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  nav2_costmap_2d::Observation obs(p, c0, 100.0, 100.0);
  std::vector<nav2_costmap_2d::Observation> obsBuf;
  obsBuf.push_back(obs);

  map.updateWorld(0, 0, obsBuf, obsBuf);

  ASSERT_EQ(map.getCost(3, 2), nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  ASSERT_EQ(map.getCost(3, 3), nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
}



TEST(costmap, testInflation3) {
  std::vector<unsigned char> mapData;
  for (unsigned int i = 0; i < GRID_WIDTH; i++) {
    for (unsigned int j = 0; j < GRID_HEIGHT; j++) {
      mapData.push_back(0);
    }
  }

  nav2_costmap_2d::Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0,
    ROBOT_RADIUS, ROBOT_RADIUS * 2, ROBOT_RADIUS * 3,
    10.0, MAX_Z, 10.0, 1, mapData, THRESHOLD);


  std::vector<unsigned int> ids;

  for (unsigned int i = 0; i < 10; ++i) {
    for (unsigned int j = 0; j < 10; ++j) {
      if (map.getCost(i, j) == nav2_costmap_2d::LETHAL_OBSTACLE ||
        map.getCost(i, j) == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        ids.push_back(map.getIndex(i, j));
      }
    }
  }

  ASSERT_EQ(ids.size(), (unsigned int)0);


  pcl::PointCloud<pcl::PointXYZ> c0;
  c0.points.resize(1);
  c0.points[0].x = 5;
  c0.points[0].y = 5;
  c0.points[0].z = MAX_Z;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  nav2_costmap_2d::Observation obs(p, c0, 100.0, 100.0);
  std::vector<nav2_costmap_2d::Observation> obsBuf;
  obsBuf.push_back(obs);

  map.updateWorld(0, 0, obsBuf, obsBuf);

  for (unsigned int i = 0; i < 10; ++i) {
    for (unsigned int j = 0; j < 10; ++j) {
      if (map.getCost(i, j) != nav2_costmap_2d::FREE_SPACE) {
        ids.push_back(map.getIndex(i, j));
      }
    }
  }

  ASSERT_EQ(ids.size(), (unsigned int)29);

  ids.clear();
  for (unsigned int i = 0; i < 10; ++i) {
    for (unsigned int j = 0; j < 10; ++j) {
      if (map.getCost(i, j) == nav2_costmap_2d::LETHAL_OBSTACLE ||
        map.getCost(i, j) == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        ids.push_back(map.getIndex(i, j));
      }
    }
  }

  ASSERT_EQ(ids.size(), (unsigned int)5);


  map.updateWorld(0, 0, obsBuf, obsBuf);

  ids.clear();
  for (unsigned int i = 0; i < 10; ++i) {
    for (unsigned int j = 0; j < 10; ++j) {
      if (map.getCost(i, j) != nav2_costmap_2d::FREE_SPACE) {
        ids.push_back(map.getIndex(i, j));
      }
    }
  }

  ASSERT_EQ(ids.size(), (unsigned int)29);
}




TEST(costmap, testRaytracing2) {
  nav2_costmap_2d::Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0,
    ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS,
    100.0, MAX_Z, 100.0, 1, MAP_10_BY_10, THRESHOLD);



  pcl::PointCloud<pcl::PointXYZ> c0;
  c0.points.resize(1);
  c0.points[0].x = 9.5;
  c0.points[0].y = 9.5;
  c0.points[0].z = MAX_Z;

  geometry_msgs::Point p;
  p.x = 0.5;
  p.y = 0.5;
  p.z = MAX_Z;

  nav2_costmap_2d::Observation obs(p, c0, 100.0, 100.0);
  std::vector<nav2_costmap_2d::Observation> obsBuf;
  obsBuf.push_back(obs);

  std::vector<unsigned int> obstacles;

  for (unsigned int i = 0; i < 10; ++i) {
    for (unsigned int j = 0; j < 10; ++j) {
      if (map.getCost(i, j) == nav2_costmap_2d::LETHAL_OBSTACLE) {
        obstacles.push_back(map.getIndex(i, j));
      }
    }
  }

  unsigned int obs_before = obstacles.size();

  map.updateWorld(0, 0, obsBuf, obsBuf);

  obstacles.clear();
  for (unsigned int i = 0; i < 10; ++i) {
    for (unsigned int j = 0; j < 10; ++j) {
      if (map.getCost(i, j) == nav2_costmap_2d::LETHAL_OBSTACLE) {
        obstacles.push_back(map.getIndex(i, j));
      }
    }
  }


  ASSERT_EQ(obstacles.size(), obs_before - 2);





  unsigned char test[10] = {0, 0, 0, 253, 253, 0, 0, 253, 253, 254};
  for (unsigned int i = 0; i < 10; i++) {
    ASSERT_EQ(map.getCost(i, i), test[i]);
  }
}




TEST(costmap, testTrickyPropagation) {
  const unsigned char MAP_HALL_CHAR[10 * 10] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    254, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 254, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 254, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 254, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  };
  std::vector<unsigned char> MAP_HALL;
  for (int i = 0; i < 10 * 10; i++) {
    MAP_HALL.push_back(MAP_HALL_CHAR[i]);
  }

  nav2_costmap_2d::Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0,
    ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS,
    100.0, MAX_Z, 100.0, 1, MAP_HALL, THRESHOLD);



  pcl::PointCloud<pcl::PointXYZ> c2;
  c2.points.resize(3);

  c2.points[0].x = 7.0;
  c2.points[0].y = 8.0;
  c2.points[0].z = 1.0;


  c2.points[1].x = 3.0;
  c2.points[1].y = 4.0;
  c2.points[1].z = 1.0;

  c2.points[2].x = 6.0;
  c2.points[2].y = 3.0;
  c2.points[2].z = 1.0;

  geometry_msgs::Point p2;
  p2.x = 0.5;
  p2.y = 0.5;
  p2.z = MAX_Z;

  nav2_costmap_2d::Observation obs2(p2, c2, 100.0, 100.0);
  std::vector<nav2_costmap_2d::Observation> obsBuf2;
  obsBuf2.push_back(obs2);

  map.updateWorld(0, 0, obsBuf2, obsBuf2);

  const unsigned char MAP_HALL_CHAR_TEST[10 * 10] = {
    253, 254, 253, 0, 0, 0, 0, 0, 0, 0,
    0, 253, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 253, 0, 0, 0, 0, 0,
    0, 0, 0, 253, 254, 253, 0, 0, 0, 0,
    0, 0, 0, 0, 253, 0, 0, 253, 0, 0,
    0, 0, 0, 253, 0, 0, 253, 254, 253, 0,
    0, 0, 253, 254, 253, 0, 0, 253, 253, 0,
    0, 0, 0, 253, 0, 0, 0, 253, 254, 253,
    0, 0, 0, 0, 0, 0, 0, 0, 253, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  };

  for (int i = 0; i < 10 * 10; i++) {
    ASSERT_EQ(map.getCost(i / 10, i % 10), MAP_HALL_CHAR_TEST[i]);
  }

  pcl::PointCloud<pcl::PointXYZ> c;
  c.points.resize(1);

  c.points[0].x = 4.0;
  c.points[0].y = 5.0;
  c.points[0].z = 1.0;

  geometry_msgs::Point p3;
  p3.x = 0.5;
  p3.y = 0.5;
  p3.z = MAX_Z;

  nav2_costmap_2d::Observation obs3(p3, c, 100.0, 100.0);
  std::vector<nav2_costmap_2d::Observation> obsBuf3;
  obsBuf3.push_back(obs3);

  map.updateWorld(0, 0, obsBuf3, obsBuf3);

  const unsigned char MAP_HALL_CHAR_TEST2[10 * 10] = {
    253, 254, 253, 0, 0, 0, 0, 0, 0, 0,
    0, 253, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 253, 0, 0, 0, 0,
    0, 0, 0, 0, 253, 254, 253, 253, 0, 0,
    0, 0, 0, 253, 0, 253, 253, 254, 253, 0,
    0, 0, 253, 254, 253, 0, 0, 253, 253, 0,
    0, 0, 0, 253, 0, 0, 0, 253, 254, 253,
    0, 0, 0, 0, 0, 0, 0, 0, 253, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  };

  for (int i = 0; i < 10 * 10; i++) {
    ASSERT_EQ(map.getCost(i / 10, i % 10), MAP_HALL_CHAR_TEST2[i]);
  }
}


int main(int argc, char ** argv)
{
  for (unsigned int i = 0; i < GRID_WIDTH * GRID_HEIGHT; i++) {
    EMPTY_10_BY_10.push_back(0);
    MAP_10_BY_10.push_back(MAP_10_BY_10_CHAR[i]);
  }

  for (unsigned int i = 0; i < 5 * 5; i++) {
    MAP_5_BY_5.push_back(MAP_10_BY_10_CHAR[i]);
  }

  for (unsigned int i = 0; i < 100 * 100; i++) {
    EMPTY_100_BY_100.push_back(0);
  }

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
