


#include "gtest/gtest.h"
#include "dwb_plugins/one_d_velocity_iterator.hpp"

using dwb_plugins::OneDVelocityIterator;

const double EPSILON = 1e-3;

TEST(VelocityIterator, basics)
{
  OneDVelocityIterator it(2.0, 0.0, 5.0, 1.0, -1.0, 1.0, 2);
  EXPECT_FALSE(it.isFinished());
  EXPECT_NEAR(it.getVelocity(), 1.0, EPSILON);
  EXPECT_FALSE(it.isFinished());
  ++it;
  EXPECT_FALSE(it.isFinished());
  EXPECT_NEAR(it.getVelocity(), 3.0, EPSILON);
  EXPECT_FALSE(it.isFinished());
  ++it;
  EXPECT_TRUE(it.isFinished());
  it.reset();
  EXPECT_FALSE(it.isFinished());
  EXPECT_NEAR(it.getVelocity(), 1.0, EPSILON);
}

TEST(VelocityIterator, limits)
{
  OneDVelocityIterator it(2.0, 1.5, 2.5, 1.0, -1.0, 1.0, 2);
  EXPECT_NEAR(it.getVelocity(), 1.5, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), 2.5, EPSILON);
}

TEST(VelocityIterator, acceleration)
{
  OneDVelocityIterator it(2.0, 0.0, 5.0, 0.5, -0.5, 1.0, 2);
  EXPECT_NEAR(it.getVelocity(), 1.5, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), 2.5, EPSILON);
}


TEST(VelocityIterator, time)
{
  OneDVelocityIterator it(2.0, 0.0, 5.0, 1.0, -1.0, 0.5, 2);
  EXPECT_NEAR(it.getVelocity(), 1.5, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), 2.5, EPSILON);
}

TEST(VelocityIterator, samples)
{
  OneDVelocityIterator it(2.0, 0.0, 5.0, 1.0, -1.0, 1.0, 3);
  EXPECT_NEAR(it.getVelocity(), 1.0, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), 2.0, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), 3.0, EPSILON);
  ++it;
  EXPECT_TRUE(it.isFinished());
}


TEST(VelocityIterator, samples2)
{
  OneDVelocityIterator it(2.0, 0.0, 5.0, 1.0, -1.0, 1.0, 5);
  EXPECT_NEAR(it.getVelocity(), 1.0, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), 1.5, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), 2.0, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), 2.5, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), 3.0, EPSILON);
  ++it;
  EXPECT_TRUE(it.isFinished());
}

TEST(VelocityIterator, around_zero)
{
  OneDVelocityIterator it(0.0, -5.0, 5.0, 1.0, -1.0, 1.0, 2);
  EXPECT_NEAR(it.getVelocity(), -1.0, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), 0.0, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), 1.0, EPSILON);
  ++it;
}

TEST(VelocityIterator, around_zero2)
{
  OneDVelocityIterator it(0.0, -5.0, 5.0, 1.0, -1.0, 1.0, 4);
  EXPECT_NEAR(it.getVelocity(), -1.0, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), -0.3333, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), 0.0, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), 0.3333, EPSILON);
  ++it;
  EXPECT_NEAR(it.getVelocity(), 1.0, EPSILON);
  ++it;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
