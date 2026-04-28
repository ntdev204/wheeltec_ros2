


#include <string>
#include "gtest/gtest.h"
#include "costmap_queue/map_based_queue.hpp"

using costmap_queue::MapBasedQueue;

void letter_test(MapBasedQueue<char> & q, const char test_letter)
{
  ASSERT_FALSE(q.isEmpty());
  char c = q.front();
  EXPECT_EQ(c, test_letter);
  q.pop();
}

TEST(MapBasedQueue, emptyQueue)
{
  MapBasedQueue<char> q;
  EXPECT_TRUE(q.isEmpty());
  q.enqueue(1.0, 'A');
  EXPECT_FALSE(q.isEmpty());
}

TEST(MapBasedQueue, checkOrdering)
{
  MapBasedQueue<char> q;
  q.enqueue(1.0, 'A');
  q.enqueue(3.0, 'B');
  q.enqueue(2.0, 'C');
  q.enqueue(5.0, 'D');
  q.enqueue(0.0, 'E');

  std::string expected = "EACBD";
  for (unsigned int i = 0; i < expected.size(); i++) {
    letter_test(q, expected[i]);
  }
  EXPECT_TRUE(q.isEmpty());
}

TEST(MapBasedQueue, checkDynamicOrdering)
{
  MapBasedQueue<char> q;
  q.enqueue(1.0, 'A');
  q.enqueue(3.0, 'B');
  q.enqueue(2.0, 'C');
  q.enqueue(5.0, 'D');

  std::string expected = "ACB";
  for (unsigned int i = 0; i < expected.size(); i++) {
    letter_test(q, expected[i]);
  }

  q.enqueue(0.0, 'E');
  letter_test(q, 'E');
}

TEST(MapBasedQueue, checkDynamicOrdering2)
{
  MapBasedQueue<char> q;
  q.enqueue(1.0, 'A');
  q.enqueue(2.0, 'B');
  letter_test(q, 'A');
  q.enqueue(3.0, 'C');
  letter_test(q, 'B');
}

TEST(MapBasedQueue, checkDynamicOrdering3)
{
  MapBasedQueue<char> q;
  q.enqueue(1.0, 'A');
  q.enqueue(2.0, 'B');
  q.enqueue(5.0, 'D');
  letter_test(q, 'A');
  letter_test(q, 'B');
  q.enqueue(1.0, 'C');
  letter_test(q, 'C');
  letter_test(q, 'D');
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
