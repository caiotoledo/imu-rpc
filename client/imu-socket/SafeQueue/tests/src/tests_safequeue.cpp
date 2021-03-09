#include <gtest/gtest.h>

#include <SafeQueue.hpp>

TEST(SafeQueue, Simple_Enqueue_Dequeue)
{
  /* Construct objects */
  Queue::SafeQueue<int> queue;

  /* Prepare local variables */
  int expectedVal = 10;
  int val = 0;

  /* Perform test */
  queue.enqueue(expectedVal);
  auto ret = queue.dequeue(val, -1);

  /* Check Results */
  EXPECT_EQ(ret, true);
  EXPECT_EQ(val, expectedVal);
}

TEST(SafeQueue, Empty_Queue)
{
  /* Construct objects */
  Queue::SafeQueue<int> queue;

  /* Prepare local variables */
  int val = 0;

  /* Perform test */
  auto ret = queue.dequeue(val, 1);

  /* Check Results */
  EXPECT_EQ(ret, false);
}
