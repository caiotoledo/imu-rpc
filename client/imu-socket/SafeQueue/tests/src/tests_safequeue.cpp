#include <future>

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

TEST(SafeQueue, Async_Wait_Dequeue)
{
  /* Construct objects */
  Queue::SafeQueue<int> queue;

  /* Prepare local variables */
  constexpr auto SLEEP_WAIT = 5; /* ms */
  int expectedVal1 = 10, expectedVal2 = 20;
  int val1 = 0, val2 = 0;

  auto asyncDequeue = [&queue](int timeout)
  {
    int val = 0;
    queue.dequeue(val, timeout);
    return val;
  };

  /* Perform test */
  auto fut1 = std::async(std::launch::async, asyncDequeue, -1); /* Wait forever for a value */
  std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_WAIT)); /* Keep async dequeue blocked waiting */
  queue.enqueue(expectedVal1);
  val1 = fut1.get();

  auto fut2 = std::async(std::launch::async, asyncDequeue, 5*SLEEP_WAIT);
  std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_WAIT)); /* Keep async dequeue blocked waiting */
  queue.enqueue(expectedVal2);
  val2 = fut2.get();

  /* Check Results */
  EXPECT_EQ(val1, expectedVal1);
  EXPECT_EQ(val2, expectedVal2);
}
