#include <unity.h>

// not used directly, but required so that the library dependency finder gives lib/betaflight
// the Espfc include path, same reason as in test_msp
#include <platform.h>

#include <Hal/Adc.hpp>
#include <Hal/Board.hpp>
#include <Hal/Gpio.hpp>
#include <Hal/Queue.hpp>
#include <type_traits>

using namespace Espfc;

void test_hal_board_id()
{
  TEST_ASSERT_EQUAL_UINT32(0, Hal::Board::getId0());
  TEST_ASSERT_EQUAL_UINT32(0, Hal::Board::getId1());
  TEST_ASSERT_EQUAL_UINT32(0, Hal::Board::getId2());
}

void test_hal_board_stats()
{
  TEST_ASSERT_EQUAL_UINT32(1, Hal::Board::getCpuFreq());
  TEST_ASSERT_EQUAL_UINT32(1, Hal::Board::getFreeHeap());
}

void test_hal_board_reset()
{
  Hal::Board::reset();
  TEST_PASS();
}

void test_hal_board_reset_reason()
{
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Hal::ResetReason::UNKNOWN), static_cast<int>(Hal::Board::getResetReason()));
}

void test_hal_unexpected_reset()
{
  TEST_ASSERT_TRUE(Hal::isUnexpectedReset(Hal::ResetReason::SOFTWARE));
  TEST_ASSERT_TRUE(Hal::isUnexpectedReset(Hal::ResetReason::WATCHDOG));
  TEST_ASSERT_TRUE(Hal::isUnexpectedReset(Hal::ResetReason::PANIC));

  TEST_ASSERT_FALSE(Hal::isUnexpectedReset(Hal::ResetReason::UNKNOWN));
  TEST_ASSERT_FALSE(Hal::isUnexpectedReset(Hal::ResetReason::POWER_ON));
  TEST_ASSERT_FALSE(Hal::isUnexpectedReset(Hal::ResetReason::EXTERNAL_RESET));
  TEST_ASSERT_FALSE(Hal::isUnexpectedReset(Hal::ResetReason::BROWNOUT));
  TEST_ASSERT_FALSE(Hal::isUnexpectedReset(Hal::ResetReason::DEEP_SLEEP));
  TEST_ASSERT_FALSE(Hal::isUnexpectedReset(Hal::ResetReason::OTHER));
}

void test_hal_gpio_interrupt()
{
  int context = 0;
  Hal::Gpio::attachInterrupt(1, [](void* arg) { *static_cast<int*>(arg) = 1; }, &context, Hal::Gpio::Rising);
  Hal::Gpio::detachInterrupt(1);
  TEST_ASSERT_EQUAL_INT(0, context);
}

void test_hal_adc_read()
{
  Hal::Adc::begin(1);
  TEST_ASSERT_EQUAL_UINT16(0, Hal::Adc::read(1));
}

template<typename Index>
void queue_atomic_empty_full()
{
  Hal::Detail::QueueAtomic<int, 4, Index> q;
  q.begin();

  int e1 = 1, e2 = 2, e3 = 3, e4 = 4;
  int r1 = 91, r2 = 92, r3 = 93, r4 = 94;

  TEST_ASSERT_EQUAL_size_t(3, q.capacity());

  // empty
  TEST_ASSERT_TRUE(q.isEmpty());
  TEST_ASSERT_FALSE(q.isFull());
  TEST_ASSERT_EQUAL_size_t(0, q.size());

  // pop on empty queue does not touch the argument
  TEST_ASSERT_FALSE(q.pop(r1));
  TEST_ASSERT_EQUAL_INT(91, r1);

  // push first element
  TEST_ASSERT_TRUE(q.push(e1));
  TEST_ASSERT_FALSE(q.isEmpty());
  TEST_ASSERT_FALSE(q.isFull());
  TEST_ASSERT_EQUAL_size_t(1, q.size());

  // pop last element
  TEST_ASSERT_TRUE(q.pop(r1));
  TEST_ASSERT_EQUAL_INT(1, r1);
  TEST_ASSERT_TRUE(q.isEmpty());
  TEST_ASSERT_EQUAL_size_t(0, q.size());

  // pop element again
  TEST_ASSERT_FALSE(q.pop(r1));
  TEST_ASSERT_EQUAL_INT(1, r1);

  // make full, push on full queue is rejected
  TEST_ASSERT_TRUE(q.push(e1));
  TEST_ASSERT_TRUE(q.push(e2));
  TEST_ASSERT_TRUE(q.push(e3));
  TEST_ASSERT_FALSE(q.push(e4));

  TEST_ASSERT_FALSE(q.isEmpty());
  TEST_ASSERT_TRUE(q.isFull());
  TEST_ASSERT_EQUAL_size_t(3, q.size());

  // make empty, order is preserved and the rejected element is not there
  TEST_ASSERT_TRUE(q.pop(r1));
  TEST_ASSERT_EQUAL_INT(1, r1);

  TEST_ASSERT_TRUE(q.pop(r2));
  TEST_ASSERT_EQUAL_INT(2, r2);

  TEST_ASSERT_TRUE(q.pop(r3));
  TEST_ASSERT_EQUAL_INT(3, r3);

  TEST_ASSERT_FALSE(q.pop(r4));
  TEST_ASSERT_EQUAL_INT(94, r4);

  TEST_ASSERT_TRUE(q.isEmpty());
  TEST_ASSERT_FALSE(q.isFull());
}

template<typename Index>
void queue_atomic_wrap_around()
{
  Hal::Detail::QueueAtomic<int, 4, Index> q;
  q.begin();

  // push and pop many times to cross the buffer boundary repeatedly
  for (int i = 0; i < 100; i++)
  {
    int in = i;
    int out = -1;
    TEST_ASSERT_TRUE(q.push(in));
    TEST_ASSERT_TRUE(q.pop(out));
    TEST_ASSERT_EQUAL_INT(i, out);
    TEST_ASSERT_TRUE(q.isEmpty());
  }

  // partially filled queue crossing the boundary keeps the order
  for (int i = 0; i < 100; i++)
  {
    int a = i * 2;
    int b = i * 2 + 1;
    int out = -1;

    TEST_ASSERT_TRUE(q.push(a));
    TEST_ASSERT_TRUE(q.push(b));
    TEST_ASSERT_EQUAL_size_t(2, q.size());

    TEST_ASSERT_TRUE(q.pop(out));
    TEST_ASSERT_EQUAL_INT(a, out);
    TEST_ASSERT_TRUE(q.pop(out));
    TEST_ASSERT_EQUAL_INT(b, out);
  }
}

void test_hal_queue_atomic_plain_index()
{
  queue_atomic_empty_full<Hal::Detail::PlainIndex>();
}

void test_hal_queue_atomic_plain_index_wrap()
{
  queue_atomic_wrap_around<Hal::Detail::PlainIndex>();
}

void test_hal_queue_atomic_atomic_index()
{
  queue_atomic_empty_full<Hal::Detail::AtomicIndex>();
}

void test_hal_queue_atomic_atomic_index_wrap()
{
  queue_atomic_wrap_around<Hal::Detail::AtomicIndex>();
}

void test_hal_queue_null()
{
  Hal::Detail::QueueNull<int, 16> q;
  q.begin();

  int in = 1;
  int out = 91;

  TEST_ASSERT_EQUAL_size_t(0, q.capacity());
  TEST_ASSERT_TRUE(q.isEmpty());
  TEST_ASSERT_FALSE(q.isFull());

  TEST_ASSERT_FALSE(q.push(in));
  TEST_ASSERT_TRUE(q.isEmpty());
  TEST_ASSERT_EQUAL_size_t(0, q.size());

  TEST_ASSERT_FALSE(q.pop(out));
  TEST_ASSERT_EQUAL_INT(91, out);
}

void test_hal_queue_selection()
{
  // single core host build must end up with the empty implementation
  static_assert(!Hal::MULTI_CORE, "host build is single core");
  static_assert(std::is_same_v<Hal::Queue<int, 16>, Hal::Detail::QueueNull<int, 16>>, "unexpected queue selected");
  TEST_PASS();
}

int main(int argc, char** argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_hal_board_id);
  RUN_TEST(test_hal_board_stats);
  RUN_TEST(test_hal_board_reset);
  RUN_TEST(test_hal_board_reset_reason);
  RUN_TEST(test_hal_unexpected_reset);
  RUN_TEST(test_hal_gpio_interrupt);
  RUN_TEST(test_hal_adc_read);
  RUN_TEST(test_hal_queue_atomic_plain_index);
  RUN_TEST(test_hal_queue_atomic_plain_index_wrap);
  RUN_TEST(test_hal_queue_atomic_atomic_index);
  RUN_TEST(test_hal_queue_atomic_atomic_index_wrap);
  RUN_TEST(test_hal_queue_null);
  RUN_TEST(test_hal_queue_selection);

  return UNITY_END();
}
