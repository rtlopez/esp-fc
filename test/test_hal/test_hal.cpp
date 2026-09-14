#include <unity.h>

// not used directly, but required so that the library dependency finder gives lib/betaflight
// the Espfc include path, same reason as in test_msp
#include <platform.h>

#include <Hal/Board.hpp>

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

int main(int argc, char** argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_hal_board_id);
  RUN_TEST(test_hal_board_stats);
  RUN_TEST(test_hal_board_reset);

  return UNITY_END();
}
