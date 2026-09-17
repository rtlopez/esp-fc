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

int main(int argc, char** argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_hal_board_id);
  RUN_TEST(test_hal_board_stats);
  RUN_TEST(test_hal_board_reset);
  RUN_TEST(test_hal_board_reset_reason);
  RUN_TEST(test_hal_unexpected_reset);

  return UNITY_END();
}
