#include <unity.h>
#include <Math.h>

// void setUp(void) {
// // set stuff up here
// }

// void tearDown(void) {
// // clean stuff up here
// }

void test_math_true(void) {
    TEST_ASSERT_TRUE(true);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_math_true);
    UNITY_END();

    return 0;
}