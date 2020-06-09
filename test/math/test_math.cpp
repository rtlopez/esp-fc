#include <unity.h>
#include <Math.h>

// void setUp(void) {
// // set stuff up here
// }

// void tearDown(void) {
// // clean stuff up here
// }

void test_math_map()
{
    TEST_ASSERT_FLOAT_WITHIN(1.f,     0.f, Espfc::Math::map(   0.0f, -100.0f, 100.0f, -1000.0f, 1000.0f));
    TEST_ASSERT_FLOAT_WITHIN(1.f,  1000.f, Espfc::Math::map( 100.0f, -100.0f, 100.0f, -1000.0f, 1000.0f));
    TEST_ASSERT_FLOAT_WITHIN(1.f, -1000.f, Espfc::Math::map(-100.0f, -100.0f, 100.0f, -1000.0f, 1000.0f));
    TEST_ASSERT_FLOAT_WITHIN(1.f,   200.f, Espfc::Math::map(  20.0f, -100.0f, 100.0f, -1000.0f, 1000.0f));

    TEST_ASSERT_FLOAT_WITHIN(.001f,  0.f, Espfc::Math::map(   0.0f, -100.0f, 100.0f, -1.0f, 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(.001f,  1.f, Espfc::Math::map( 100.0f, -100.0f, 100.0f, -1.0f, 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(.001f, -1.f, Espfc::Math::map(-100.0f, -100.0f, 100.0f, -1.0f, 1.0f));
}

void test_math_map3()
{
    TEST_ASSERT_FLOAT_WITHIN(1.f,    0.f, Espfc::Math::map3(  0.0f, -100.0f, 0.0f, 100.0f, -1000.0f, 0.0f, 1000.0f));
    TEST_ASSERT_FLOAT_WITHIN(1.f,  500.f, Espfc::Math::map3( 50.0f, -100.0f, 0.0f, 100.0f, -1000.0f, 0.0f, 1000.0f));
    TEST_ASSERT_FLOAT_WITHIN(1.f, -500.f, Espfc::Math::map3(-50.0f, -100.0f, 0.0f, 100.0f, -1000.0f, 0.0f, 1000.0f));
}

void test_math_deadband()
{
    TEST_ASSERT_EQUAL_INT32( 0, Espfc::Math::deadband(  0, 10));
    TEST_ASSERT_EQUAL_INT32( 0, Espfc::Math::deadband( 10, 10));
    TEST_ASSERT_EQUAL_INT32( 1, Espfc::Math::deadband( 11, 10));
    TEST_ASSERT_EQUAL_INT32(-1, Espfc::Math::deadband(-11, 10));
    TEST_ASSERT_EQUAL_INT32( 0, Espfc::Math::deadband( -5, 10));
    TEST_ASSERT_EQUAL_INT32(10, Espfc::Math::deadband( 20, 10));
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_math_map);
    RUN_TEST(test_math_map3);
    RUN_TEST(test_math_deadband);
    UNITY_END();

    return 0;
}