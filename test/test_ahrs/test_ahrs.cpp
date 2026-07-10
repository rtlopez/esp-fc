#include <cmath>
#include <unity.h>

#include <Complementary.h>
#include <Kalman.h>
#include <Madgwick.h>
#include <Mahony.h>
#include <Rtqf.hpp>
#include <helper_3dmath.h>

// Test suite for the AHRS sensor fusion module.
//
// Conventions (matching Control/Fusion.cpp usage):
//   - gyro is in rad/s
//   - accel direction encodes attitude; magnitude is irrelevant (normalized
//     internally), so we use unit-gravity vectors
//   - euler angles returned by getEuler() are in radians (roll, pitch, yaw)
//
// A body at rest measures gravity as +1g along the body Z axis when level.
// A pure roll  of r about X gives accel = (0, sin r,  cos r)
// A pure pitch of p about Y gives accel = (-sin p, 0,  cos p)

static const float PI_F = 3.14159265358979f;

static void assert_quaternion_finite(const Quaternion& q)
{
  TEST_ASSERT_TRUE(std::isfinite(q.w));
  TEST_ASSERT_TRUE(std::isfinite(q.x));
  TEST_ASSERT_TRUE(std::isfinite(q.y));
  TEST_ASSERT_TRUE(std::isfinite(q.z));
}

static void assert_quaternion_unit(const Quaternion& q)
{
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, q.getMagnitude());
}

// ------------------------------------------------------------------ Rtqf ---

// Regression for the slerp-based correction: with zero orientation error the
// old code hit a "skip if error too small" branch to dodge normalizing a
// near-zero axis. slerp() must stay finite and unit-length in this case.
void test_rtqf_zero_error_stable()
{
  Rtqf f;
  f.begin(1000.0f);
  f.setKp(0.0002f);

  for (int i = 0; i < 5000; i++)
  {
    f.update(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f); // level, no rotation
  }

  const auto q = f.getQuaternion();
  assert_quaternion_finite(q);
  assert_quaternion_unit(q);
}

void test_rtqf_level_converges()
{
  Rtqf f;
  f.begin(1000.0f);
  f.setKp(0.0002f);

  for (int i = 0; i < 5000; i++)
  {
    f.update(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
  }

  const auto q = f.getQuaternion();
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, std::fabs(q.w));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, q.x);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, q.y);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, q.z);
}

void test_rtqf_roll_tracks()
{
  const float roll = 30.0f * PI_F / 180.0f;
  Rtqf f;
  f.begin(1000.0f);
  f.setKp(0.0002f);

  for (int i = 0; i < 5000; i++)
  {
    f.update(0.0f, 0.0f, 0.0f, 0.0f, std::sin(roll), std::cos(roll));
  }

  const auto q = f.getQuaternion();
  assert_quaternion_unit(q);
  // pure roll -> q = (cos(r/2), sin(r/2), 0, 0)
  TEST_ASSERT_FLOAT_WITHIN(0.01f, std::cos(roll / 2.0f), q.w);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, std::sin(roll / 2.0f), q.x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, q.y);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, q.z);
}

void test_rtqf_pitch_tracks()
{
  const float pitch = 20.0f * PI_F / 180.0f;
  Rtqf f;
  f.begin(1000.0f);
  f.setKp(0.0002f);

  for (int i = 0; i < 5000; i++)
  {
    f.update(0.0f, 0.0f, 0.0f, -std::sin(pitch), 0.0f, std::cos(pitch));
  }

  const auto q = f.getQuaternion();
  assert_quaternion_unit(q);
  // pure pitch -> q = (cos(p/2), 0, sin(p/2), 0)
  TEST_ASSERT_FLOAT_WITHIN(0.01f, std::cos(pitch / 2.0f), q.w);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, q.x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, std::sin(pitch / 2.0f), q.y);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, q.z);
}

// Feed inconsistent gyro + accel and ensure the estimate never diverges or
// produces NaN (guards the predict/normalize/slerp pipeline).
void test_rtqf_stays_normalized_under_motion()
{
  Rtqf f;
  f.begin(1000.0f);
  f.setKp(0.0002f);

  for (int i = 0; i < 10000; i++)
  {
    float t = i * 0.001f;
    float gx = 2.0f * std::sin(t * 3.0f);
    float gy = 1.5f * std::cos(t * 2.0f);
    float gz = 0.8f * std::sin(t);
    f.update(gx, gy, gz, 0.1f * std::sin(t), 0.1f * std::cos(t), 1.0f);
    assert_quaternion_finite(f.getQuaternion());
  }
  assert_quaternion_unit(f.getQuaternion());
}

// -------------------------------------------------------------- Madgwick ---

void test_madgwick_level_converges()
{
  Madgwick f;
  f.begin(1000.0f);
  f.setKp(0.5f);

  for (int i = 0; i < 20000; i++)
  {
    f.update(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
  }

  const auto e = f.getEuler();
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, e.x); // roll
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, e.y); // pitch
  assert_quaternion_finite(f.getQuaternion());
}

void test_madgwick_roll_converges()
{
  const float roll = 30.0f * PI_F / 180.0f;
  Madgwick f;
  f.begin(1000.0f);
  f.setKp(0.5f);

  for (int i = 0; i < 20000; i++)
  {
    f.update(0.0f, 0.0f, 0.0f, 0.0f, std::sin(roll), std::cos(roll));
  }

  const auto e = f.getEuler();
  TEST_ASSERT_FLOAT_WITHIN(0.03f, roll, e.x);
  TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.0f, e.y);
}

// ---------------------------------------------------------------- Mahony ---

void test_mahony_level_converges()
{
  Mahony f;
  f.begin(1000.0f);
  f.setKp(5.0f);
  f.setKi(0.0f);

  for (int i = 0; i < 20000; i++)
  {
    f.update(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
  }

  const auto e = f.getEuler();
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, e.x);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, e.y);
  assert_quaternion_finite(f.getQuaternion());
}

void test_mahony_pitch_converges()
{
  const float pitch = 20.0f * PI_F / 180.0f;
  Mahony f;
  f.begin(1000.0f);
  f.setKp(5.0f);
  f.setKi(0.0f);

  for (int i = 0; i < 20000; i++)
  {
    f.update(0.0f, 0.0f, 0.0f, -std::sin(pitch), 0.0f, std::cos(pitch));
  }

  const auto e = f.getEuler();
  TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.0f, e.x);
  TEST_ASSERT_FLOAT_WITHIN(0.03f, pitch, e.y);
}

// -------------------------------------------------------- Complementary ---

void test_complementary_follows_position()
{
  Complementary f;
  f.begin(1000.0f, 0.1f, 0.0f);

  float out = 0.0f;
  for (int i = 0; i < 10000; i++)
  {
    out = f.update(0.0f, 5.0f); // zero rate, steady position
  }
  // with no rate the state must relax to the measured position
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 5.0f, out);
}

void test_complementary_integrates_rate()
{
  const float tau = 0.5f;
  const float rate = 2.0f;
  Complementary f;
  f.begin(1000.0f, tau, 0.0f);

  float out = 0.0f;
  for (int i = 0; i < 20000; i++)
  {
    out = f.update(rate, 0.0f); // constant rate, zero position
  }
  // steady state of the complementary filter: state = rate * tau
  TEST_ASSERT_FLOAT_WITHIN(0.02f, rate * tau, out);
}

void test_complementary_initial_state()
{
  Complementary f;
  f.begin(1000.0f, 0.1f, 3.0f);
  // first sample with matching position/zero rate keeps the initial state
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.0f, f.update(0.0f, 3.0f));
}

// ---------------------------------------------------------------- Kalman ---

void test_kalman_tracks_static_angle()
{
  Kalman f;
  f.setAngle(0.0f);

  float angle = 0.0f;
  for (int i = 0; i < 5000; i++)
  {
    angle = f.getAngle(45.0f, 0.0f, 0.001f); // measured 45 deg, no rate
  }
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 45.0f, angle);
}

void test_kalman_rejects_bias()
{
  Kalman f;
  f.setAngle(0.0f);

  // constant gyro bias with a steady accel angle: estimate must stay near the
  // measured angle rather than drift away with the integrated rate
  float angle = 0.0f;
  for (int i = 0; i < 10000; i++)
  {
    angle = f.getAngle(10.0f, 5.0f, 0.001f);
  }
  TEST_ASSERT_FLOAT_WITHIN(2.0f, 10.0f, angle);
  TEST_ASSERT_TRUE(std::isfinite(angle));
}

// ------------------------------------------------------------ Quaternion ---

// rotation of +90 deg about the Z axis
static Quaternion rotZ90()
{
  return Quaternion(std::cos(PI_F / 4.0f), 0.0f, 0.0f, std::sin(PI_F / 4.0f));
}

void test_quaternion_product_identity()
{
  const Quaternion q(0.5f, 0.5f, 0.5f, 0.5f);
  const Quaternion id; // (1,0,0,0)
  const Quaternion r = q * id;
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, q.w, r.w);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, q.x, r.x);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, q.y, r.y);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, q.z, r.z);
}

void test_quaternion_product_compose()
{
  // two 90 deg rotations about Z compose into a 180 deg rotation: (0,0,0,1)
  const Quaternion r = rotZ90() * rotZ90();
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, r.w);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, r.x);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, r.y);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, r.z);
}

void test_quaternion_conjugate_inverse()
{
  const Quaternion q = rotZ90();
  // q * conj(q) == identity for a unit quaternion
  const Quaternion r = q * q.getConjugate();
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, r.w);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, r.x);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, r.y);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, r.z);
}

void test_quaternion_normalize()
{
  Quaternion q(0.0f, 0.0f, 3.0f, 4.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, q.getMagnitude());

  const Quaternion n = q.getNormalized();
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, n.getMagnitude());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.6f, n.y);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.8f, n.z);
}

void test_quaternion_dot_and_sign()
{
  const Quaternion a(1.0f, 0.0f, 0.0f, 0.0f);
  const Quaternion b(-1.0f, 0.0f, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, -1.0f, Quaternion::dot(a, b));

  // ensureSign flips b so it lies in the same hemisphere as the reference
  const Quaternion s = Quaternion::ensureSign(b, a);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, s.w);
  // already aligned -> unchanged
  const Quaternion s2 = Quaternion::ensureSign(a, a);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, s2.w);
}

void test_quaternion_lerp_endpoints_and_mid()
{
  const Quaternion a; // identity
  const Quaternion b = rotZ90();

  const Quaternion e0 = Quaternion::lerp(a, b, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, a.w, e0.w);

  const Quaternion e1 = Quaternion::lerp(a, b, 1.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, b.w, e1.w);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, b.z, e1.z);

  const Quaternion m = Quaternion::lerp(a, b, 0.5f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, m.getMagnitude()); // stays unit
}

void test_quaternion_slerp_midpoint()
{
  const Quaternion a; // identity
  const Quaternion b = rotZ90();

  // endpoints
  TEST_ASSERT_FLOAT_WITHIN(0.001f, a.w, Quaternion::slerp(a, b, 0.0f).w);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, b.w, Quaternion::slerp(a, b, 1.0f).w);

  // halfway between identity and 90 deg about Z is a 45 deg rotation about Z
  const Quaternion m = Quaternion::slerp(a, b, 0.5f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, std::cos(PI_F / 8.0f), m.w);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, std::sin(PI_F / 8.0f), m.z);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, m.getMagnitude());
}

void test_quaternion_slerp_shortest_path()
{
  const Quaternion a; // identity
  const Quaternion b = rotZ90();
  const Quaternion bNeg(-b.w, -b.x, -b.y, -b.z); // same rotation, opposite sign

  // slerp must take the shorter arc, so both endpoints give the same result
  const Quaternion m1 = Quaternion::slerp(a, b, 0.5f);
  const Quaternion m2 = Quaternion::slerp(a, bNeg, 0.5f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, m1.w, m2.w);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, m1.z, m2.z);
}

void test_quaternion_angle_vector_roundtrip()
{
  const float angle = PI_F / 3.0f; // 60 deg
  VectorFloat axis(0.0f, 0.0f, 1.0f);

  Quaternion q;
  q.fromAngleVector(angle, axis);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, std::cos(angle / 2.0f), q.w);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, std::sin(angle / 2.0f), q.z);

  float outAngle = 0.0f;
  VectorFloat outAxis;
  q.toAngleVector(outAngle, outAxis);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, angle, outAngle);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, outAxis.z);
}

// ----------------------------------------------------------- VectorFloat ---

void test_vector_magnitude_normalize()
{
  const VectorFloat v(3.0f, 4.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, v.getMagnitude());

  const VectorFloat n = v.getNormalized();
  TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.6f, n.x);
  TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.8f, n.y);
  TEST_ASSERT_FLOAT_WITHIN(0.002f, 1.0f, n.getMagnitude());
}

void test_vector_dot_cross()
{
  const VectorFloat x(1.0f, 0.0f, 0.0f);
  const VectorFloat y(0.0f, 1.0f, 0.0f);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, x.dot(y));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, x.dot(x));

  const VectorFloat c = x.cross(y); // +X x +Y = +Z
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, c.x);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, c.y);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, c.z);
}

void test_vector_rotate_z90()
{
  // rotating +X by +90 deg about Z yields +Y
  const VectorFloat r = VectorFloat(1.0f, 0.0f, 0.0f).getRotated(rotZ90());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, r.x);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, r.y);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, r.z);
}

void test_vector_rotate_identity_preserves()
{
  const Quaternion id;
  const VectorFloat v(1.0f, -2.0f, 3.0f);
  const VectorFloat r = v.getRotated(id);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, v.x, r.x);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, v.y, r.y);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, v.z, r.z);
}

void test_vector_accel_to_euler()
{
  // level
  VectorFloat e = VectorFloat(0.0f, 0.0f, 1.0f).accelToEuler();
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, e.x);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, e.y);

  // pure roll 30 deg -> accel (0, sin, cos)
  const float roll = 30.0f * PI_F / 180.0f;
  e = VectorFloat(0.0f, std::sin(roll), std::cos(roll)).accelToEuler();
  TEST_ASSERT_FLOAT_WITHIN(0.001f, roll, e.x);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, e.y);

  // pure pitch 20 deg -> accel (-sin, 0, cos)
  const float pitch = 20.0f * PI_F / 180.0f;
  e = VectorFloat(-std::sin(pitch), 0.0f, std::cos(pitch)).accelToEuler();
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, e.x);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, pitch, e.y);
}

void test_vector_euler_quaternion_roundtrip()
{
  const VectorFloat euler(0.3f, -0.2f, 0.1f);
  const Quaternion q = euler.eulerToQuaternion();
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, q.getMagnitude());

  VectorFloat back;
  back.eulerFromQuaternion(q);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, euler.x, back.x);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, euler.y, back.y);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, euler.z, back.z);
}

void test_vector_accel_to_quaternion_level()
{
  // gravity along Z needs no rotation from the (0,0,1) reference
  const Quaternion q = VectorFloat(0.0f, 0.0f, 1.0f).accelToQuaternion();
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, std::fabs(q.w));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, q.x);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, q.y);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, q.z);
}

void test_vector_shortest_arc_maps_dst_to_src()
{
  const VectorFloat from(1.0f, 0.0f, 0.0f);
  const VectorFloat to(0.0f, 1.0f, 0.0f);
  const Quaternion q = VectorFloat::shortestArc(from, to);

  const VectorFloat r = from.getRotated(q);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, to.x, r.x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, to.y, r.y);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, to.z, r.z);
}

int main(int argc, char** argv)
{
  (void)argc;
  (void)argv;

  UNITY_BEGIN();

  RUN_TEST(test_rtqf_zero_error_stable);
  RUN_TEST(test_rtqf_level_converges);
  RUN_TEST(test_rtqf_roll_tracks);
  RUN_TEST(test_rtqf_pitch_tracks);
  RUN_TEST(test_rtqf_stays_normalized_under_motion);

  RUN_TEST(test_madgwick_level_converges);
  RUN_TEST(test_madgwick_roll_converges);

  RUN_TEST(test_mahony_level_converges);
  RUN_TEST(test_mahony_pitch_converges);

  RUN_TEST(test_complementary_follows_position);
  RUN_TEST(test_complementary_integrates_rate);
  RUN_TEST(test_complementary_initial_state);

  RUN_TEST(test_kalman_tracks_static_angle);
  RUN_TEST(test_kalman_rejects_bias);

  RUN_TEST(test_quaternion_product_identity);
  RUN_TEST(test_quaternion_product_compose);
  RUN_TEST(test_quaternion_conjugate_inverse);
  RUN_TEST(test_quaternion_normalize);
  RUN_TEST(test_quaternion_dot_and_sign);
  RUN_TEST(test_quaternion_lerp_endpoints_and_mid);
  RUN_TEST(test_quaternion_slerp_midpoint);
  RUN_TEST(test_quaternion_slerp_shortest_path);
  RUN_TEST(test_quaternion_angle_vector_roundtrip);

  RUN_TEST(test_vector_magnitude_normalize);
  RUN_TEST(test_vector_dot_cross);
  RUN_TEST(test_vector_rotate_z90);
  RUN_TEST(test_vector_rotate_identity_preserves);
  RUN_TEST(test_vector_accel_to_euler);
  RUN_TEST(test_vector_euler_quaternion_roundtrip);
  RUN_TEST(test_vector_accel_to_quaternion_level);
  RUN_TEST(test_vector_shortest_arc_maps_dst_to_src);

  return UNITY_END();
}
