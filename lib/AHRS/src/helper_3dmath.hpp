#pragma once

#include <cmath>
#include <cstdint>

#ifdef ESP32
#include <esp_attr.h>
#define FAST_CODE_IMU_ATTR IRAM_ATTR
#else
#define FAST_CODE_IMU_ATTR
#endif

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
inline float invSqrt(float x)
{
  // return 1.f / sqrt(x);
  static_assert(sizeof(float) == sizeof(int32_t));
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#pragma GCC diagnostic ignored "-Wuninitialized"
  float halfx = 0.5f * x;
  float y = x;
  int32_t i = *(int32_t*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
#pragma GCC diagnostic pop
  return y;
}

template<typename T>
class VectorBase;

class Quaternion
{
public:
  float w = 1.f;
  float x = 0.f;
  float y = 0.f;
  float z = 0.f;

  Quaternion() = default;
  Quaternion(const Quaternion&) = default;
  Quaternion(Quaternion&&) noexcept = default;
  Quaternion& operator=(const Quaternion&) = default;
  Quaternion& operator=(Quaternion&&) noexcept = default;
  ~Quaternion() = default;

  Quaternion(float nw, float nx, float ny, float nz): w(nw), x(nx), y(ny), z(nz) {}

  /**
   * @brief Returns the product of this quaternion and another quaternion.
   * @param q The other quaternion.
   * @return Quaternion The product.
   */
  Quaternion getProduct(const Quaternion& q) const
  {
    // Quaternion multiplication is defined by:
    //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
    //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
    //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
    //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2)
    return Quaternion(w * q.w - x * q.x - y * q.y - z * q.z,  // new w
                      w * q.x + x * q.w + y * q.z - z * q.y,  // new x
                      w * q.y - x * q.z + y * q.w + z * q.x,  // new y
                      w * q.z + x * q.y - y * q.x + z * q.w); // new z
  }

  /**
   * @brief Multiplies this quaternion by another quaternion.
   * @param q The other quaternion.
   * @return Quaternion The product.
   */
  Quaternion operator*(const Quaternion& q) const
  {
    return getProduct(q);
  }

  /**
   * @brief Multiplies this quaternion by another quaternion in place.
   * @param q The other quaternion.
   * @return Quaternion& A reference to this quaternion.
   */
  Quaternion& operator*=(const Quaternion& q)
  {
    *this = getProduct(q);
    return *this;
  }

  /**
   * @brief Returns the conjugate of this quaternion.
   * @return Quaternion The conjugate.
   */
  Quaternion getConjugate() const
  {
    return Quaternion(w, -x, -y, -z);
  }

  /**
   * @brief Returns the magnitude of this quaternion.
   * @return float The magnitude.
   */
  float getMagnitude() const
  {
    return sqrt(w * w + x * x + y * y + z * z);
  }

  /**
   * @brief Normalizes this quaternion in place.
   */
  void normalize()
  {
    float m = invSqrt(w * w + x * x + y * y + z * z);
    (*this) *= m;
  }

  /**
   * @brief Returns a normalized copy of this quaternion.
   * @return Quaternion The normalized quaternion.
   */
  Quaternion getNormalized() const
  {
    Quaternion r(w, x, y, z);
    r.normalize();
    return r;
  }

  float get(size_t i) const
  {
    return i == 0 ? w : (i == 1 ? x : (i == 2 ? y : (i == 3 ? z : 0)));
  }

  /**
   * @brief Multiplies this quaternion by a scalar.
   * @param v The scalar.
   * @return Quaternion The product.
   */
  Quaternion operator*(float v) const
  {
    return {w * v, x * v, y * v, z * v};
  }

  /**
   * @brief Multiplies this quaternion by a scalar in place.
   * @param v The scalar.
   * @return Quaternion& A reference to this quaternion.
   */
  Quaternion& operator*=(float v)
  {
    w *= v;
    x *= v;
    y *= v;
    z *= v;
    return *this;
  }

  /**
   * @brief Divides this quaternion by a scalar.
   * @param v The scalar.
   * @return Quaternion The quotient.
   */
  Quaternion operator/(float v) const
  {
    return {w / v, x / v, y / v, z / v};
  }

  /**
   * @brief Adds this quaternion to another quaternion.
   * @param q The other quaternion.
   * @return Quaternion The sum.
   */
  Quaternion operator+(const Quaternion& q) const
  {
    return {w + q.w, x + q.x, y + q.y, z + q.z};
  }

  /**
   * @brief Computes the dot product of two quaternions.
   * @param q1 The first quaternion.
   * @param q2 The second quaternion.
   * @return float The dot product.
   */
  float static dot(const Quaternion& q1, const Quaternion& q2)
  {
    return q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
  }

  /**
   * @brief Ensures the sign of the quaternion is consistent with the reference quaternion.
   * @param q The quaternion to check.
   * @param reference The reference quaternion.
   * @return Quaternion The quaternion with the correct sign.
   */
  Quaternion static ensureSign(const Quaternion& q, const Quaternion& reference)
  {
    if (dot(q, reference) < 0.0f)
    {
      return q * -1.f;
    }
    return q;
  }

  /**
   * Linear interpolation
   * actually it is nlerp (normalised lerp)
   * @param q1 The first quaternion (normalized).
   * @param q2 The second quaternion (normalized).
   * @param t The interpolation parameter.
   * @return Quaternion The interpolated quaternion.
   */
  Quaternion static lerp(const Quaternion& q1, const Quaternion& q2, float t)
  {
    return (q1 * (1.f - t) + ensureSign(q2, q1) * t).getNormalized();
  }

  /**
   * Spherical linear interpolation, references:
   * http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/
   * https://keithmaggio.wordpress.com/2011/02/15/math-magician-lerp-slerp-and-nlerp/
   * http://number-none.com/product/Understanding%20Slerp,%20Then%20Not%20Using%20It/
   * https://en.wikipedia.org/wiki/Slerp
   * @param q1 The first quaternion (normalized).
   * @param q2 The second quaternion (normalized).
   * @param pc The interpolation parameter.
   * @return Quaternion The interpolated quaternion.
   */
  Quaternion static slerp(const Quaternion& q1, const Quaternion& q2, float pc)
  {
    auto qb = q2;

    // If the dot product is negative, the quaternions
    // have opposite handed-ness and slerp won't take
    // the shorter path. Fix by reversing one quaternion.
    float cosHalfTheta = dot(q1, qb);
    if (cosHalfTheta < 0)
    {
      qb = qb * -1.f;
      cosHalfTheta = -cosHalfTheta;
    }

    // if qa = qb or qa =- qb then theta = 0 and we can return qa
    if (std::fabs(cosHalfTheta) >= 0.995f)
    {
      return lerp(q1, qb, pc);
    }

    // Calculate temporary values.
    float halfTheta = acosf(cosHalfTheta);
    float sinHalfTheta = sqrtf(1.0 - cosHalfTheta * cosHalfTheta);

    // if theta = 180 degrees then result is not fully defined
    // we could rotate around any axis normal to q1 or q2
    if (std::fabs(sinHalfTheta) < 0.001)
    {
      return (q1 + qb) / 2.f;
    }

    // calculate result
    float ra = sinf((1.f - pc) * halfTheta) / sinHalfTheta;
    float rb = sinf(pc * halfTheta) / sinHalfTheta;

    return q1 * ra + qb * rb;
  }

  /**
   * @brief Converts this quaternion to an angle-axis representation.
   * @param angle The angle in radians.
   * @param v The axis vector.
   */
  template<typename T>
  void toAngleVector(float& angle, VectorBase<T>& v) const
  {
    float halfTheta = acosf(w);
    float sinHalfTheta = sinf(halfTheta);
    angle = 2.0f * halfTheta;
    if (sinHalfTheta == 0)
    {
      v.x = 1.0;
      v.y = 0;
      v.z = 0;
    }
    else
    {
      v.x = x / sinHalfTheta;
      v.y = y / sinHalfTheta;
      v.z = z / sinHalfTheta;
    }
  }

  /**
   * @brief Creates a quaternion from an angle and axis.
   * @param angle The angle in radians.
   * @param v The axis vector.
   */
  template<typename T>
  void fromAngleVector(float angle, const VectorBase<T>& v)
  {
    float halfAngle = angle * 0.5f;
    float sinHalfTheta = sinf(halfAngle);
    w = cosf(halfAngle);
    x = v.x * sinHalfTheta;
    y = v.y * sinHalfTheta;
    z = v.z * sinHalfTheta;
  }

  /**
   * @brief Creates a quaternion from an angular velocity vector and time step.
   * @param v The angular velocity vector.
   * @param dt The time step.
   */
  template<typename T>
  void fromAngularVelocity(const VectorBase<T>& v, float dt)
  {
    float theta = v.getMagnitude() * dt;
    fromAngleVector(theta, v.getNormalized());
  }
};

template<typename T>
class VectorBase
{
public:
  T x = T{};
  T y = T{};
  T z = T{};

  VectorBase() = default;
  VectorBase(const VectorBase& o) = default;
  VectorBase(VectorBase&& o) noexcept = default;
  VectorBase& operator=(const VectorBase& o) = default;
  VectorBase& operator=(VectorBase&& o) noexcept = default;
  ~VectorBase() = default;

  VectorBase(T nx, T ny, T nz): x{nx}, y{ny}, z{nz} {}

  T get(size_t i) const
  {
    return i == 0 ? x : (i == 1 ? y : (i == 2 ? z : T()));
  }

  T operator[](size_t i) const
  {
    return get(i);
  }

  void set(size_t i, T v)
  {
    i == 0 ? x = v : (i == 1 ? y = v : (i == 2 ? z = v : false));
  }

  operator VectorBase<float>() const
  {
    return VectorBase<float>(x, y, z);
  }

  /**
   * @brief Returns the magnitude of this vector.
   * @return float The magnitude.
   */
  float getMagnitude() const
  {
    return sqrtf(x * x + y * y + z * z);
  }

  /**
   * @brief Normalizes this vector in place.
   * @return VectorBase<T>& A reference to this vector.
   */
  VectorBase<T>& normalize()
  {
    float m = invSqrt(x * x + y * y + z * z);
    (*this) *= m;
    return *this;
  }

  /**
   * @brief Returns a new vector that is normalized.
   * @return VectorBase<T> The normalized vector.
   */
  VectorBase<T> getNormalized() const
  {
    VectorBase<T> r(x, y, z);
    r.normalize();
    return r;
  }

  /**
   * @brief Rotates this vector by the given quaternion.
   * @param q The quaternion to rotate by.
   */
  void rotate(const Quaternion& q)
  {
    // http://www.cprogramming.com/tutorial/3d/quaternions.html
    // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
    // http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation
    // ^ or:
    // http://webcache.googleusercontent.com/search?q=cache:xgJAp3bDNhQJ:content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation&hl=en&gl=us&strip=1

    // P_out = q * P_in * conj(q)
    // - P_out is the output vector
    // - q is the orientation quaternion
    // - P_in is the input vector (a*aReal)
    // - conj(q) is the conjugate of the orientation quaternion (q=[w,x,y,z], q*=[w,-x,-y,-z])
    Quaternion p(0, x, y, z);

    // quaternion multiplication: q * p, stored back in p
    // p = q.getProduct(p);

    // quaternion multiplication: p * conj(q), stored back in p
    // p = p.getProduct(q.getConjugate());

    p = q * p * q.getConjugate();

    // p quaternion is now [0, x', y', z']
    x = p.x;
    y = p.y;
    z = p.z;
  }

  /**
   * @brief Returns a new vector that is rotated by the given quaternion.
   * @param q The quaternion to rotate by.
   * @return VectorBase<T> The rotated vector.
   */
  VectorBase<T> getRotated(const Quaternion& q) const
  {
    VectorBase<T> r(x, y, z);
    r.rotate(q);
    return r;
  }

  /**
   * @brief Calculates the dot product of two vectors.
   * @param a The first vector.
   * @param b The second vector.
   * @return float The dot product.
   */
  float static dotProduct(const VectorBase<T>& a, const VectorBase<T>& b)
  {
    return a.x * b.x + a.y * b.y + a.z * b.z;
  }

  /**
   * @brief Calculates the cross product of two vectors.
   * @param a The first vector.
   * @param b The second vector.
   * @return VectorBase<T> The cross product.
   */
  VectorBase<T> static crossProduct(const VectorBase<T>& a, const VectorBase<T>& b)
  {
    VectorBase<T> r;
    r.x = a.y * b.z - a.z * b.y;
    r.y = a.z * b.x - a.x * b.z;
    r.z = a.x * b.y - a.y * b.x;
    return r;
  }

  /**
   * @brief Calculates the dot product of this vector with another vector.
   * @param v The other vector.
   * @return float The dot product.
   */
  float dot(const VectorBase<T>& v) const
  {
    return dotProduct(*this, v);
  }

  /**
   * @brief Calculates the cross product of this vector with another vector.
   * @param v The other vector.
   * @return VectorBase<T> The cross product.
   */
  VectorBase<T> cross(const VectorBase<T>& v) const
  {
    return crossProduct(*this, v);
  }

  /**
   * @brief Converts acceleration vector to Euler angles.
   * @return VectorBase<T> The Euler angles.
   */
  VectorBase<T> accelToEuler() const
  {
    VectorBase<T> na = getNormalized();
    T tx = atan2f(na.y, na.z);
    T ty = -atan2f(na.x, sqrt(na.y * na.y + na.z * na.z));
    T tz = 0.f;
    return VectorBase<T>(tx, ty, tz);
  }

  /**
   * @brief Converts acceleration vector to quaternion.
   * Uses the shortest arc rotation from the Z-axis to the acceleration vector.
   * @return Quaternion
   */
  Quaternion accelToQuaternion() const
  {
    return shortestArc(*this, VectorBase<T>(T(0), T(0), T(1)), 1.0f);
  }

  /**
   * @brief Calculates the shortest arc rotation from one vector to another.
   * @param from The starting vector.
   * @param to The target vector.
   * @param gain The gain to apply to the rotation angle.
   * @return Quaternion The shortest arc rotation.
   */
  static Quaternion shortestArc(const VectorBase<T>& from, const VectorBase<T>& to, float gain = 1.f)
  {
    auto from_n = from.getNormalized();
    auto to_n = to.getNormalized();

    float angle = acosf(from_n.dot(to_n));
    auto v = from_n.cross(to_n).getNormalized();

    Quaternion q;
    q.fromAngleVector(angle * gain, v);
    return q.getNormalized();
  }

  /**
   * @brief Converts Euler angles to quaternion.
   * @return Quaternion
   */
  Quaternion eulerToQuaternion() const
  {
    Quaternion q;
    float cosX2 = cosf(x / 2.0f);
    float sinX2 = sinf(x / 2.0f);
    float cosY2 = cosf(y / 2.0f);
    float sinY2 = sinf(y / 2.0f);
    float cosZ2 = cosf(z / 2.0f);
    float sinZ2 = sinf(z / 2.0f);

    q.w = cosX2 * cosY2 * cosZ2 + sinX2 * sinY2 * sinZ2;
    q.x = sinX2 * cosY2 * cosZ2 - cosX2 * sinY2 * sinZ2;
    q.y = cosX2 * sinY2 * cosZ2 + sinX2 * cosY2 * sinZ2;
    q.z = cosX2 * cosY2 * sinZ2 - sinX2 * sinY2 * cosZ2;
    q.normalize();
    return q;
  }

  /**
   * @brief Converts quaternion to Euler angles.
   * @param q The quaternion to convert.
   * @return VectorBase<T> The Euler angles.
   */
  VectorBase<T> eulerFromQuaternion(const Quaternion& q)
  {
    // x = atan2f(2.0f * (q.y * q.z + q.w * q.x), 1.f - 2.0f * (q.x * q.x + q.y * q.y));
    // y =  asinf(2.0f * (q.w * q.y - q.x * q.z));
    // z = atan2f(2.0f * (q.x * q.y + q.w * q.z), 1.f - 2.0f * (q.y * q.y + q.z * q.z));
    x = atan2f(q.w * q.x + q.y * q.z, 0.5f - q.x * q.x - q.y * q.y);
    y = asinf(-2.0f * (q.x * q.z - q.w * q.y));
    z = atan2f(q.x * q.y + q.w * q.z, 0.5f - q.y * q.y - q.z * q.z);
    return *this;
  }

  // vector arithmetics
  VectorBase<T>& operator+=(const VectorBase<T>& o)
  {
    x += o.x;
    y += o.y;
    z += o.z;
    return *this;
  }

  VectorBase<T> operator+(const VectorBase<T>& o)
  {
    VectorBase<T> r(*this);
    r += o;
    return r;
  }

  VectorBase<T>& operator-=(const VectorBase<T>& o)
  {
    x -= o.x;
    y -= o.y;
    z -= o.z;
    return *this;
  }

  VectorBase<T> operator-(const VectorBase<T>& o)
  {
    VectorBase<T> r(*this);
    r -= o;
    return r;
  }

  VectorBase<T>& operator*=(const VectorBase<T>& o)
  {
    x *= o.x;
    y *= o.y;
    z *= o.z;
    return *this;
  }

  VectorBase<T> operator*(const VectorBase<T>& o)
  {
    VectorBase<T> r(*this);
    r *= o;
    return r;
  }

  // scalar operations
  VectorBase<T>& operator*=(T o)
  {
    x *= o;
    y *= o;
    z *= o;
    return *this;
  }

  VectorBase<T> operator*(T o)
  {
    VectorBase<T> r(*this);
    r *= o;
    return r;
  }

  VectorBase<T>& operator/=(T o)
  {
    x /= o;
    y /= o;
    z /= o;
    return *this;
  }

  VectorBase<T> operator/(T o)
  {
    VectorBase<T> r(*this);
    r /= o;
    return r;
  }
};

template<typename T>
class RotationMatrix
{
public:
  RotationMatrix() = default;

  void init(const VectorBase<T>& v)
  {
    const T cosx = cosf(v.x);
    const T sinx = sinf(v.x);
    const T cosy = cosf(v.y);
    const T siny = sinf(v.y);
    const T cosz = cosf(v.z);
    const T sinz = sinf(v.z);

    const T coszcosx = cosz * cosx;
    const T sinzcosx = sinz * cosx;
    const T coszsinx = sinx * cosz;
    const T sinzsinx = sinx * sinz;

    _m[0][0] = cosz * cosy;
    _m[0][1] = -cosy * sinz;
    _m[0][2] = siny;
    _m[1][0] = sinzcosx + (coszsinx * siny);
    _m[1][1] = coszcosx - (sinzsinx * siny);
    _m[1][2] = -sinx * cosy;
    _m[2][0] = (sinzsinx) - (coszcosx * siny);
    _m[2][1] = (coszsinx) + (sinzcosx * siny);
    _m[2][2] = cosy * cosx;
  }

  VectorBase<T> apply(const VectorBase<T>& v)
  {
    const T x = _m[0][0] * v.x + _m[1][0] * v.y + _m[2][0] * v.z;
    const T y = _m[0][1] * v.x + _m[1][1] * v.y + _m[2][1] * v.z;
    const T z = _m[0][2] * v.x + _m[1][2] * v.y + _m[2][2] * v.z;
    return VectorBase<T>{x, y, z};
  }

private:
  T _m[3][3] = {
      {T{1}, T{0}, T{0}},
      {T{0}, T{1}, T{0}},
      {T{0}, T{0}, T{1}},
  };
};

using VectorFloat = VectorBase<float>;
using VectorInt16 = VectorBase<int16_t>;
using RotationMatrixFloat = RotationMatrix<float>;
