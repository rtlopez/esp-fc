#ifndef _HELPER_3DMATH_H_
#define _HELPER_3DMATH_H_

#include <cmath>
#include <cstdint>

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
inline float invSqrt(float x)
{
  //return 1.f / sqrt(x);
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
  #pragma GCC diagnostic ignored "-Wuninitialized"
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  #pragma GCC diagnostic pop
  return y;
}

template<typename T>
class VectorBase;

class Quaternion {
    public:
        float w;
        float x;
        float y;
        float z;

        Quaternion(): w(1.f), x(0.f), y(0.f), z(0.f) {}

        Quaternion(float nw, float nx, float ny, float nz): w(nw), x(nx), y(ny), z(nz) {}

        Quaternion getProduct(const Quaternion& q) const {
            // Quaternion multiplication is defined by:
            //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
            //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
            //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
            //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2)
            return Quaternion(
                w*q.w - x*q.x - y*q.y - z*q.z,  // new w
                w*q.x + x*q.w + y*q.z - z*q.y,  // new x
                w*q.y - x*q.z + y*q.w + z*q.x,  // new y
                w*q.z + x*q.y - y*q.x + z*q.w); // new z
        }

        Quaternion operator*(const Quaternion& q) const {
          return getProduct(q);
        }

        Quaternion& operator*=(const Quaternion& q) {
          *this = getProduct(q);
          return *this;
        }

        Quaternion getConjugate() const {
          return Quaternion(w, -x, -y, -z);
        }

        float getMagnitude() const {
          return sqrt(w * w + x * x + y * y + z * z);
        }

        void normalize() {
          float m = invSqrt(w * w + x * x + y * y + z * z);
          (*this) *= m;
        }

        Quaternion getNormalized() const {
            Quaternion r(w, x, y, z);
            r.normalize();
            return r;
        }

        float get(size_t i) const {
          return i == 0 ? w :
                (i == 1 ? x :
                (i == 2 ? y :
                (i == 3 ? z :
                 0
          )));
        }

        Quaternion operator*(float v) const {
          return Quaternion(w * v, x * v, y * v, z * v);
        }

        Quaternion& operator*=(float v) {
          w *= v;
          x *= v;
          y *= v;
          z *= v;
          return *this;
        }

        Quaternion operator/(float v) const {
          return Quaternion(w / v, x / v, y / v, z / v);
        }

        Quaternion operator+(const Quaternion& q) const {
          return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
        }

        float static dot(const Quaternion& q1, const Quaternion& q2) {
          return q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
        }

        /**
         * Linear interpolation
         * actually it is nlerp (normalised lerp)
         */
        Quaternion static lerp(const Quaternion &q1, const Quaternion &q2, float t)
        {
          return (q1 * (1.f - t) + q2 * t).getNormalized();
        }

        /**
         * Spherical linear interpolation, references:
         * http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/
         * https://keithmaggio.wordpress.com/2011/02/15/math-magician-lerp-slerp-and-nlerp/
         * http://number-none.com/product/Understanding%20Slerp,%20Then%20Not%20Using%20It/
         * https://en.wikipedia.org/wiki/Slerp
         */
        Quaternion static slerp(const Quaternion& q1, const Quaternion& q2, float pc)
        {
          Quaternion qa = q1.getNormalized();
          Quaternion qb = q2.getNormalized();

          // If the dot product is negative, the quaternions
          // have opposite handed-ness and slerp won't take
          // the shorter path. Fix by reversing one quaternion.
          float cosHalfTheta = dot(qa, qb);
          if(cosHalfTheta < 0)
          {
            qb = qb * -1.f;
            cosHalfTheta = -cosHalfTheta;
          }

          // if qa = qb or qa =- qb then theta = 0 and we can return qa
          if(std::fabs(cosHalfTheta) >= 0.995f)
          {
            return lerp(qa, qb, pc);
          }

          // Calculate temporary values.
          float halfTheta = acosf(cosHalfTheta);
          float sinHalfTheta = sqrtf(1.0 - cosHalfTheta * cosHalfTheta);

          // if theta = 180 degrees then result is not fully defined
          // we could rotate around any axis normal to q1 or q2
          if(std::fabs(sinHalfTheta) < 0.001)
          {
            return (qa + qb) / 2.f;
          }

          // calculate result
          float ra = sinf((1.f - pc) * halfTheta) / sinHalfTheta;
          float rb = sinf(pc * halfTheta) / sinHalfTheta;

          return qa * ra + qb * rb;
        }

        template<typename T>
        void toAngleVector(float& angle, VectorBase<T>& v) const
        {
          float halfTheta = acosf(w);
          float sinHalfTheta = sinf(halfTheta);
          angle = 2.0f * halfTheta;
          if (sinHalfTheta == 0) {
            v.x = 1.0;
            v.y = 0;
            v.z = 0;
          } else {
            v.x = x / sinHalfTheta;
            v.y = y / sinHalfTheta;
            v.z = z / sinHalfTheta;
          }
        }

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

        template<typename T>
        void fromAngularVelocity(const VectorBase<T>& v, float dt)
        {
          float theta = v.getMagnitude() * dt;
          fromAngleVector(theta, v.getNormalized());
        }
};

template<typename T>
class VectorBase {
  public:
    T x;
    T y;
    T z;

    VectorBase<T>(): x(), y(), z() {}
    VectorBase<T>(T nx, T ny, T nz): x(nx), y(ny), z(nz) {}
    VectorBase<T>(const VectorBase<T>& o): x(o.x), y(o.y), z(o.z) {}

    VectorBase<T>& operator =(const VectorBase<T>& o) {
      if(this == &o) return *this;
      x = o.x;
      y = o.y;
      z = o.z;
      return *this;
    }

    T get(size_t i) const { return i == 0 ? x : (i == 1 ? y : (i == 2 ? z : T())); }
    T operator[](size_t i) const { return get(i); }

    void set(size_t i, T v) { i == 0 ? x = v : (i == 1 ? y = v : (i == 2 ? z = v : false)); }

    operator VectorBase<float>() const {
      return VectorBase<float>(x, y, z);
    }

    float getMagnitude() const {
      return sqrtf(x * x + y * y + z * z);
    }

    VectorBase<T>& normalize() {
      float m = invSqrt(x * x + y * y + z * z);
      (*this) *= m;
      return *this;
    }

    VectorBase<T> getNormalized() const {
      VectorBase<T> r(x, y, z);
      r.normalize();
      return r;
    }

    void rotate(const Quaternion& q) {
        // http://www.cprogramming.com/tutorial/3d/quaternions.html
        // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
        // http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation
        // ^ or: http://webcache.googleusercontent.com/search?q=cache:xgJAp3bDNhQJ:content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation&hl=en&gl=us&strip=1

        // P_out = q * P_in * conj(q)
        // - P_out is the output vector
        // - q is the orientation quaternion
        // - P_in is the input vector (a*aReal)
        // - conj(q) is the conjugate of the orientation quaternion (q=[w,x,y,z], q*=[w,-x,-y,-z])
        Quaternion p(0, x, y, z);

        // quaternion multiplication: q * p, stored back in p
        //p = q.getProduct(p);

        // quaternion multiplication: p * conj(q), stored back in p
        //p = p.getProduct(q.getConjugate());

        p = q * p * q.getConjugate();

        // p quaternion is now [0, x', y', z']
        x = p.x;
        y = p.y;
        z = p.z;
    }

    VectorBase<T> getRotated(const Quaternion& q) const {
        VectorBase<T> r(x, y, z);
        r.rotate(q);
        return r;
    }

    float static dotProduct(const VectorBase<T>& a, const VectorBase<T>& b)
    {
      return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    VectorBase<T> static crossProduct(const VectorBase<T>& a, const VectorBase<T>& b)
    {
      VectorBase<T> r;
      r.x = a.y * b.z - a.z * b.y;
      r.y = a.z * b.x - a.x * b.z;
      r.z = a.x * b.y - a.y * b.x;
      return r;
    }

    float dot(const VectorBase<T>& v) const
    {
      return dotProduct(*this, v);
    }

    VectorBase<T> cross(const VectorBase<T>& v) const
    {
      return crossProduct(*this, v);
    }

    VectorBase<T> accelToEuler() const
    {
      VectorBase<T> rpy; // roll pitch yaw
      VectorBase<T> na = getNormalized();
      rpy.x =  atan2f(na.y, na.z);
      rpy.y = -atan2f(na.x, sqrt(na.y * na.y + na.z * na.z));
      rpy.z = 0.f;
      return rpy;
    }

    Quaternion accelToQuaternion() const
    {
       /*VectorBase<T> ref(0, 0, 1);
       VectorBase<T> na = getNormalized();
       float angle = acosf(ref.dot(na));
       VectorBase<T> v = na.cross(ref).getNormalized();
       Quaternion q;
       q.fromAngleVector(angle, v);
       q.normalize();
       return q;*/
       return diffVectors(VectorBase<T>(T(0), T(0), T(1)), *this, 1.0f);
    }

    static Quaternion diffVectors(const VectorBase<T>& src, const VectorBase<T>& dst, float gain = 1.f)
    {
       VectorBase<T> src_n = src.getNormalized();
       VectorBase<T> dst_n = dst.getNormalized();
       Quaternion q;

       float angle = acosf(dst_n.dot(src_n));
       VectorBase<T> v = dst_n.cross(src_n).getNormalized();
       q.fromAngleVector(angle * gain, v);
       return q.getNormalized();
    }

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

    VectorBase<T> eulerFromQuaternion(const Quaternion& q)
    {
      //x = atan2f(2.0f * (q.y * q.z + q.w * q.x), 1.f - 2.0f * (q.x * q.x + q.y * q.y));
      //y =  asinf(2.0f * (q.w * q.y - q.x * q.z));
      //z = atan2f(2.0f * (q.x * q.y + q.w * q.z), 1.f - 2.0f * (q.y * q.y + q.z * q.z));
      x = atan2f(q.w * q.x + q.y * q.z, 0.5f - q.x * q.x - q.y * q.y);
      y = asinf(-2.0f * (q.x * q.z - q.w * q.y));
      z = atan2f(q.x * q.y + q.w * q.z, 0.5f - q.y * q.y - q.z * q.z);
      return *this;
    }

    // vector arithmetics
    VectorBase<T>& operator+=(const VectorBase<T>& o) {
      x += o.x;
      y += o.y;
      z += o.z;
      return *this;
    }

    VectorBase<T> operator+(const VectorBase<T>& o) {
      VectorBase<T> r(*this);
      r += o;
      return r;
    }

    VectorBase<T>& operator-=(const VectorBase<T>& o) {
      x -= o.x;
      y -= o.y;
      z -= o.z;
      return *this;
    }

    VectorBase<T> operator-(const VectorBase<T>& o) {
      VectorBase<T> r(*this);
      r -= o;
      return r;
    }

    VectorBase<T>& operator*=(const VectorBase<T>& o) {
      x *= o.x;
      y *= o.y;
      z *= o.z;
      return *this;
    }

    VectorBase<T> operator*(const VectorBase<T>& o) {
      VectorBase<T> r(*this);
      r *= o;
      return r;
    }

    // scalar operations
    VectorBase<T>& operator*=(T o) {
      x *= o;
      y *= o;
      z *= o;
      return *this;
    }

    VectorBase<T> operator*(T o) {
      VectorBase<T> r(*this);
      r *= o;
      return r;
    }

    VectorBase<T>& operator/=(T o) {
      x /= o;
      y /= o;
      z /= o;
      return *this;
    }

    VectorBase<T> operator/(T o) {
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
    { T{1}, T{0}, T{0} },
    { T{0}, T{1}, T{0} },
    { T{0}, T{0}, T{1} },
  };
};

typedef VectorBase<float> VectorFloat;
typedef VectorBase<int16_t> VectorInt16;
typedef RotationMatrix<float> RotationMatrixFloat;

#endif /* _HELPER_3DMATH_H_ */
