// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class, 3D math helper
// 6/5/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2012-06-05 - add 3D math helper file to DMP6 example sketch

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _HELPER_3DMATH_H_
#define _HELPER_3DMATH_H_

#include "Arduino.h"
#include <math.h>

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
            //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
            return Quaternion(
                w*q.w - x*q.x - y*q.y - z*q.z,  // new w
                w*q.x + x*q.w + y*q.z - z*q.y,  // new x
                w*q.y - x*q.z + y*q.w + z*q.x,  // new y
                w*q.z + x*q.y - y*q.x + z*q.w); // new z
        }

        Quaternion operator*(const Quaternion& q) const {
          return this->getProduct(q);
        }

        Quaternion getConjugate() const {
            return Quaternion(w, -x, -y, -z);
        }

        float getMagnitude() const {
            return sqrt(w * w + x * x + y * y + z * z);
        }

        void normalize() {
            float m = getMagnitude();
            w /= m;
            x /= m;
            y /= m;
            z /= m;
        }

        Quaternion getNormalized() const {
            Quaternion r(w, x, y, z);
            r.normalize();
            return r;
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
      return sqrt(x * x + y * y + z * z);
    }

    void normalize() {
        float m = getMagnitude();
        x /= m;
        y /= m;
        z /= m;
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

    VectorBase<T> accelToEuler() const
    {
      VectorBase<T> rpy; // roll pitch yaw
      VectorBase<T> na = this->getNormalized();
      rpy.x = atan2(na.y, na.z);
      rpy.y = -atan2(na.x, sqrt(na.y * na.y + na.z * na.z));
      rpy.z = 0.f;
      return rpy;
    }

    Quaternion eulerToQuaternion() const
    {
      Quaternion q;
      float cosX2 = cos(x / 2.0f);
      float sinX2 = sin(x / 2.0f);
      float cosY2 = cos(y / 2.0f);
      float sinY2 = sin(y / 2.0f);
      float cosZ2 = cos(z / 2.0f);
      float sinZ2 = sin(z / 2.0f);

      q.w = cosX2 * cosY2 * cosZ2 + sinX2 * sinY2 * sinZ2;
      q.x = sinX2 * cosY2 * cosZ2 - cosX2 * sinY2 * sinZ2;
      q.y = cosX2 * sinY2 * cosZ2 + sinX2 * cosY2 * sinZ2;
      q.z = cosX2 * cosY2 * sinZ2 - sinX2 * sinY2 * cosZ2;
      q.normalize();
      return q;
    }

    VectorBase<T> eulerFromQuaternion(const Quaternion& q)
    {
      VectorBase<T> vec;
      x = atan2(2.0 * (q.y * q.z + q.w * q.x), 1 - 2.0 * (q.x * q.x + q.y * q.y));
      y = asin(2.0 * (q.w * q.y - q.x * q.z));
      z = atan2(2.0 * (q.x * q.y + q.w * q.z), 1 - 2.0 * (q.y * q.y + q.z * q.z));
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

typedef VectorBase<float> VectorFloat;
typedef VectorBase<int16_t> VectorInt16;

#endif /* _HELPER_3DMATH_H_ */
