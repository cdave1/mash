#ifndef _MATH_SCRAPS_QUATERNION_H_
#define _MATH_SCRAPS_QUATERNION_H_

#include <Constants.h>
#include <Vec3.h>
#include <Functions.h>

#include <assert.h>

typedef struct Quaternion {
	Quaternion () {
        v[0] = 0.0f;
        v[1] = 0.0f;
        v[2] = 0.0f;
        v[3] = 1.0f;
    }


    Quaternion(const Vec3& axis, float theta) {
        float halfAngle = theta * 0.5f;
        Vec3 unitAxis = scraps::normalize(axis);
        float sinHalfAngle = sinf(halfAngle);

        v[0] = unitAxis[0] * sinHalfAngle;
        v[1] = unitAxis[1] * sinHalfAngle;
        v[2] = unitAxis[2] * sinHalfAngle;
        v[3] = cosf(halfAngle);
    }

    /*
    Quaternion(Axis A, float angle, RotateDirection d) {
        const float RHS = 1.0f;

        float sinHalfAngle = RHS * d * sin(angle * 0.5f);
        Vec3 vec;
        vec[0] = vec[1] = vec[2] = 0.0f;
        vec[A] = sinHalfAngle;

        v[0] = vec[0];
        v[1] = vec[1];
        v[2] = vec[2];
        v[3] = cosf(angle * 0.5f);
    }*/


    Quaternion (float x, float y, float z, float w) {
        v[0] = x;
        v[1] = y;
        v[2] = z;
        v[3] = w;
    }

/*
    static Quaternion LookRotation(const Vec3& at, const Vec3& up) {
        Vector forward = scraps::normalize(at);
        Vector::OrthoNormalize(&up, &forward); // Keeps up the same, make forward orthogonal to up
        Vector right = Vector::Cross(up, forward);

        Quaternion ret;
        ret.w = sqrtf(1.0f + right.x + up.y + forward.z) * 0.5f;
        float w4_recip = 1.0f / (4.0f * ret.w);
        ret.x = (forward.y - up.z) * w4_recip;
        ret.y = (right.z - forward.x) * w4_recip;
        ret.z = (up.x - right.y) * w4_recip;

        return ret;
    }*/


    static Quaternion LookAt(const Vec3 &eye, const Vec3 &at, const Vec3 &up) {
        Vec3 direction = scraps::normalize(at - eye);
        Vec3 forward(0, 0, 1);
        float dot = forward * direction;

        if (fabs(dot - (-1.0f)) < 0.000001f) {
            return Quaternion(up, math_pi);
        }
        if (fabs(dot - (1.0f)) < 0.000001f) {
            return Quaternion();
        }

        float rotationAngle = acosf(dot);
        Vec3 rotationAxis = scraps::cross(forward, direction);
        rotationAxis = scraps::normalize(rotationAxis);
        return Quaternion(rotationAxis, rotationAngle);
    }


    /*

     {
     float norm_u_norm_v = sqrt(dot(u, u) * dot(v, v));
     float real_part = norm_u_norm_v + dot(u, v);
     vec3 w;

     if (real_part < 1.e-6f * norm_u_norm_v)
     {
     // If u and v are exactly opposite, rotate 180 degrees
     // around an arbitrary orthogonal axis. Axis normalisation
     // can happen later, when we normalise the quaternion.
    real_part = 0.0f;
    w = abs(u.x) > abs(u.z) ? vec3(-u.y, u.x, 0.f)
    : vec3(0.f, -u.z, u.y);
}
else
{
    // Otherwise, build quaternion the standard way.
    w = cross(u, v);
}

return normalize(quat(real_part, w.x, w.y, w.z));
}
http://lolengine.net/blog/2014/02/24/quaternion-from-two-vectors-final
     */
    static Quaternion FromTwoVectors(const Vec3 &u, const Vec3 &v) {
        float norm_u_norm_v = sqrt((u * u) * (v * v));
        float real_part = norm_u_norm_v + (u * v);
        Vec3 w;
        if (real_part < 1.e-6f * norm_u_norm_v) {
            real_part = 0.0f;
            w = fabs(u[0]) > fabs(u[3]) ? Vec3(-u[1], u[0], 0.0f) : Vec3(0.0f, -u[2], u[1]);
        } else {
            w = scraps::cross(u, v);
        }
        return Quaternion(w[0], w[1], w[2], real_part);
    }


    void SetIdentity() {
        v[0] = 0.0f;
        v[1] = 0.0f;
        v[2] = 0.0f;
        v[3] = 1.0f;
    }


	void Set(float x, float y, float z, float w) {
        v[0] = x;
        v[1] = y;
        v[2] = z;
        v[3] = w;
    }


    void Set(const Quaternion &quat) {
        Set(quat[0], quat[1], quat[2], quat[3]);
    }


    void Invert() {
        v[0] = -v[0];
        v[1] = -v[1];
        v[2] = -v[2];
        v[3] = v[3];
    }

/*
    void Normalize() {
        float len = Length();
        if (len == 0.0f) {
            Set(0.0f, 0.0f, 0.0f, 1.0f);
        } else {
            len = 1.0f / len;
            Set(v[0] * len, v[1] * len, v[2] * len, v[3] * len);
        }
    }*/


    Quaternion operator *(const Quaternion &b) const {
        Quaternion t;
        t[0] = v[3] * b[0] + v[0] * b[3] + v[1] * b[2] - v[2] * b[1];
        t[1] = v[3] * b[1] - v[0] * b[2] + v[1] * b[3] + v[2] * b[0];
        t[2] = v[3] * b[2] + v[0] * b[1] - v[1] * b[0] + v[2] * b[3];
        t[3] = v[3] * b[3] - v[0] * b[0] - v[1] * b[1] - v[2] * b[2];
        return t;
    }


    ///
    /// Multiply vector by this quaternion.  Equivalent to Matrix::Vec3Multiply.
    /// via http://molecularmusings.wordpress.com/2013/05/24/a-faster-quaternion-vector-multiplication/
    ///
    Vec3 operator *(const Vec3 &vec) const {
        Vec3 q(v[0], v[1], v[2]);
        Vec3 t = 2 * scraps::cross(q, vec);
        Vec3 ret = vec + v[3] * t + scraps::cross(q, t);
        return ret;
    }


    inline float Length() const {
        return sqrt(v[0] * v[0] +
                    v[1] * v[1] +
                    v[2] * v[2] +
                    v[3] * v[3]);
    }


    inline float& operator[](unsigned int index) {
        return v[index];
    }


    inline const float& operator[](unsigned int index) const {
        return v[index];
    }

	float v[4];
} Quaternion;


inline float diff(const Quaternion &a, const Quaternion &b) {
    return a[0] == b[0] && a[1] == b[1]  && a[2] == b[2] && a[3] == b[3];
}


inline bool operator == (const Quaternion &a, const Quaternion &b) {
    return a[0] == b[0] && a[1] == b[1]  && a[2] == b[2] && a[3] == b[3];
}

#endif
