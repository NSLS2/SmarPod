/**
 * Small 3D rigid-body math used by the SmarPod simulator.
 *
 * Rotation convention matches the SmarPod Programmer's Guide:
 * R = Rz(rz) * Ry(ry) * Rx(rx), angles in degrees.
 *
 * Copyright (c) : Brookhaven National Laboratory, 2026
 */
#pragma once

#include <algorithm>
#include <cmath>

namespace spmath {

inline constexpr double kPi = 3.14159265358979323846;

struct Vec3 {
        double x, y, z;
};

struct Mat3 {
        double m[3][3];
};

inline Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }

inline Vec3 apply(const Mat3& r, const Vec3& v) {
    return {r.m[0][0] * v.x + r.m[0][1] * v.y + r.m[0][2] * v.z,
            r.m[1][0] * v.x + r.m[1][1] * v.y + r.m[1][2] * v.z,
            r.m[2][0] * v.x + r.m[2][1] * v.y + r.m[2][2] * v.z};
}

inline Mat3 multiply(const Mat3& a, const Mat3& b) {
    Mat3 c{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            double s = 0.0;
            for (int k = 0; k < 3; ++k) s += a.m[i][k] * b.m[k][j];
            c.m[i][j] = s;
        }
    return c;
}

inline Mat3 transpose(const Mat3& a) {
    Mat3 t{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) t.m[i][j] = a.m[j][i];
    return t;
}

// R = Rz(rz) * Ry(ry) * Rx(rx), angles in degrees.
inline Mat3 eulerToMatrix(double rxDeg, double ryDeg, double rzDeg) {
    double a = rxDeg * kPi / 180.0, b = ryDeg * kPi / 180.0, g = rzDeg * kPi / 180.0;
    double ca = std::cos(a), sa = std::sin(a);
    double cb = std::cos(b), sb = std::sin(b);
    double cg = std::cos(g), sg = std::sin(g);
    Mat3 r;
    r.m[0][0] = cb * cg;
    r.m[0][1] = sa * sb * cg - ca * sg;
    r.m[0][2] = sa * sg + ca * sb * cg;
    r.m[1][0] = cb * sg;
    r.m[1][1] = sa * sb * sg + ca * cg;
    r.m[1][2] = ca * sb * sg - sa * cg;
    r.m[2][0] = -sb;
    r.m[2][1] = sa * cb;
    r.m[2][2] = ca * cb;
    return r;
}

// Inverse of eulerToMatrix; returns (rx, ry, rz) in degrees.
inline Vec3 matrixToEuler(const Mat3& r) {
    double sb = std::max(-1.0, std::min(1.0, -r.m[2][0]));
    double b = std::asin(sb);
    double a, g;
    if (std::abs(std::cos(b)) > 1e-9) {
        a = std::atan2(r.m[2][1], r.m[2][2]);
        g = std::atan2(r.m[1][0], r.m[0][0]);
    } else {  // gimbal lock: pin rz, solve rx
        g = 0.0;
        a = std::atan2(-r.m[1][2], r.m[1][1]);
    }
    return {a * 180.0 / kPi, b * 180.0 / kPi, g * 180.0 / kPi};
}

}  // namespace spmath
