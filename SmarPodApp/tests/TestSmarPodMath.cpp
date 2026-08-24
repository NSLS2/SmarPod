/**
 * TestSmarPodMath.cpp
 *
 * Unit tests for the 3D rigid-body math helpers (spmath).
 *
 * Copyright (c): Brookhaven National Laboratory 2026
 */
#include <gtest/gtest.h>

#include <cmath>

#include "SmarPodMath.hpp"

using namespace spmath;

namespace {
constexpr double kTol = 1e-12;

Mat3 identity() { return {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}; }

void expectVec3Near(const Vec3& a, const Vec3& b, double tol) {
    EXPECT_NEAR(a.x, b.x, tol);
    EXPECT_NEAR(a.y, b.y, tol);
    EXPECT_NEAR(a.z, b.z, tol);
}

void expectMat3Near(const Mat3& a, const Mat3& b, double tol) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            EXPECT_NEAR(a.m[i][j], b.m[i][j], tol) << "at (" << i << "," << j << ")";
}
}  // namespace

//---------------------------------------------------------------------------
// Vector operators
//---------------------------------------------------------------------------
TEST(TestSmarPodMath, VectorAddSubtract) {
    Vec3 a{1.0, -2.0, 3.0};
    Vec3 b{0.5, 0.5, -1.0};
    expectVec3Near(a + b, {1.5, -1.5, 2.0}, kTol);
    expectVec3Near(a - b, {0.5, -2.5, 4.0}, kTol);
}

//---------------------------------------------------------------------------
// apply / multiply / transpose
//---------------------------------------------------------------------------
TEST(TestSmarPodMath, ApplyIdentityReturnsVector) {
    Vec3 v{0.3, -0.7, 1.1};
    expectVec3Near(apply(identity(), v), v, kTol);
}

TEST(TestSmarPodMath, ApplyKnownMatrix) {
    // Row-wise dot products.
    Mat3 a{{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}}};
    expectVec3Near(apply(a, {1.0, 0.0, -1.0}), {-2.0, -2.0, -2.0}, kTol);
}

TEST(TestSmarPodMath, MultiplyByIdentity) {
    Mat3 a{{{1, 2, 3}, {4, 5, 6}, {7, 8, 10}}};
    expectMat3Near(multiply(a, identity()), a, kTol);
    expectMat3Near(multiply(identity(), a), a, kTol);
}

TEST(TestSmarPodMath, MultiplyKnownProduct) {
    Mat3 a{{{1, 2, 0}, {0, 1, 0}, {0, 0, 1}}};
    Mat3 b{{{1, 0, 0}, {0, 1, 0}, {3, 0, 1}}};
    Mat3 expected{{{1, 2, 0}, {0, 1, 0}, {3, 0, 1}}};
    expectMat3Near(multiply(a, b), expected, kTol);
}

TEST(TestSmarPodMath, TransposeSwapsIndices) {
    Mat3 a{{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}}};
    Mat3 t = transpose(a);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) EXPECT_DOUBLE_EQ(t.m[i][j], a.m[j][i]);
    expectMat3Near(transpose(t), a, kTol);
}

TEST(TestSmarPodMath, ApplyMultiplyIsAssociative) {
    Mat3 a = eulerToMatrix(12.0, -34.0, 56.0);
    Mat3 b = eulerToMatrix(-7.0, 21.0, -3.0);
    Vec3 v{0.11, -0.22, 0.33};
    expectVec3Near(apply(multiply(a, b), v), apply(a, apply(b, v)), 1e-12);
}

//---------------------------------------------------------------------------
// eulerToMatrix
//---------------------------------------------------------------------------
TEST(TestSmarPodMath, ZeroAnglesGiveIdentity) {
    expectMat3Near(eulerToMatrix(0, 0, 0), identity(), kTol);
}

TEST(TestSmarPodMath, RotationAboutXAxis) {
    // +90 deg about X maps +Y -> +Z.
    expectVec3Near(apply(eulerToMatrix(90, 0, 0), {0, 1, 0}), {0, 0, 1}, 1e-12);
}

TEST(TestSmarPodMath, RotationAboutYAxis) {
    // +90 deg about Y maps +X -> -Z.
    expectVec3Near(apply(eulerToMatrix(0, 90, 0), {1, 0, 0}), {0, 0, -1}, 1e-12);
}

TEST(TestSmarPodMath, RotationAboutZAxis) {
    // +90 deg about Z maps +X -> +Y.
    expectVec3Near(apply(eulerToMatrix(0, 0, 90), {1, 0, 0}), {0, 1, 0}, 1e-12);
}

TEST(TestSmarPodMath, RotationsAreOrthonormal) {
    Mat3 r = eulerToMatrix(23.0, -41.0, 67.0);
    // R * R^T == I for a proper rotation.
    expectMat3Near(multiply(r, transpose(r)), identity(), 1e-12);
}

TEST(TestSmarPodMath, ComposesInZYXOrder) {
    // The guide defines R = Rz * Ry * Rx.
    Mat3 rx = eulerToMatrix(15.0, 0, 0);
    Mat3 ry = eulerToMatrix(0, -25.0, 0);
    Mat3 rz = eulerToMatrix(0, 0, 40.0);
    expectMat3Near(eulerToMatrix(15.0, -25.0, 40.0), multiply(rz, multiply(ry, rx)), 1e-12);
}

TEST(TestSmarPodMath, SameAxisRotationsAdd) {
    expectMat3Near(multiply(eulerToMatrix(0, 0, 30.0), eulerToMatrix(0, 0, 15.0)),
                   eulerToMatrix(0, 0, 45.0), 1e-12);
}

//---------------------------------------------------------------------------
// matrixToEuler
//---------------------------------------------------------------------------
TEST(TestSmarPodMath, EulerRoundTripReturnsAngles) {
    struct {
            double rx, ry, rz;
    } cases[] = {{0, 0, 0},     {10, 20, 30}, {-15, 40, -60},
                 {5, -80, 170}, {90, 0, -90}, {-179, 12, 179}};
    for (auto& c : cases) {
        Vec3 e = matrixToEuler(eulerToMatrix(c.rx, c.ry, c.rz));
        EXPECT_NEAR(e.x, c.rx, 1e-9) << "rx for (" << c.rx << "," << c.ry << "," << c.rz << ")";
        EXPECT_NEAR(e.y, c.ry, 1e-9) << "ry";
        EXPECT_NEAR(e.z, c.rz, 1e-9) << "rz";
    }
}

TEST(TestSmarPodMath, MatrixReproducedThroughEulerAtGimbalLock) {
    // At ry = +/-90 the Euler angles are ambiguous, but re-building the matrix
    // from the recovered angles must reproduce the original rotation.
    for (double ry : {90.0, -90.0}) {
        Mat3 r = eulerToMatrix(25.0, ry, 40.0);
        Vec3 e = matrixToEuler(r);
        EXPECT_NEAR(e.y, ry, 1e-9);
        expectMat3Near(eulerToMatrix(e.x, e.y, e.z), r, 1e-9);
    }
}
