/**
 * TestSimHexapodAPI.cpp
 *
 * Behavioural unit tests for the simulated hexapod API (SimHexapod).
 *
 * Copyright (c): Brookhaven National Laboratory 2026
 */
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

#include "TestSimHexaodAPI.hpp"

using namespace std::chrono_literals;

namespace {
constexpr double kEps = 1e-9;

Pose makePose(double x, double y, double z, double rx, double ry, double rz) {
    return {x, y, z, rx, ry, rz};
}

void expectPoseNear(const Pose& actual, const Pose& expected, double tol) {
    EXPECT_NEAR(actual.x, expected.x, tol);
    EXPECT_NEAR(actual.y, expected.y, tol);
    EXPECT_NEAR(actual.z, expected.z, tol);
    EXPECT_NEAR(actual.rx, expected.rx, tol);
    EXPECT_NEAR(actual.ry, expected.ry, tol);
    EXPECT_NEAR(actual.rz, expected.rz, tol);
}
}  // namespace

//---------------------------------------------------------------------------
// Defaults and simple state
//---------------------------------------------------------------------------
TEST_F(TestSimHexapod, DefaultsAreReasonable) {
    auto [speed, speedControl] = hex->GetSpeed();
    EXPECT_DOUBLE_EQ(speed, 0.001);
    EXPECT_TRUE(speedControl);

    auto [accel, accelControl] = hex->GetAcceleration();
    EXPECT_DOUBLE_EQ(accel, 0.01);
    EXPECT_FALSE(accelControl);

    EXPECT_EQ(hex->GetMaxFrequency(), 18500u);
    EXPECT_DOUBLE_EQ(hex->GetFindRefAndCalibFreq(), 8000.0);
    EXPECT_EQ(hex->GetSensorMode(), SensorMode::POWERSAVE);
    EXPECT_EQ(hex->GetPivotMode(), PivotMode::RELATIVE);
    EXPECT_EQ(hex->GetFindRefMethod(), FrefMethod::DEFAULT);
    EXPECT_EQ(hex->GetLocator(), "sim:123456");
    EXPECT_FALSE(hex->IsReferenced());
    EXPECT_EQ(hex->GetMoveStatus(), MoveStatus::STOPPED);
}

TEST_F(TestSimHexapod, GetPoseThrowsWhenNotReferenced) {
    EXPECT_THROW(hex->GetPose(), std::runtime_error);
}

TEST_F(TestSimHexapod, FindReferenceMarksEnablesPose) {
    ASSERT_FALSE(hex->IsReferenced());
    makeFast();
    hex->FindReferenceMarks();
    EXPECT_TRUE(hex->IsReferenced());
    EXPECT_EQ(hex->GetMoveStatus(), MoveStatus::STOPPED);

    Pose p{};
    ASSERT_NO_THROW(p = hex->GetPose());
    expectPoseNear(p, makePose(0, 0, 0, 0, 0, 0), kEps);
}

TEST_F(TestSimHexapod, CalibrateCompletesWithoutThrowing) {
    reference();
    EXPECT_NO_THROW(hex->Calibrate());
    EXPECT_EQ(hex->GetMoveStatus(), MoveStatus::STOPPED);
}

//---------------------------------------------------------------------------
// Setter / getter round trips
//---------------------------------------------------------------------------
TEST_F(TestSimHexapod, SpeedRoundTrip) {
    hex->SetSpeed(0.5, false);
    auto [speed, control] = hex->GetSpeed();
    EXPECT_DOUBLE_EQ(speed, 0.5);
    EXPECT_FALSE(control);
}

TEST_F(TestSimHexapod, AccelerationRoundTrip) {
    hex->SetAcceleration(0.25, true);
    auto [accel, control] = hex->GetAcceleration();
    EXPECT_DOUBLE_EQ(accel, 0.25);
    EXPECT_TRUE(control);
}

TEST_F(TestSimHexapod, MaxFrequencyRoundTrip) {
    hex->SetMaxFrequency(1000);
    EXPECT_EQ(hex->GetMaxFrequency(), 1000u);
}

TEST_F(TestSimHexapod, SensorModeRoundTrip) {
    hex->SetSensorMode(SensorMode::ENABLED);
    EXPECT_EQ(hex->GetSensorMode(), SensorMode::ENABLED);
    hex->SetSensorMode(SensorMode::DISABLED);
    EXPECT_EQ(hex->GetSensorMode(), SensorMode::DISABLED);
}

TEST_F(TestSimHexapod, PivotRoundTrip) {
    hex->SetPivot(0.01, -0.02, 0.03);
    auto [x, y, z] = hex->GetPivot();
    EXPECT_DOUBLE_EQ(x, 0.01);
    EXPECT_DOUBLE_EQ(y, -0.02);
    EXPECT_DOUBLE_EQ(z, 0.03);
}

TEST_F(TestSimHexapod, PivotModeRoundTrip) {
    hex->SetPivotMode(PivotMode::FIXED);
    EXPECT_EQ(hex->GetPivotMode(), PivotMode::FIXED);
}

TEST_F(TestSimHexapod, FindRefMethodRoundTrip) {
    hex->SetFindRefMethod(FrefMethod::ZSAFE);
    EXPECT_EQ(hex->GetFindRefMethod(), FrefMethod::ZSAFE);
}

TEST_F(TestSimHexapod, FindRefAndCalibFreqRoundTrip) {
    hex->SetFindRefAndCalibFreq(1234.0);
    EXPECT_DOUBLE_EQ(hex->GetFindRefAndCalibFreq(), 1234.0);
}

TEST_F(TestSimHexapod, AxesOrientationRoundTrip) {
    hex->SetAxesOrientation(1.0, 2.0, 3.0);
    auto [rx, ry, rz] = hex->GetAxesOrientation();
    EXPECT_DOUBLE_EQ(rx, 1.0);
    EXPECT_DOUBLE_EQ(ry, 2.0);
    EXPECT_DOUBLE_EQ(rz, 3.0);
}

TEST_F(TestSimHexapod, AxesOrientationEnforcesAngleRanges) {
    EXPECT_NO_THROW(hex->SetAxesOrientation(180.0, 60.0, -180.0));
    EXPECT_THROW(hex->SetAxesOrientation(181.0, 0, 0), std::runtime_error);   // rx > 180
    EXPECT_THROW(hex->SetAxesOrientation(0, 61.0, 0), std::runtime_error);    // ry > 60
    EXPECT_THROW(hex->SetAxesOrientation(0, 0, -181.0), std::runtime_error);  // rz < -180
}

TEST_F(TestSimHexapod, AxesOrientationAndCoordSystemAreMutuallyExclusive) {
    // Axes orientation may only be set when CS and pivot are zero.
    hex->SetCoordinateSystem(makePose(0.01, 0, 0, 0, 0, 0));
    EXPECT_THROW(hex->SetAxesOrientation(5.0, 0, 0), std::runtime_error);

    // Clearing the CS allows an axes orientation, which then blocks CS/pivot.
    hex->SetCoordinateSystem(makePose(0, 0, 0, 0, 0, 0));
    ASSERT_NO_THROW(hex->SetAxesOrientation(5.0, 0, 0));
    EXPECT_THROW(hex->SetCoordinateSystem(makePose(0.01, 0, 0, 0, 0, 0)), std::runtime_error);
    EXPECT_THROW(hex->SetPivot(0.01, 0, 0), std::runtime_error);
}

TEST_F(TestSimHexapod, PivotBlockedByNonZeroAxesOrientation) {
    hex->SetAxesOrientation(0, 10.0, 0);
    EXPECT_THROW(hex->SetPivot(0.01, 0, 0), std::runtime_error);
}

TEST_F(TestSimHexapod, CoordinateSystemRoundTrip) {
    Pose cs = makePose(0.01, 0.02, 0.03, 4, 5, 6);
    hex->SetCoordinateSystem(cs);
    expectPoseNear(hex->GetCoordinateSystem(), cs, kEps);
}

TEST_F(TestSimHexapod, CoordinateSystemAnglesBeyond45Throw) {
    EXPECT_THROW(hex->SetCoordinateSystem(makePose(0, 0, 0, 46, 0, 0)), std::runtime_error);
    EXPECT_THROW(hex->SetCoordinateSystem(makePose(0, 0, 0, 0, -46, 0)), std::runtime_error);
    EXPECT_THROW(hex->SetCoordinateSystem(makePose(0, 0, 0, 0, 0, 90)), std::runtime_error);
    EXPECT_NO_THROW(hex->SetCoordinateSystem(makePose(0, 0, 0, 45, -45, 45)));
}

TEST_F(TestSimHexapod, FindRefDirectionIsPerAxis) {
    hex->SetFindRefDirection(Axis::X, FrefDirection::POSITIVE);
    hex->SetFindRefDirection(Axis::Y, FrefDirection::NEGATIVE);
    hex->SetFindRefDirection(Axis::Z, FrefDirection::REVERSE);
    EXPECT_EQ(hex->GetFindRefDirection(Axis::X), FrefDirection::POSITIVE);
    EXPECT_EQ(hex->GetFindRefDirection(Axis::Y), FrefDirection::NEGATIVE);
    EXPECT_EQ(hex->GetFindRefDirection(Axis::Z), FrefDirection::REVERSE);
}

//---------------------------------------------------------------------------
// Workspace / reachability
//---------------------------------------------------------------------------
TEST_F(TestSimHexapod, PoseWithinWorkspaceIsReachable) {
    EXPECT_TRUE(hex->IsPoseReachable(makePose(0.05, -0.05, 0.05, 10, -10, 15)));
}

TEST_F(TestSimHexapod, PoseOutsideLinearRangeIsUnreachable) {
    EXPECT_FALSE(hex->IsPoseReachable(makePose(0.2, 0, 0, 0, 0, 0)));
}

TEST_F(TestSimHexapod, PoseOutsideAngularRangeIsUnreachable) {
    EXPECT_FALSE(hex->IsPoseReachable(makePose(0, 0, 0, 30, 0, 0)));
}

TEST_F(TestSimHexapod, CoordinateSystemShiftsReachableRange) {
    hex->SetCoordinateSystem(makePose(0.08, 0, 0, 0, 0, 0));
    EXPECT_FALSE(hex->IsPoseReachable(makePose(0.05, 0, 0, 0, 0, 0)));  // 0.13 m > 0.1 m
    EXPECT_TRUE(hex->IsPoseReachable(makePose(-0.05, 0, 0, 0, 0, 0)));  // 0.03 m within
}

TEST_F(TestSimHexapod, AxesOrientationShiftsAngularRange) {
    hex->SetAxesOrientation(15.0, 0, 0);
    EXPECT_FALSE(hex->IsPoseReachable(makePose(0, 0, 0, 10, 0, 0)));  // 25 deg > 20 deg
    EXPECT_TRUE(hex->IsPoseReachable(makePose(0, 0, 0, -10, 0, 0)));  // 5 deg within
}

//---------------------------------------------------------------------------
// Move guard conditions
//---------------------------------------------------------------------------
TEST_F(TestSimHexapod, MoveThrowsWhenNotReferenced) {
    EXPECT_THROW(hex->Move(makePose(0.01, 0, 0, 0, 0, 0), 0, true), std::runtime_error);
}

TEST_F(TestSimHexapod, MoveThrowsWhenSensorsDisabled) {
    reference();
    hex->SetSensorMode(SensorMode::DISABLED);
    EXPECT_THROW(hex->Move(makePose(0.01, 0, 0, 0, 0, 0), 0, true), std::runtime_error);
}

TEST_F(TestSimHexapod, MoveThrowsWhenTargetUnreachable) {
    reference();
    EXPECT_THROW(hex->Move(makePose(0.5, 0, 0, 0, 0, 0), 0, true), std::runtime_error);
}

//---------------------------------------------------------------------------
// Blocking moves
//---------------------------------------------------------------------------
TEST_F(TestSimHexapod, BlockingMoveReachesTargetAndStops) {
    reference();
    hex->SetSpeed(1.0, true);  // fast, to keep the test short
    Pose target = makePose(0.02, -0.01, 0.03, 5, -5, 10);
    hex->Move(target, 0, true);
    expectPoseNear(hex->GetPose(), target, 1e-6);
    EXPECT_EQ(hex->GetMoveStatus(), MoveStatus::STOPPED);
}

TEST_F(TestSimHexapod, BlockingMoveWithInfiniteHoldEntersHolding) {
    reference();
    hex->SetSpeed(1.0, true);
    hex->Move(makePose(0.02, 0, 0, 0, 0, 0), 60000, true);  // 60000 ms == infinite hold
    EXPECT_EQ(hex->GetMoveStatus(), MoveStatus::HOLDING);
}

//---------------------------------------------------------------------------
// Non-blocking motion model
//---------------------------------------------------------------------------
TEST_F(TestSimHexapod, NonBlockingMoveInterpolatesThenCompletes) {
    reference();
    hex->SetSpeed(0.1, true);  // 0.1 m/s -> ~0.5 s for a 0.05 m move
    Pose target = makePose(0.05, 0, 0, 0, 0, 0);
    hex->Move(target, 0, false);
    EXPECT_EQ(hex->GetMoveStatus(), MoveStatus::MOVING);

    std::this_thread::sleep_for(200ms);
    Pose mid = hex->GetPose();
    EXPECT_GT(mid.x, 0.0);
    EXPECT_LT(mid.x, target.x);

    std::this_thread::sleep_for(600ms);  // well past the move end
    expectPoseNear(hex->GetPose(), target, 1e-6);
    EXPECT_EQ(hex->GetMoveStatus(), MoveStatus::STOPPED);
}

TEST_F(TestSimHexapod, StopFreezesPoseMidMove) {
    reference();
    hex->SetSpeed(0.1, true);
    hex->Move(makePose(0.05, 0, 0, 0, 0, 0), 0, false);

    std::this_thread::sleep_for(100ms);
    hex->Stop();
    EXPECT_EQ(hex->GetMoveStatus(), MoveStatus::STOPPED);

    Pose frozen = hex->GetPose();
    EXPECT_GT(frozen.x, 0.0);
    EXPECT_LT(frozen.x, 0.05);

    std::this_thread::sleep_for(100ms);
    EXPECT_NEAR(hex->GetPose().x, frozen.x, kEps);  // no further motion
}

TEST_F(TestSimHexapod, StopAndHoldEntersHolding) {
    reference();
    hex->StopAndHold(60000);
    EXPECT_EQ(hex->GetMoveStatus(), MoveStatus::HOLDING);
}

TEST_F(TestSimHexapod, StandbyEntersStandby) {
    reference();
    hex->Standby();
    EXPECT_EQ(hex->GetMoveStatus(), MoveStatus::STANDBY);
}

TEST_F(TestSimHexapod, SetCurrentPoseAsZeroResetsPose) {
    reference();
    hex->SetSpeed(1.0, true);
    hex->Move(makePose(0.02, 0.02, 0, 0, 0, 0), 0, true);
    hex->SetCurrentPoseAsZero();
    expectPoseNear(hex->GetPose(), makePose(0, 0, 0, 0, 0, 0), kEps);
}

//---------------------------------------------------------------------------
// Coordinate system / pivot shift the reported pose
//---------------------------------------------------------------------------
TEST_F(TestSimHexapod, CoordinateSystemTranslationOffsetsReportedPose) {
    reference();
    expectPoseNear(hex->GetPose(), makePose(0, 0, 0, 0, 0, 0), kEps);

    // A pure coordinate-system translation shifts the reported pose by its
    // negative (ct term): t = C^T*(d - ct) with C = I, d = 0.
    hex->SetCoordinateSystem(makePose(0.01, -0.02, 0.03, 0, 0, 0));
    expectPoseNear(hex->GetPose(), makePose(-0.01, 0.02, -0.03, 0, 0, 0), 1e-9);
}

TEST_F(TestSimHexapod, CoordinateSystemReportedPoseRoundTrips) {
    reference();
    hex->SetSpeed(1.0, true);
    // GetPose is expressed in the active coordinate system, so a completed move
    // reports the commanded target back even with a rotated/translated CS.
    hex->SetCoordinateSystem(makePose(0.005, 0, 0, 0, 0, 10));
    Pose target = makePose(0.01, -0.005, 0.002, 3, -2, 4);
    hex->Move(target, 0, true);
    expectPoseNear(hex->GetPose(), target, 1e-6);
}

TEST_F(TestSimHexapod, PivotDoesNotAffectReportedPoseWithoutRotation) {
    reference();
    // With no rotation the pivot term (R*p - p) vanishes, so a pivot change
    // leaves the reported pose unchanged.
    hex->SetPivot(0.01, 0.02, 0.03);
    expectPoseNear(hex->GetPose(), makePose(0, 0, 0, 0, 0, 0), kEps);
}

TEST_F(TestSimHexapod, PivotReportedPoseRoundTripsUnderRotation) {
    reference();
    hex->SetSpeed(1.0, true);
    hex->SetPivot(0.01, 0.0, 0.0);
    Pose target = makePose(0.005, 0, 0, 0, 8, 0);
    hex->Move(target, 0, true);
    expectPoseNear(hex->GetPose(), target, 1e-6);
}

TEST_F(TestSimHexapod, SetCurrentPoseAsZeroZeroesReportedPoseWithOffsets) {
    reference();
    hex->SetSpeed(1.0, true);
    hex->Move(makePose(0.02, 0, 0, 0, 0, 0), 0, true);
    hex->SetPivot(0.01, 0, 0);
    hex->SetCoordinateSystem(makePose(0.005, 0, 0, 0, 0, 0));
    hex->SetCurrentPoseAsZero();
    expectPoseNear(hex->GetPose(), makePose(0, 0, 0, 0, 0, 0), kEps);
}

TEST_F(TestSimHexapod, SetCurrentPoseAsZeroBlockedByNonZeroAxesOrientation) {
    reference();
    hex->SetAxesOrientation(5.0, 0, 0);
    EXPECT_THROW(hex->SetCurrentPoseAsZero(), std::runtime_error);
}

//---------------------------------------------------------------------------
// Factory-level behaviour
//---------------------------------------------------------------------------
TEST(TestSimHexapodAPI, ReportsVersionModelsAndLocator) {
    SimHexapodAPI api{"sim:abc"};

    auto [major, minor, update] = api.GetVersion();
    EXPECT_EQ(major, 1);
    EXPECT_EQ(minor, 0);
    EXPECT_EQ(update, 0);

    EXPECT_EQ(api.GetModelName(0), "Simulated Hexapod");

    auto models = api.GetSupportedModels();
    ASSERT_EQ(models.size(), 1u);
    EXPECT_EQ(models[0], 0u);

    auto systems = api.FindSystems();
    ASSERT_EQ(systems.size(), 1u);
    EXPECT_EQ(systems[0], "sim:abc");

    EXPECT_EQ(api.GetStatusMessage(StatusCode::OK), "OK");
    EXPECT_NE(api.GetStatusMessage(StatusCode::OTHER_ERROR), "OK");
}

TEST(TestSimHexapodAPI, OpenReturnsUsableHexapod) {
    SimHexapodAPI api{"sim:xyz"};
    auto hex = api.Open(0, "sim:xyz");
    ASSERT_NE(hex, nullptr);
    EXPECT_FALSE(hex->IsReferenced());
    EXPECT_EQ(hex->GetMoveStatus(), MoveStatus::STOPPED);
}
