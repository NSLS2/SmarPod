#ifndef TEST_SIMHEXAPODAPI_H
#define TEST_SIMHEXAPODAPI_H

#include <gtest/gtest.h>

#include <memory>

#include "SimHexapodAPI.hpp"

// Opens a fresh simulated hexapod for each test.
class TestSimHexapod : public ::testing::Test {
    protected:
        void SetUp() override { hex = api.Open(0, kLocator); }

        // Referencing/calibration jog every axis for a real, frequency-derived
        // duration. A very high find-ref/calibration frequency shrinks that
        // duration to sub-millisecond so tests don't sleep for ~10 s each.
        void makeFast() { hex->SetFindRefAndCalibFreq(1.0e8); }

        // The pose can only be read / a move started after referencing.
        void reference() {
            makeFast();
            hex->FindReferenceMarks();
        }

        static constexpr const char* kLocator = "sim:123456";
        SimHexapodAPI api{kLocator};
        std::unique_ptr<Hexapod> hex;
};

#endif  // TEST_SIMHEXAPODAPI_H