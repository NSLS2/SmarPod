/**
 * TestSmarPodMain.cpp
 *
 * Unit tests for SmarPod areaDetector driver.
 *
 * Author: Jakub Wlodek
 *
 * Copyright (c): Brookhaven National Laboratory 2025
 */
#include <gmock/gmock.h>
#include <gtest/gtest.h>

class PrintRequestsAndReturnsOnFailure : public ::testing::EmptyTestEventListener {
        virtual void OnTestStart(const ::testing::TestInfo& /*test_info*/) override {
            // Start capturing output
            ::testing::internal::CaptureStdout();
            ::testing::internal::CaptureStderr();
        }

        virtual void OnTestEnd(const ::testing::TestInfo& test_info) override {
            std::string standardOutput = ::testing::internal::GetCapturedStdout();
            std::string standardError = ::testing::internal::GetCapturedStderr();
            if (test_info.result()->Failed()) {
                if (!standardOutput.empty()) {
                    std::cout << "Captured Standard Output:\n" << standardOutput << std::endl;
                }

                if (!standardError.empty()) {
                    std::cerr << "Captured Standard Error:\n" << standardError << std::endl;
                }
            }
        }
};

// Main function to run tests
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);

    // Instantiate our custom listener
    ::testing::TestEventListeners& listeners = ::testing::UnitTest::GetInstance()->listeners();
    listeners.Append(new PrintRequestsAndReturnsOnFailure());

    return RUN_ALL_TESTS();
}
