#include "TrajectoryUploadWait.h"
#include "L1Robot.h"
#include "FXErrorCode.h"
#include <cassert>
#include <chrono>
#include <iostream>
#include <limits>
#include <vector>

extern FXRobotType FX_ROBOT_TYPE;

int main()
{
    using namespace std::chrono;
    using gento_upload::WaitResult;
    int elapsed = 0;
    auto now = [&]() { return steady_clock::time_point(milliseconds(elapsed)); };
    auto pause = [&]() { ++elapsed; };
    auto healthy = []() { return true; };
    auto result = gento_upload::wait_for_confirmation(
        [&]() { return elapsed >= 35; }, healthy, now, pause, milliseconds(2000));
    assert(result == WaitResult::confirmed && elapsed == 35);
    elapsed = 0;
    result = gento_upload::wait_for_confirmation(
        [&]() { return elapsed >= 10; }, healthy, now, pause, milliseconds(10));
    assert(result == WaitResult::confirmed && elapsed == 10); // Old loop rejected this ACK.
    elapsed = 0;
    result = gento_upload::wait_for_confirmation(
        []() { return false; }, healthy, now, pause, milliseconds(2000));
    assert(result == WaitResult::timeout && elapsed == 2000);
    elapsed = 0;
    result = gento_upload::wait_for_confirmation(
        []() { return false; }, [&]() { return elapsed < 7; }, now, pause, milliseconds(2000));
    assert(result == WaitResult::interrupted && elapsed == 7);

    // Invalid input must be rejected without any connection or UDP operation.
    const auto original = FX_ROBOT_TYPE;
    FX_ROBOT_TYPE = FX_ROBOT_GENTO_LUNA;
    std::vector<double> points(35, 0.);
    assert(FX_L1_Config_SetTraj(FX_OBJ_ARM0, 0, points.data()) == FUNC_RET_INVALID_INPUT_ARG);
    assert(FX_L1_Config_SetTraj(FX_OBJ_ARM0, 4, points.data()) == FUNC_RET_INVALID_INPUT_ARG);
    assert(FX_L1_Config_SetTraj(FX_OBJ_ARM1, 5001, points.data()) == FUNC_RET_INVALID_INPUT_ARG);
    assert(FX_L1_Config_SetTraj(FX_OBJ_ARM0, 5, nullptr) == FUNC_RET_INVALID_INPUT_ARG);
    points[3] = std::numeric_limits<double>::quiet_NaN();
    assert(FX_L1_Config_SetTraj(FX_OBJ_ARM1, 5, points.data()) == FUNC_RET_INVALID_INPUT_ARG);
    FX_ROBOT_TYPE = original;
    std::cout << "trajectory upload regression passed\n";
}
