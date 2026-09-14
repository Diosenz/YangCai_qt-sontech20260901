/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_teach_trajectory_recorder.cpp
 * @brief       teach trajectory recorder 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "TeachTrajectoryRecorder.h"

namespace {

namespace control = i2r_rlc::control;
namespace robot = i2r_rlc::robot_comm;
namespace fs = std::filesystem;

void require(bool condition, const char *message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

fs::path makeTempDirectory()
{
    const fs::path root = fs::temp_directory_path()
        / "i2r_rlc_teach_trajectory_recorder_tests";
    fs::remove_all(root);
    fs::create_directories(root);
    return root;
}

robot::RobotState makeState(
    const double xMm,
    const double yMm,
    const double zMm,
    const long saveSignal)
{
    robot::RobotState state;
    state.s_Bsavetraj = saveSignal;
    state.curx_r = static_cast<long>(xMm * 1000.0);
    state.cury_r = static_cast<long>(yMm * 1000.0);
    state.curz_r = static_cast<long>(zMm * 1000.0);
    state.curRx_r = 10000;
    state.curRy_r = 20000;
    state.curRz_r = 30000;
    state.curS1 = 40000;
    state.curS2 = 50000;
    return state;
}

std::vector<std::string> readLines(const fs::path &path)
{
    std::ifstream stream(path);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(stream, line)) {
        if (!line.empty()) {
            lines.push_back(line);
        }
    }
    return lines;
}

void testRecordBySaveSignalEdges()
{
    const fs::path root = makeTempDirectory();
    control::CTeachTrajectoryRecorder recorder(root / "temp");
    auto update = recorder.startRecording();
    require(update.succeeded, "startRecording should create and clear temp files");

    update = recorder.onRobotStateChanged(makeState(10.0, 0.0, 0.0, 1));
    require(update.trajectoryPointCount == 1, "first save point should append W");
    require(update.teachPointCount == 1, "first save point should append T");

    update = recorder.onRobotStateChanged(makeState(10.3, 0.0, 0.0, 1));
    require(update.trajectoryPointCount == 1,
            "distance <= 0.5 mm should not append W");
    require(update.teachPointCount == 1,
            "distance <= 0.5 mm should not append T");

    update = recorder.onRobotStateChanged(makeState(11.0, 0.0, 0.0, 1));
    require(update.trajectoryPointCount == 2,
            "distance > 0.5 mm should append W");
    require(update.teachPointCount == 1,
            "distance <= 2 mm should not append extra T");

    update = recorder.onRobotStateChanged(makeState(13.5, 0.0, 0.0, 1));
    require(update.trajectoryPointCount == 3,
            "large move should append another W");
    require(update.teachPointCount == 2,
            "distance > 2 mm should append T");

    update = recorder.onRobotStateChanged(makeState(14.0, 0.0, 0.0, 0));
    require(update.trajectoryPointCount == 4,
            "save signal falling edge should append final W");
    require(update.teachPointCount == 3,
            "save signal falling edge should append final T");
    require(!update.segmentActive, "falling edge should release segment latch");

    const auto trajectoryLines = readLines(root / "temp" / "SeamTraj.txt");
    const auto teachLines = readLines(root / "temp" / "SeamTeach.txt");
    require(trajectoryLines.size() == 4, "SeamTraj line count should match W count");
    require(teachLines.size() == 3, "SeamTeach line count should match T count");
    require(trajectoryLines.front().find("10.000 0.000 0.000") == 0,
            "SeamTraj should write mm coordinates");
    require(teachLines.front().find("10.000 0.000 0.000 1.0000") == 0,
            "SeamTeach should write deg pose");
}

void testAllPtsAndSaveNoOverwrite()
{
    const fs::path root = makeTempDirectory();
    control::CTeachTrajectoryRecorder recorder(root / "temp");
    recorder.startRecording();

    auto state = makeState(10.0, 1.0, 2.0, 1);
    state.sStart = 1;
    auto update = recorder.onRobotStateChanged(state);
    require(update.allPointCount == 1, "sStart==1 should append AllPts");
    recorder.onRobotStateChanged(makeState(12.5, 1.0, 2.0, 1));
    recorder.onRobotStateChanged(makeState(13.0, 1.0, 2.0, 0));

    const auto allLines = readLines(root / "temp" / "AllPts.txt");
    require(allLines.size() == 1, "AllPts should have one line");
    require(allLines.front() == "10000 1000 2000 10000 20000 30000 40000 50000",
            "AllPts should keep old integer protocol units");

    update = recorder.saveCapturedTrajectory(
        control::TeachTrajectorySaveRequest{ (root / "WorkPiece").string(), "TESTTRAJ" });
    require(update.succeeded, "saveCapturedTrajectory should copy W/T");
    require(fs::exists(root / "WorkPiece" / "TESTTRAJ_W.txt"),
            "saved W file should exist");
    require(fs::exists(root / "WorkPiece" / "TESTTRAJ_T.txt"),
            "saved T file should exist");

    update = recorder.saveCapturedTrajectory(
        control::TeachTrajectorySaveRequest{ (root / "WorkPiece").string(), "TESTTRAJ" });
    require(!update.succeeded, "saving existing target should fail without overwrite");

    update = recorder.saveCapturedTrajectory(
        control::TeachTrajectorySaveRequest{ (root / "WorkPiece").string(), "BAD_W" });
    require(!update.succeeded, "base name with _W suffix should fail");
}

} // namespace

int main()
{
    try {
        testRecordBySaveSignalEdges();
        testAllPtsAndSaveNoOverwrite();
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }

    std::cout << "PASS: teach trajectory recorder tests\n";
    return 0;
}
