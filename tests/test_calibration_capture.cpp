/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_calibration_capture.cpp
 * @brief       calibration capture 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "CalibrationCaptureController.h"

namespace fs = std::filesystem;

namespace {

bool expectTrue(bool condition, const std::string &message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

i2r_rlc::robot_comm::RobotState makeRobotState(int index)
{
    i2r_rlc::robot_comm::RobotState state;
    state.curx_r = 100000 + index;
    state.cury_r = 200000 + index;
    state.curz_r = 300000 + index;
    state.curRx_r = 10000 + index;
    state.curRy_r = 20000 + index;
    state.curRz_r = 30000 + index;
    return state;
}

cv::Mat makeImage(unsigned char value)
{
    return cv::Mat(24, 32, CV_8UC1, cv::Scalar(value));
}

std::vector<std::string> readLines(const fs::path &path)
{
    std::ifstream input(path);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(input, line)) {
        lines.push_back(line);
    }
    return lines;
}

fs::path makeCaseRoot(const std::string &name)
{
    const fs::path root = fs::temp_directory_path()
        / ("i2r_rlc_calibration_capture_" + name);
    fs::remove_all(root);
    fs::create_directories(root);
    return root;
}

i2r_rlc::control::CalibrationCaptureOptions makeOptions(const fs::path &root)
{
    i2r_rlc::control::CalibrationCaptureOptions options;
    options.outputRootDirectory = root.string();
    return options;
}

bool testManualCapture(void)
{
    using namespace i2r_rlc::control;
    const fs::path root = makeCaseRoot("manual");
    CCalibrationCaptureController controller;
    controller.configure(makeOptions(root));

    bool ok = true;
    auto update = controller.startManualCapture();
    ok &= expectTrue(update.succeeded, "manual capture should start");
    ok &= expectTrue(update.requiredBoardImageCount == 19,
        "manual capture should require 19 board images including paired positions");
    ok &= expectTrue(update.requestedExposureUs == 8000 && !update.requestLaserOn,
        "manual capture should start with board exposure and laser off");
    for (int i = 0; i < 15; ++i) {
        update = controller.captureBoardSample(makeImage(static_cast<unsigned char>(20 + i)),
            makeRobotState(i + 1));
        ok &= expectTrue(update.succeeded, "board sample should be captured");
        ok &= expectTrue(update.robotCommand.valid, "first 15 board samples should request next position");
        ok &= expectTrue(update.robotCommand.capturedSampleCount == i + 1,
            "C command count should track robot positions");
    }
    ok &= expectTrue(
        controller.state().phase == CalibrationCapturePhase::ManualCapturingBoard,
        "manual capture should keep board phase after the first 15 board images");

    for (int i = 0; i < 4; ++i) {
        update = controller.captureBoardSample(makeImage(static_cast<unsigned char>(60 + i)),
            makeRobotState(16 + i));
        ok &= expectTrue(update.succeeded, "paired board sample should be captured");
        ok &= expectTrue(!update.robotCommand.valid,
            "paired board sample should not request next position before laser image");
        ok &= expectTrue(update.requestedExposureUs == 200 && update.requestLaserOn,
            "paired board sample should request laser exposure and laser on");
        update = controller.captureLaserImage(makeImage(static_cast<unsigned char>(100 + i)),
            makeRobotState(16 + i));
        ok &= expectTrue(update.succeeded, "laser sample should be captured");
        ok &= expectTrue(update.robotCommand.valid == (i < 3),
            "paired laser sample should request next position except final point");
        if (i < 3) {
            ok &= expectTrue(update.robotCommand.capturedSampleCount == 16 + i,
                "paired laser C command count should be the completed robot position");
        }
    }
    ok &= expectTrue(
        controller.state().phase == CalibrationCapturePhase::Completed,
        "manual capture should complete after 19 board images and 4 laser images");
    ok &= expectTrue(controller.finishCapture().succeeded, "finish should succeed on complete data");

    const fs::path methodDir = root / "CALID" / "4 TWO_STEP";
    ok &= expectTrue(fs::exists(methodDir / "CalibOriginData.txt"),
        "CalibOriginData.txt should exist");
    ok &= expectTrue(fs::exists(methodDir / "CalibResult.txt"),
        "CalibResult.txt should exist");
    ok &= expectTrue(fs::exists(methodDir / "image_01.png"), "image_01.png should exist");
    ok &= expectTrue(fs::exists(methodDir / "image_15.png"), "image_15.png should exist");
    ok &= expectTrue(fs::exists(methodDir / "image_16.png"), "image_16.png should exist");
    ok &= expectTrue(fs::exists(methodDir / "image_19.png"), "image_19.png should exist");
    ok &= expectTrue(fs::exists(methodDir / "image_16L.png"), "image_16L.png should exist");
    ok &= expectTrue(fs::exists(methodDir / "image_19L.png"), "image_19L.png should exist");

    const std::vector<std::string> lines = readLines(methodDir / "CalibOriginData.txt");
    ok &= expectTrue(lines.size() == 20, "origin data should contain header plus 19 poses");
    ok &= expectTrue(lines.size() > 1 && lines[1] == "100001 200001 300001 10001 20001 30001",
        "first robot pose should be written in legacy integer units");
    ok &= expectTrue(lines.size() > 19 && lines[19] == "100019 200019 300019 10019 20019 30019",
        "last origin pose should come from paired board image, not laser image");

    const std::vector<std::string> resultLines =
        readLines(methodDir / "CalibResult.txt");
    ok &= expectTrue(resultLines.size() == 10,
        "CalibResult.txt should keep the external calibration format");
    const std::vector<std::string> expectedArrays{
        "cx[] = {100001,100002,100003,100004,100005,100006,100007,100008,100009,100010,100011,100012,100013,100014,100015,100016,100017,100018,100019,};",
        "cy[] = {200001,200002,200003,200004,200005,200006,200007,200008,200009,200010,200011,200012,200013,200014,200015,200016,200017,200018,200019,};",
        "cz[] = {300001,300002,300003,300004,300005,300006,300007,300008,300009,300010,300011,300012,300013,300014,300015,300016,300017,300018,300019,};",
        "crx[] = {10001,10002,10003,10004,10005,10006,10007,10008,10009,10010,10011,10012,10013,10014,10015,10016,10017,10018,10019,};",
        "cry[] = {20001,20002,20003,20004,20005,20006,20007,20008,20009,20010,20011,20012,20013,20014,20015,20016,20017,20018,20019,};",
        "crz[] = {30001,30002,30003,30004,30005,30006,30007,30008,30009,30010,30011,30012,30013,30014,30015,30016,30017,30018,30019,};"};
    for (std::size_t index = 0; index < expectedArrays.size(); ++index) {
        ok &= expectTrue(
            resultLines.size() > index + 3
                && resultLines[index + 3] == expectedArrays[index],
            "CalibResult pose arrays should match image_01 through image_19");
    }
    return ok;
}

bool testDeleteLastSample(void)
{
    using namespace i2r_rlc::control;
    const fs::path root = makeCaseRoot("delete");
    CCalibrationCaptureController controller;
    controller.configure(makeOptions(root));
    bool ok = true;

    ok &= expectTrue(controller.startManualCapture().succeeded, "manual capture should start");
    ok &= expectTrue(controller.captureBoardSample(makeImage(30), makeRobotState(1)).succeeded,
        "board sample should be captured before delete");
    ok &= expectTrue(controller.deleteLastSample().succeeded, "delete should succeed");
    ok &= expectTrue(controller.state().boardImageCount == 0, "board count should roll back");
    ok &= expectTrue(controller.state().samples.empty(), "sample list should roll back");
    return ok;
}

bool testAutomaticCaptureSignalLatch(void)
{
    using namespace i2r_rlc::control;
    const fs::path root = makeCaseRoot("automatic");
    CCalibrationCaptureController controller;
    controller.configure(makeOptions(root));
    bool ok = true;

    const auto started = controller.startAutomaticCapture();
    ok &= expectTrue(started.succeeded, "automatic capture should start");
    ok &= expectTrue(started.automaticGuideOverlayActive,
        "automatic capture should enable the UI guide overlay");
    i2r_rlc::robot_comm::RobotState state = makeRobotState(1);
    state.s_Bcalib = 1;
    auto update = controller.onRobotStateChanged(makeImage(40), state);
    ok &= expectTrue(update.succeeded, "automatic rising signal should capture one sample");
    ok &= expectTrue(update.robotCommand.valid, "automatic capture should request next position");
    ok &= expectTrue(controller.state().boardImageCount == 1, "automatic should capture one board sample");

    update = controller.onRobotStateChanged(makeImage(41), state);
    ok &= expectTrue(update.succeeded, "held high signal should be accepted");
    ok &= expectTrue(controller.state().boardImageCount == 1, "held high signal should not capture twice");

    state.s_Bcalib = 0;
    ok &= expectTrue(controller.onRobotStateChanged(makeImage(42), state).succeeded,
        "signal release should be accepted");
    state.s_Bcalib = 1;
    state.curx_r += 1;
    ok &= expectTrue(controller.onRobotStateChanged(makeImage(43), state).succeeded,
        "next rising signal should capture another sample");
    ok &= expectTrue(controller.state().boardImageCount == 2, "second rising signal should capture second sample");
    return ok;
}

bool testAutomaticPairedLaserCapture(void)
{
    using namespace i2r_rlc::control;
    const fs::path root = makeCaseRoot("automatic_pair");
    CCalibrationCaptureController controller;
    controller.configure(makeOptions(root));
    bool ok = true;

    ok &= expectTrue(controller.startAutomaticCapture().succeeded,
        "automatic paired capture should start");
    i2r_rlc::robot_comm::RobotState state;
    for (int i = 0; i < 15; ++i) {
        state = makeRobotState(i + 1);
        state.s_Bcalib = 1;
        auto update = controller.onRobotStateChanged(
            makeImage(static_cast<unsigned char>(30 + i)),
            state);
        ok &= expectTrue(update.succeeded, "automatic board-only point should capture");
        ok &= expectTrue(update.robotCommand.valid,
            "automatic board-only point should request next position");
        state.s_Bcalib = 0;
        ok &= expectTrue(controller.onRobotStateChanged(makeImage(70), state).succeeded,
            "automatic board-only release should be accepted");
    }

    state = makeRobotState(16);
    state.s_Bcalib = 1;
    auto update = controller.onRobotStateChanged(makeImage(90), state);
    ok &= expectTrue(update.succeeded, "automatic paired board should capture");
    ok &= expectTrue(
        update.phase == CalibrationCapturePhase::AutomaticCapturingLaser,
        "automatic paired board should wait for delayed laser image");
    ok &= expectTrue(!update.robotCommand.valid,
        "automatic paired board should not request next position before laser");
    ok &= expectTrue(controller.state().boardImageCount == 16,
        "automatic paired board count should include image_16.png");
    ok &= expectTrue(controller.state().laserImageCount == 0,
        "automatic paired laser image should still be pending");

    state.s_Bcalib = 0;
    update = controller.onRobotStateChanged(makeImage(91), state);
    ok &= expectTrue(update.phase == CalibrationCapturePhase::AutomaticCapturingLaser,
        "release before delayed laser capture should not leave laser phase");

    update = controller.captureLaserImage(makeImage(120), makeRobotState(16));
    ok &= expectTrue(update.succeeded, "automatic delayed laser image should capture");
    ok &= expectTrue(update.robotCommand.valid,
        "automatic paired laser image should request the next robot point");
    ok &= expectTrue(update.robotCommand.capturedSampleCount == 16,
        "automatic paired laser C count should stay on the completed position");

    const fs::path methodDir = root / "CALID" / "4 TWO_STEP";
    ok &= expectTrue(fs::exists(methodDir / "image_16.png"),
        "automatic paired board should write image_16.png");
    ok &= expectTrue(fs::exists(methodDir / "image_16L.png"),
        "automatic delayed laser should write image_16L.png");
    const std::vector<std::string> lines = readLines(methodDir / "CalibOriginData.txt");
    ok &= expectTrue(lines.size() == 17,
        "automatic paired laser should not append an extra origin pose");
    return ok;
}

bool testAutomaticGuidePreviewGate(void)
{
    using namespace i2r_rlc::control;
    const fs::path root = makeCaseRoot("automatic_preview");
    CCalibrationCaptureController controller;
    controller.configure(makeOptions(root));
    bool ok = true;

    auto update = controller.previewAutomaticCapture();
    ok &= expectTrue(update.succeeded, "automatic preview should succeed");
    ok &= expectTrue(update.phase == CalibrationCapturePhase::AutomaticGuidePreview,
        "automatic preview should enter guide preview phase");
    ok &= expectTrue(update.automaticGuideOverlayActive,
        "automatic preview should enable red guide overlay");
    ok &= expectTrue(!update.robotCommand.valid,
        "automatic preview should not request robot movement");

    i2r_rlc::robot_comm::RobotState state = makeRobotState(1);
    state.s_Bcalib = 1;
    update = controller.onRobotStateChanged(makeImage(50), state);
    ok &= expectTrue(update.succeeded,
        "robot handshake should be ignored while waiting user confirmation");
    ok &= expectTrue(controller.state().boardImageCount == 0,
        "automatic preview should not capture before confirmation");
    ok &= expectTrue(!update.robotCommand.valid,
        "automatic preview should not emit C command before confirmation");

    update = controller.startAutomaticCapture();
    ok &= expectTrue(update.succeeded, "confirmed automatic capture should start");
    ok &= expectTrue(update.phase == CalibrationCapturePhase::AutomaticWaitingRobot,
        "confirmed automatic capture should wait robot signal");
    ok &= expectTrue(update.automaticGuideOverlayActive,
        "confirmed automatic capture should keep red guide overlay active");
    return ok;
}

bool testAutomaticGuideOverlayLifecycle(void)
{
    using namespace i2r_rlc::control;
    const fs::path root = makeCaseRoot("automatic_overlay");
    CCalibrationCaptureController controller;
    auto options = makeOptions(root);
    options.boardImageCount = 0;
    options.laserImageCount = 0;
    controller.configure(options);
    bool ok = true;

    auto update = controller.startAutomaticCapture();
    ok &= expectTrue(update.succeeded, "automatic overlay lifecycle should start");
    ok &= expectTrue(update.automaticGuideOverlayActive,
        "automatic waiting phase should keep red guide overlay active");

    update = controller.finishCapture();
    ok &= expectTrue(update.succeeded, "finish with zero required images should complete");
    ok &= expectTrue(!update.automaticGuideOverlayActive,
        "completed automatic capture should disable red guide overlay");

    update = controller.startAutomaticCapture();
    ok &= expectTrue(update.automaticGuideOverlayActive,
        "automatic overlay should re-enable after restart");
    update = controller.reset();
    ok &= expectTrue(update.succeeded, "reset should succeed");
    ok &= expectTrue(!update.automaticGuideOverlayActive,
        "reset should disable red guide overlay");

    CCalibrationCaptureController errorController;
    auto errorOptions = makeOptions(root / "error");
    errorOptions.boardImageCount = 1;
    errorOptions.laserImageCount = 0;
    errorController.configure(errorOptions);
    update = errorController.startAutomaticCapture();
    ok &= expectTrue(update.automaticGuideOverlayActive,
        "automatic overlay should be active before an acquisition error");
    i2r_rlc::robot_comm::RobotState state = makeRobotState(1);
    state.s_Bcalib = 1;
    update = errorController.onRobotStateChanged(cv::Mat{}, state);
    ok &= expectTrue(!update.succeeded, "empty automatic image should fail");
    ok &= expectTrue(update.phase == CalibrationCapturePhase::Error,
        "empty automatic image should enter error phase");
    ok &= expectTrue(!update.automaticGuideOverlayActive,
        "error phase should disable red guide overlay");
    return ok;
}

bool testInvalidInputs(void)
{
    using namespace i2r_rlc::control;
    const fs::path root = makeCaseRoot("invalid");
    CCalibrationCaptureController controller;
    controller.configure(makeOptions(root));
    bool ok = true;

    ok &= expectTrue(controller.startManualCapture().succeeded, "manual capture should start");
    ok &= expectTrue(!controller.captureBoardSample(cv::Mat{}, makeRobotState(1)).succeeded,
        "empty image should fail");
    ok &= expectTrue(controller.state().phase == CalibrationCapturePhase::Error,
        "empty image should enter error phase");

    controller.startManualCapture();
    ok &= expectTrue(!controller.captureBoardSample(makeImage(60), i2r_rlc::robot_comm::RobotState{}).succeeded,
        "invalid robot pose should fail");
    return ok;
}

} // namespace

int main()
{
    bool ok = true;
    ok &= testManualCapture();
    ok &= testDeleteLastSample();
    ok &= testAutomaticCaptureSignalLatch();
    ok &= testAutomaticPairedLaserCapture();
    ok &= testAutomaticGuidePreviewGate();
    ok &= testAutomaticGuideOverlayLifecycle();
    ok &= testInvalidInputs();

    if (!ok) {
        return 1;
    }

    std::cout << "PASS: calibration capture tests\n";
    return 0;
}
