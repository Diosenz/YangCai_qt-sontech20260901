/*******************************************************************************
 *       ______   ______   _______         __                  __
 *      /      | /      \ /       \       /  |                /  |
 *      $$$$$$/ /$$$$$$  |$$$$$$$  |      $$ |        ______  $$ |____
 *        $$ |  $$____$$ |$$ |__$$ |      $$ |       /      \ $$      \
 *        $$ |   /    $$/ $$    $$<       $$ |       $$$$$$  |$$$$$$$  |
 *        $$ |  /$$$$$$/  $$$$$$$  |      $$ |       /    $$ |$$ |  $$ |
 *       _$$ |_ $$ |_____ $$ |  $$ |      $$ |_____ /$$$$$$$ |$$ |__$$ |
 *      / $$   |$$       |$$ |  $$ |      $$       |$$    $$ |$$    $$/
 *      $$$$$$/ $$$$$$$$/ $$/   $$/       $$$$$$$$/  $$$$$$$/ $$$$$$$/
 *
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        CalibrationCaptureController.cpp
 * @brief       标定数据采集控制器实现
 * @details     使用 OpenCV 文件输出和显式状态机实现旧 Calibration 数据采集流程。
 *
 * @version     1.1
 * @author      LT
 * @date        2026 年 6 月 6 日
 *
 * @par         修改日志
 * <table>
 * <tr><th>Version  <th>Author      <th>Date        <th>Description         </tr>
 * <tr><td>1.0      <td>LT          <td>260606      <td>创建文件            </tr>
 * <tr><td>1.1      <td>LT          <td>260606      <td>初步完成            </tr>
 * </table>
 *
 ******************************************************************************/
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <opencv2/imgcodecs.hpp>

#include "CalibrationCaptureController.h"

namespace i2r_rlc::control {
namespace {

/// @brief 标定板采集建议曝光时间，单位为 us。
constexpr int kBoardExposureUs = 8000;
/// @brief 激光采集建议曝光时间，单位为 us。
constexpr int kLaserExposureUs = 200;
/// @brief 采集完成后恢复的建议曝光时间，单位为 us。
constexpr int kFinalExposureUs = 1200;

/**
 * @brief 生成旧工程 two-step 标定数据目录。
 * @param options 标定采集配置。
 * @return 输出目录路径，不负责创建目录。
 */
std::filesystem::path methodDirectoryPath(const CalibrationCaptureOptions &options)
{
    return std::filesystem::path(options.outputRootDirectory) / options.methodDirectory;
}

/**
 * @brief 将样本序号格式化为两位数字字符串。
 * @param value 样本序号。
 * @return 例如 1 转为 "01"。
 */
std::string twoDigit(int value)
{
    std::ostringstream stream;
    stream << std::setw(2) << std::setfill('0') << value;
    return stream.str();
}

/**
 * @brief 判断机器人位姿是否包含非零坐标或姿态。
 * @param state 机器人状态，位置单位为 um，姿态单位为 deg * 10000。
 * @return 至少一个位姿字段非零时返回 true。
 */
bool isRobotPoseValid(const robot_comm::RobotState &state)
{
    return state.curx_r != 0 || state.cury_r != 0 || state.curz_r != 0
        || state.curRx_r != 0 || state.curRy_r != 0 || state.curRz_r != 0;
}

} // namespace

void CCalibrationCaptureController::configure(const CalibrationCaptureOptions &options)
{
    m_options = options;
    m_state = CalibrationCaptureState{};
    m_state.outputDirectory = methodDirectoryPath(m_options).string();
    m_originDataInitialized = false;
    setPhase(CalibrationCapturePhase::Idle, "calibration capture configured");
}

CalibrationCaptureUpdate CCalibrationCaptureController::startManualCapture()
{
    m_state = CalibrationCaptureState{};
    m_state.mode = CalibrationCaptureMode::Manual;
    m_state.outputDirectory = methodDirectoryPath(m_options).string();
    m_originDataInitialized = false;
    setPhase(CalibrationCapturePhase::ManualCapturingBoard, "manual calibration capture started");
    return makeUpdate(true, m_state.message);
}

CalibrationCaptureUpdate CCalibrationCaptureController::previewAutomaticCapture()
{
    m_state = CalibrationCaptureState{};
    m_state.mode = CalibrationCaptureMode::Automatic;
    m_state.outputDirectory = methodDirectoryPath(m_options).string();
    m_originDataInitialized = false;
    setPhase(
        CalibrationCapturePhase::AutomaticGuidePreview,
        "automatic calibration guide preview waiting user confirmation");
    return makeUpdate(true, m_state.message);
}

CalibrationCaptureUpdate CCalibrationCaptureController::startAutomaticCapture()
{
    m_state = CalibrationCaptureState{};
    m_state.mode = CalibrationCaptureMode::Automatic;
    m_state.outputDirectory = methodDirectoryPath(m_options).string();
    m_originDataInitialized = false;
    setPhase(CalibrationCapturePhase::AutomaticWaitingRobot, "automatic calibration capture waiting robot");
    return makeUpdate(true, m_state.message);
}

CalibrationCaptureUpdate CCalibrationCaptureController::captureBoardSample(
    const cv::Mat &image,
    const robot_comm::RobotState &robotState)
{
    if (isPairedLaserImagePending()) {
        setPhase(
            CalibrationCapturePhase::ManualCapturingLaser,
            "paired laser image is required before moving to next board position");
        return makeUpdate(false, m_state.message);
    }
    if (m_state.boardImageCount >= totalBoardImageCount()) {
        setPhase(CalibrationCapturePhase::Completed, "calibration board samples are complete");
        return makeUpdate(false, m_state.message);
    }
    return captureSample(image, robotState, false);
}

CalibrationCaptureUpdate CCalibrationCaptureController::captureLaserImage(
    const cv::Mat &image,
    const robot_comm::RobotState &robotState)
{
    const bool automaticMode = m_state.mode == CalibrationCaptureMode::Automatic;
    if (m_state.boardImageCount <= m_options.boardImageCount) {
        setPhase(
            automaticMode
                ? CalibrationCapturePhase::AutomaticWaitingRobot
                : CalibrationCapturePhase::ManualCapturingBoard,
            "paired board image is required before laser capture");
        return makeUpdate(false, m_state.message);
    }
    if (!isPairedLaserImagePending()) {
        setPhase(
            isCaptureComplete()
                ? CalibrationCapturePhase::Completed
                : (automaticMode
                       ? CalibrationCapturePhase::AutomaticWaitingRobot
                       : CalibrationCapturePhase::ManualCapturingBoard),
            "no paired laser image is pending");
        return makeUpdate(false, m_state.message);
    }
    if (m_state.laserImageCount >= m_options.laserImageCount) {
        return makeError("laser samples are complete");
    }
    return captureSample(image, robotState, true);
}

CalibrationCaptureUpdate CCalibrationCaptureController::deleteLastSample()
{
    if (m_state.samples.empty()) {
        return makeError("no calibration sample to delete");
    }

    const CalibrationCaptureSample removed = m_state.samples.back();
    m_state.samples.pop_back();
    if (removed.laserImage) {
        m_state.laserImageCount = std::max(0, m_state.laserImageCount - 1);
        setPhase(CalibrationCapturePhase::ManualCapturingLaser, "last laser sample removed");
    } else {
        m_state.boardImageCount = std::max(0, m_state.boardImageCount - 1);
        setPhase(CalibrationCapturePhase::ManualCapturingBoard, "last board sample removed");
    }
    return makeUpdate(true, m_state.message);
}

CalibrationCaptureUpdate CCalibrationCaptureController::onRobotStateChanged(
    const cv::Mat &image,
    const robot_comm::RobotState &robotState)
{
    if (m_state.mode != CalibrationCaptureMode::Automatic) {
        return makeUpdate(true, "robot state accepted");
    }

    if (m_state.phase == CalibrationCapturePhase::AutomaticGuidePreview) {
        return makeUpdate(true, "automatic calibration guide waiting user confirmation");
    }

    if (m_state.phase == CalibrationCapturePhase::AutomaticCapturingLaser) {
        if (robotState.s_Bcalib == 0) {
            m_state.waitingRobotSignalRelease = false;
        }
        return makeUpdate(true, "waiting paired laser image capture");
    }

    if (robotState.s_Bcalib == 0) {
        m_state.waitingRobotSignalRelease = false;
        setPhase(CalibrationCapturePhase::AutomaticWaitingRobot, "waiting robot calibration signal");
        return makeUpdate(true, m_state.message);
    }

    if (m_state.waitingRobotSignalRelease) {
        return makeUpdate(true, "waiting robot calibration signal release");
    }

    m_state.waitingRobotSignalRelease = true;
    if (m_state.boardImageCount < totalBoardImageCount()) {
        setPhase(CalibrationCapturePhase::AutomaticCapturingBoard, "automatic board sample triggered");
        return captureSample(image, robotState, false);
    }

    setPhase(CalibrationCapturePhase::Completed, "calibration capture completed");
    return makeUpdate(true, m_state.message);
}

CalibrationCaptureUpdate CCalibrationCaptureController::finishCapture()
{
    if (isCaptureComplete()) {
        std::string errorMessage;
        if (!writeCalibrationResult(&errorMessage)) {
            return makeError(errorMessage);
        }

        setPhase(CalibrationCapturePhase::Completed, "calibration capture completed");
        CalibrationCaptureUpdate update = makeUpdate(true, m_state.message);
        update.requestedExposureUs = kFinalExposureUs;
        update.requestLaserOn = false;
        return update;
    }

    return makeError("calibration capture data is incomplete");
}

CalibrationCaptureUpdate CCalibrationCaptureController::reset()
{
    m_state = CalibrationCaptureState{};
    m_state.outputDirectory = methodDirectoryPath(m_options).string();
    m_originDataInitialized = false;
    setPhase(CalibrationCapturePhase::Idle, "calibration capture reset");
    return makeUpdate(true, m_state.message);
}

const CalibrationCaptureState &CCalibrationCaptureController::state() const
{
    return m_state;
}

CalibrationCaptureUpdate CCalibrationCaptureController::makeUpdate(
    bool succeeded,
    const std::string &message) const
{
    CalibrationCaptureUpdate update;
    update.succeeded = succeeded;
    update.phase = m_state.phase;
    update.state = m_state;
    update.message = message;
    update.automaticGuideLeftPixel = m_options.automaticGuideLeftPixel;
    update.automaticGuideTopPixel = m_options.automaticGuideTopPixel;
    update.automaticGuideRightPixel = m_options.automaticGuideRightPixel;
    update.automaticGuideBottomPixel = m_options.automaticGuideBottomPixel;
    update.automaticGuideLineWidthPixel = m_options.automaticGuideLineWidthPixel;
    update.requiredBoardImageCount = totalBoardImageCount();
    update.requiredLaserImageCount = m_options.laserImageCount;
    update.automaticGuideOverlayActive =
        m_options.automaticGuideOverlayEnabled
        && (m_state.phase == CalibrationCapturePhase::AutomaticGuidePreview
            || m_state.phase == CalibrationCapturePhase::AutomaticWaitingRobot
            || m_state.phase == CalibrationCapturePhase::AutomaticCapturingBoard
            || m_state.phase == CalibrationCapturePhase::AutomaticCapturingLaser);
    if (m_state.phase == CalibrationCapturePhase::ManualCapturingLaser
        || m_state.phase == CalibrationCapturePhase::AutomaticCapturingLaser) {
        update.requestLaserOn = true;
        update.requestedExposureUs = kLaserExposureUs;
    } else if (m_state.phase == CalibrationCapturePhase::Idle
        || m_state.phase == CalibrationCapturePhase::Completed
        || m_state.phase == CalibrationCapturePhase::Error) {
        update.requestLaserOn = false;
        update.requestedExposureUs = kFinalExposureUs;
    } else {
        update.requestLaserOn = false;
        update.requestedExposureUs = kBoardExposureUs;
    }
    return update;
}

CalibrationCaptureUpdate CCalibrationCaptureController::makeError(const std::string &message)
{
    setPhase(CalibrationCapturePhase::Error, message);
    return makeUpdate(false, message);
}

void CCalibrationCaptureController::setPhase(
    CalibrationCapturePhase phase,
    const std::string &message)
{
    m_state.phase = phase;
    m_state.message = message;
}

bool CCalibrationCaptureController::ensureOutputDirectory(std::string *errorMessage) const
{
    std::error_code errorCode;
    const std::filesystem::path directory = methodDirectoryPath(m_options);
    std::filesystem::create_directories(directory, errorCode);
    if (errorCode) {
        if (errorMessage) {
            *errorMessage = "failed to create calibration output directory: "
                + errorCode.message();
        }
        return false;
    }
    return true;
}

bool CCalibrationCaptureController::writeOriginHeaderIfNeeded(std::string *errorMessage)
{
    if (m_originDataInitialized) {
        return true;
    }

    if (!ensureOutputDirectory(errorMessage)) {
        return false;
    }

    const std::filesystem::path originPath =
        methodDirectoryPath(m_options) / "CalibOriginData.txt";
    std::ofstream output(
        originPath,
        m_options.overwriteOriginData ? std::ios::trunc : std::ios::app);
    if (!output) {
        if (errorMessage) {
            *errorMessage = "failed to open CalibOriginData.txt";
        }
        return false;
    }
    output << "save tempPos\n";
    m_originDataInitialized = true;
    return true;
}

bool CCalibrationCaptureController::appendOriginDataLine(
    const robot_comm::RobotState &robotState,
    std::string *errorMessage)
{
    if (!writeOriginHeaderIfNeeded(errorMessage)) {
        return false;
    }

    const std::filesystem::path originPath =
        methodDirectoryPath(m_options) / "CalibOriginData.txt";
    std::ofstream output(originPath, std::ios::app);
    if (!output) {
        if (errorMessage) {
            *errorMessage = "failed to append CalibOriginData.txt";
        }
        return false;
    }

    output << robotState.curx_r << ' '
           << robotState.cury_r << ' '
           << robotState.curz_r << ' '
           << robotState.curRx_r << ' '
           << robotState.curRy_r << ' '
           << robotState.curRz_r << '\n';
    return true;
}

bool CCalibrationCaptureController::writeCalibrationResult(
    std::string *errorMessage) const
{
    std::vector<const CalibrationCaptureSample *> boardSamples;
    boardSamples.reserve(static_cast<std::size_t>(totalBoardImageCount()));
    for (const auto &sample : m_state.samples) {
        if (!sample.laserImage) {
            boardSamples.push_back(&sample);
        }
    }

    if (static_cast<int>(boardSamples.size()) != totalBoardImageCount()) {
        if (errorMessage) {
            *errorMessage = "failed to write CalibResult.txt: board sample count mismatch";
        }
        return false;
    }

    for (std::size_t index = 0; index < boardSamples.size(); ++index) {
        const std::string expectedName =
            "image_" + twoDigit(static_cast<int>(index) + 1) + ".png";
        const std::string actualName =
            std::filesystem::path(boardSamples[index]->imagePath).filename().string();
        if (actualName != expectedName) {
            if (errorMessage) {
                *errorMessage = "failed to write CalibResult.txt: pose/image order mismatch at "
                    + expectedName;
            }
            return false;
        }
    }

    if (!ensureOutputDirectory(errorMessage)) {
        return false;
    }

    const std::filesystem::path resultPath =
        methodDirectoryPath(m_options) / "CalibResult.txt";
    std::ofstream output(resultPath, std::ios::trunc);
    if (!output) {
        if (errorMessage) {
            *errorMessage = "failed to open CalibResult.txt";
        }
        return false;
    }

    auto writeArray = [&](const char *name, long CalibrationCaptureSample::*field) {
        output << name << "[] = {";
        for (const auto *sample : boardSamples) {
            output << (sample->*field) << ',';
        }
        output << "};\n";
    };

    output << "CalibResult\n\nRobotPos\n";
    writeArray("cx", &CalibrationCaptureSample::xUm);
    writeArray("cy", &CalibrationCaptureSample::yUm);
    writeArray("cz", &CalibrationCaptureSample::zUm);
    writeArray("crx", &CalibrationCaptureSample::rxDeg10000);
    writeArray("cry", &CalibrationCaptureSample::ryDeg10000);
    writeArray("crz", &CalibrationCaptureSample::rzDeg10000);
    output << '\n';

    if (!output) {
        if (errorMessage) {
            *errorMessage = "failed to write CalibResult.txt";
        }
        return false;
    }
    return true;
}

bool CCalibrationCaptureController::saveImage(
    const cv::Mat &image,
    const std::string &fileName,
    std::string *outputPath,
    std::string *errorMessage) const
{
    if (image.empty()) {
        if (errorMessage) {
            *errorMessage = "calibration image is empty";
        }
        return false;
    }
    if (!ensureOutputDirectory(errorMessage)) {
        return false;
    }

    const std::filesystem::path path = methodDirectoryPath(m_options) / fileName;
    try {
        if (!cv::imwrite(path.string(), image)) {
            if (errorMessage) {
                *errorMessage = "failed to write calibration image";
            }
            return false;
        }
    } catch (const cv::Exception &exception) {
        if (errorMessage) {
            *errorMessage = exception.what();
        }
        return false;
    }

    if (outputPath) {
        *outputPath = path.string();
    }
    return true;
}

CalibrationCaptureUpdate CCalibrationCaptureController::captureSample(
    const cv::Mat &image,
    const robot_comm::RobotState &robotState,
    bool laserImage)
{
    if (!isRobotPoseValid(robotState)) {
        return makeError("robot pose is invalid");
    }

    const int imageIndex = laserImage
        ? m_options.boardImageCount + m_state.laserImageCount + 1
        : m_state.boardImageCount + 1;
    const std::string fileName = laserImage
        ? "image_" + twoDigit(imageIndex) + "L.png"
        : "image_" + twoDigit(imageIndex) + ".png";

    std::string errorMessage;
    std::string imagePath;
    if (!laserImage && !appendOriginDataLine(robotState, &errorMessage)) {
        return makeError(errorMessage);
    }
    if (!saveImage(image, fileName, &imagePath, &errorMessage)) {
        return makeError(errorMessage);
    }

    CalibrationCaptureSample sample;
    sample.xUm = robotState.curx_r;
    sample.yUm = robotState.cury_r;
    sample.zUm = robotState.curz_r;
    sample.rxDeg10000 = robotState.curRx_r;
    sample.ryDeg10000 = robotState.curRy_r;
    sample.rzDeg10000 = robotState.curRz_r;
    sample.imagePath = imagePath;
    sample.laserImage = laserImage;
    sample.valid = true;
    m_state.samples.push_back(sample);

    if (laserImage) {
        ++m_state.laserImageCount;
        if (m_state.mode == CalibrationCaptureMode::Automatic) {
            m_state.waitingRobotSignalRelease = false;
        }
    } else {
        ++m_state.boardImageCount;
    }

    if (isCaptureComplete()) {
        if (!writeCalibrationResult(&errorMessage)) {
            return makeError(errorMessage);
        }
        setPhase(CalibrationCapturePhase::Completed, "calibration capture completed");
    } else if (m_state.mode == CalibrationCaptureMode::Automatic) {
        if (isPairedLaserImagePending()) {
            setPhase(
                CalibrationCapturePhase::AutomaticCapturingLaser,
                "paired board image captured, waiting laser image at same position");
        } else {
            setPhase(CalibrationCapturePhase::AutomaticWaitingRobot, "calibration sample captured");
        }
    } else if (isPairedLaserImagePending()) {
        setPhase(
            CalibrationCapturePhase::ManualCapturingLaser,
            "paired board image captured, capture laser image at same position");
    } else {
        setPhase(CalibrationCapturePhase::ManualCapturingBoard, "board sample captured");
    }

    CalibrationCaptureUpdate update = makeUpdate(true, m_state.message);
    if (m_state.phase != CalibrationCapturePhase::Completed
        && !isPairedLaserImagePending()) {
        update.robotCommand = makeNextPositionCommand();
    }
    return update;
}

int CCalibrationCaptureController::totalBoardImageCount() const
{
    return m_options.boardImageCount + m_options.laserImageCount;
}

bool CCalibrationCaptureController::isCaptureComplete() const
{
    return m_state.boardImageCount == totalBoardImageCount()
        && m_state.laserImageCount == m_options.laserImageCount;
}

bool CCalibrationCaptureController::isPairedLaserImagePending() const
{
    return m_state.boardImageCount > m_options.boardImageCount
        && m_state.boardImageCount <= totalBoardImageCount()
        && m_state.laserImageCount
            < (m_state.boardImageCount - m_options.boardImageCount);
}

robot_comm::RobotCalibrationCommand
CCalibrationCaptureController::makeNextPositionCommand() const
{
    robot_comm::RobotCalibrationCommand command;
    command.requestNextCalibrationPosition = true;
    command.calibResetSignal = 1;
    command.capturedSampleCount = m_state.boardImageCount;
    command.valid = true;
    command.message = "calibration sample captured, request next position";
    return command;
}

} // namespace i2r_rlc::control
