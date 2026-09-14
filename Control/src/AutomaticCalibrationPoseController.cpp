/*******************************************************************************
 * @file        AutomaticCalibrationPoseController.cpp
 * @brief       自动标定位姿文件读取、修正和机器人命令生成实现
 * @details     保留旧工程 CalibPose.txt 的 19 点模板和 O 命令下发语义。
 * @author      I2R Laboratory
 * @version     1.0
 ******************************************************************************/
#include "AutomaticCalibrationPoseController.h"

#include <cctype>
#include <exception>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace i2r_rlc::control {
namespace {

constexpr int kAutomaticCalibrationPoseCount = 19;
constexpr int kLegacyOfflineSignal = 4;

AutomaticCalibrationPoseResult makeFailure(
    const std::string &filePath,
    const std::string &message)
{
    AutomaticCalibrationPoseResult result;
    result.succeeded = false;
    result.sourceFilePath = filePath;
    result.message = message;
    return result;
}

bool isBlankLine(const std::string &line)
{
    for (const char character : line) {
        if (!std::isspace(static_cast<unsigned char>(character))) {
            return false;
        }
    }
    return true;
}

bool parseLongToken(
    const std::string &token,
    long *value)
{
    if (value == nullptr) {
        return false;
    }

    std::size_t consumed = 0;
    try {
        const long parsed = std::stol(token, &consumed, 10);
        if (consumed != token.size()) {
            return false;
        }
        *value = parsed;
    } catch (const std::exception &) {
        return false;
    }
    return true;
}

bool parsePoseLine(
    const std::string &line,
    AutomaticCalibrationPose *pose,
    std::string *errorMessage)
{
    if (pose == nullptr) {
        return false;
    }

    std::istringstream stream(line);
    std::string token;
    std::vector<long> values;
    while (stream >> token) {
        long value = 0;
        if (!parseLongToken(token, &value)) {
            if (errorMessage != nullptr) {
                *errorMessage = "invalid integer token: " + token;
            }
            return false;
        }
        values.push_back(value);
    }

    if (values.size() != 6U && values.size() != 8U) {
        if (errorMessage != nullptr) {
            *errorMessage =
                "CalibPose line must contain 6 or 8 integer fields";
        }
        return false;
    }

    pose->xUm = values[0];
    pose->yUm = values[1];
    pose->zUm = values[2];
    pose->rxDeg10000 = values[3];
    pose->ryDeg10000 = values[4];
    pose->rzDeg10000 = values[5];
    if (values.size() == 8U) {
        pose->s1Deg10000 = values[6];
        pose->s2Deg10000 = values[7];
        pose->stationAxesValid = true;
    }
    return true;
}

bool writeCorrectedPoseFile(
    const std::string &filePath,
    const std::vector<AutomaticCalibrationPose> &poses,
    std::string *errorMessage)
{
    const std::filesystem::path path(filePath);
    const std::filesystem::path parent = path.parent_path();
    if (!parent.empty()) {
        std::error_code errorCode;
        std::filesystem::create_directories(parent, errorCode);
        if (errorCode) {
            if (errorMessage != nullptr) {
                *errorMessage = "failed to create CalibPose_new directory: "
                    + errorCode.message();
            }
            return false;
        }
    }

    std::ofstream output(path, std::ios::trunc);
    if (!output) {
        if (errorMessage != nullptr) {
            *errorMessage = "failed to open CalibPose_new.txt for writing";
        }
        return false;
    }

    for (const auto &pose : poses) {
        output << pose.xUm << ' '
               << pose.yUm << ' '
               << pose.zUm << ' '
               << pose.rxDeg10000 << ' '
               << pose.ryDeg10000 << ' '
               << pose.rzDeg10000 << '\n';
    }
    return true;
}

} // namespace

AutomaticCalibrationPoseResult
CAutomaticCalibrationPoseController::loadPoseFile(
    const std::string &filePath) const
{
    std::ifstream input(filePath);
    if (!input) {
        return makeFailure(
            filePath,
            "CalibPose.txt was not found or cannot be opened");
    }

    AutomaticCalibrationPoseResult result;
    result.succeeded = true;
    result.sourceFilePath = filePath;

    std::string line;
    int lineNumber = 0;
    int expectedFieldCount = 0;
    while (std::getline(input, line)) {
        ++lineNumber;
        if (isBlankLine(line)) {
            continue;
        }

        AutomaticCalibrationPose pose;
        std::string errorMessage;
        if (!parsePoseLine(line, &pose, &errorMessage)) {
            return makeFailure(
                filePath,
                "failed to parse CalibPose.txt line "
                    + std::to_string(lineNumber) + ": " + errorMessage);
        }

        const int currentFieldCount = pose.stationAxesValid ? 8 : 6;
        if (expectedFieldCount == 0) {
            expectedFieldCount = currentFieldCount;
        } else if (currentFieldCount != expectedFieldCount) {
            return makeFailure(
                filePath,
                "CalibPose.txt must not mix 6-field and 8-field lines");
        }

        result.poses.push_back(pose);
    }

    if (static_cast<int>(result.poses.size())
        != kAutomaticCalibrationPoseCount) {
        return makeFailure(
            filePath,
            "CalibPose.txt must contain exactly 19 pose lines, actual="
                + std::to_string(result.poses.size()));
    }

    result.message = "CalibPose.txt loaded";
    return result;
}

AutomaticCalibrationPoseResult
CAutomaticCalibrationPoseController::makeTemplateCommand(
    const std::string &templateFilePath) const
{
    AutomaticCalibrationPoseResult result = loadPoseFile(templateFilePath);
    if (!result.succeeded) {
        return result;
    }

    result.robotCommand = makeRobotCommand(
        result.poses,
        "automatic calibration template poses");
    result.message = "automatic calibration template O command generated";
    return result;
}

AutomaticCalibrationPoseResult
CAutomaticCalibrationPoseController::makeCorrectedCommand(
    const std::string &templateFilePath,
    const std::string &outputFilePath,
    const robot_comm::RobotState &currentState) const
{
    AutomaticCalibrationPoseResult result = loadPoseFile(templateFilePath);
    if (!result.succeeded) {
        return result;
    }

    const AutomaticCalibrationPose &firstPose = result.poses.front();
    const long dxUm = firstPose.xUm - currentState.curx_r;
    const long dyUm = firstPose.yUm - currentState.cury_r;
    const long dzUm = firstPose.zUm - currentState.curz_r;
    const long drzDeg10000 = firstPose.rzDeg10000 - currentState.curRz_r;

    std::vector<AutomaticCalibrationPose> correctedPoses;
    correctedPoses.reserve(result.poses.size());
    for (const auto &pose : result.poses) {
        AutomaticCalibrationPose corrected = pose;
        corrected.xUm = pose.xUm - dxUm;
        corrected.yUm = pose.yUm - dyUm;
        corrected.zUm = pose.zUm - dzUm;
        corrected.rzDeg10000 = pose.rzDeg10000 - drzDeg10000;
        corrected.s1Deg10000 = 0;
        corrected.s2Deg10000 = 0;
        corrected.stationAxesValid = false;
        correctedPoses.push_back(corrected);
    }

    std::string errorMessage;
    if (!writeCorrectedPoseFile(
            outputFilePath,
            correctedPoses,
            &errorMessage)) {
        return makeFailure(templateFilePath, errorMessage);
    }

    result.succeeded = true;
    result.outputFilePath = outputFilePath;
    result.poses = correctedPoses;
    result.robotCommand = makeRobotCommand(
        result.poses,
        "automatic calibration corrected poses");
    result.message = "automatic calibration corrected O command generated";
    return result;
}

robot_comm::RobotCorrectionCommand
CAutomaticCalibrationPoseController::makeRobotCommand(
    const std::vector<AutomaticCalibrationPose> &poses,
    const std::string &message) const
{
    robot_comm::RobotCorrectionCommand command;
    command.requestTeachPointUpdate = true;
    command.pointCount = static_cast<int>(poses.size());
    command.offlineSignal = kLegacyOfflineSignal;
    command.message = message;
    command.valid = !poses.empty();
    command.teachPoints.reserve(poses.size());

    bool includeStationAxes = !poses.empty();
    for (const auto &pose : poses) {
        includeStationAxes = includeStationAxes && pose.stationAxesValid;

        robot_comm::TeachPointUpdateCommand teachPoint;
        teachPoint.xUm = pose.xUm;
        teachPoint.yUm = pose.yUm;
        teachPoint.zUm = pose.zUm;
        teachPoint.rxDeg10000 = pose.rxDeg10000;
        teachPoint.ryDeg10000 = pose.ryDeg10000;
        teachPoint.rzDeg10000 = pose.rzDeg10000;
        teachPoint.valid = true;
        command.teachPoints.push_back(teachPoint);
    }

    if (includeStationAxes) {
        command.stationAxes.reserve(poses.size());
        for (const auto &pose : poses) {
            robot_comm::StationAxisUpdateCommand stationAxis;
            stationAxis.s1Deg10000 = pose.s1Deg10000;
            stationAxis.s2Deg10000 = pose.s2Deg10000;
            stationAxis.valid = true;
            command.stationAxes.push_back(stationAxis);
        }
    }

    return command;
}

} // namespace i2r_rlc::control
