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
 * @file        OnlineCorrectionTrajectoryPlanner.cpp
 * @brief       在线纠偏轨迹修正服务实现
 * @details     使用标准 C++ 和显式单位实现旧 OnlineCor change 主线中的轨迹读取、
 *              起点对齐、参考轨迹平移和整段示教点命令生成。
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
#include "OnlineCorrectionTrajectoryPlanner.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace i2r_rlc::control {
namespace {

/**
 * @brief 计算两个在线纠偏空间点之间的欧氏距离。
 * @param lhs 第一个点，坐标单位为 mm。
 * @param rhs 第二个点，坐标单位为 mm。
 * @return 距离，单位为 mm。
 */
double distanceMm(const OnlineCorrectionPoint &lhs, const OnlineCorrectionPoint &rhs)
{
    const double dx = lhs.xMm - rhs.xMm;
    const double dy = lhs.yMm - rhs.yMm;
    const double dz = lhs.zMm - rhs.zMm;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * @brief 将机器人协议位姿转换为在线纠偏空间点。
 * @param state 机器人状态，位置单位为 um。
 * @return 当前 TCP 空间点，单位为 mm。
 */
OnlineCorrectionPoint pointFromRobotState(const robot_comm::RobotState &state)
{
    return OnlineCorrectionPoint{
        static_cast<double>(state.curx_r) / 1000.0,
        static_cast<double>(state.cury_r) / 1000.0,
        static_cast<double>(state.curz_r) / 1000.0,
        true
    };
}

/**
 * @brief 使用行主序 4x4 矩阵变换空间点。
 * @param point 输入空间点，单位为 mm。
 * @param rowMajor 行主序 4x4 齐次变换矩阵。
 * @return 变换后的空间点，单位为 mm。
 */
OnlineCorrectionPoint transformPoint(
    const OnlineCorrectionPoint &point,
    const std::array<double, 16> &rowMajor)
{
    const double x = point.xMm;
    const double y = point.yMm;
    const double z = point.zMm;
    return OnlineCorrectionPoint{
        rowMajor[0] * x + rowMajor[1] * y + rowMajor[2] * z + rowMajor[3],
        rowMajor[4] * x + rowMajor[5] * y + rowMajor[6] * z + rowMajor[7],
        rowMajor[8] * x + rowMajor[9] * y + rowMajor[10] * z + rowMajor[11],
        point.valid
    };
}

/**
 * @brief 从 4x4 修正矩阵中提取 RZ 方向角修正。
 * @param rowMajor 行主序 4x4 齐次变换矩阵。
 * @return 限制到 [-90,90] 附近的 yaw 修正角，单位为 deg。
 */
double yawCorrectionDeg(const std::array<double, 16> &rowMajor)
{
    const double rzRad = std::atan2(rowMajor[4], rowMajor[0]);
    double rzDeg = rzRad * 180.0 / 3.14159265358979323846;
    if (rzDeg < -90.0) {
        rzDeg += 180.0;
    } else if (rzDeg > 90.0) {
        rzDeg -= 180.0;
    }
    return rzDeg;
}

/**
 * @brief 将 mm 转换为机器人协议 um。
 * @param valueMm 长度，单位为 mm。
 * @return 四舍五入后的 um 整数。
 */
long toUm(double valueMm)
{
    return static_cast<long>(std::llround(valueMm * 1000.0));
}

/**
 * @brief 将 deg 转换为机器人协议 deg * 10000。
 * @param valueDeg 角度，单位为 deg。
 * @return 四舍五入后的协议整数。
 */
long toDeg10000(double valueDeg)
{
    return static_cast<long>(std::llround(valueDeg * 10000.0));
}

/**
 * @brief 判断文本行是否为空行或注释行。
 * @param line 原始文本行。
 * @return 空白、# 或 * 开头的行返回 true。
 */
bool isCommentOrEmpty(const std::string &line)
{
    const auto first = line.find_first_not_of(" \t\r\n");
    return first == std::string::npos || line[first] == '#' || line[first] == '*';
}


} // namespace

std::vector<OnlineCorrectionPoint> COnlineCorrectionTrajectoryPlanner::readPathPoints(
    const std::string &path,
    double minDistanceMm,
    std::vector<OnlineCorrectionPose> &poses,
    std::vector<OnlineCorrectionStationAxis> &axes)
{
    std::ifstream input(path);
    std::vector<OnlineCorrectionPoint> points;
    poses.clear();
    axes.clear();
    if (!input) {
        return points;
    }

    std::string line;
    while (std::getline(input, line)) {
        if (isCommentOrEmpty(line)) {
            continue;
        }

        std::istringstream stream(line);
        OnlineCorrectionPoint point;
        if (!(stream >> point.xMm >> point.yMm >> point.zMm)) {
            continue;
        }
        point.valid = true;

        if (points.empty() || minDistanceMm <= 0.0
            || distanceMm(points.back(), point) >= minDistanceMm) {
            OnlineCorrectionPose pose;
            OnlineCorrectionStationAxis axis;
            pose.xMm = point.xMm;
            pose.yMm = point.yMm;
            pose.zMm = point.zMm;
            if (stream >> pose.rxDeg >> pose.ryDeg >> pose.rzDeg
                >> axis.s1Deg >> axis.s2Deg) {
                pose.valid = true;
                axis.valid = true;
            }
            points.push_back(point);
            poses.push_back(pose);
            axes.push_back(axis);
        }
    }

    return points;
}

/**
 * @brief 读取示教位姿和可选变位机轴角。
 * @param path 文件路径。
 * @param poses 输出示教位姿，位置单位 mm，姿态单位 deg。
 * @param axes 输出变位机轴角，单位 deg。
 * @param segments 输出模板分段编号，缺省为 0。
 * @return 至少读取到一个有效位姿时返回 true。
 */
bool COnlineCorrectionTrajectoryPlanner::readTeachPoses(
    const std::string &path,
    std::vector<OnlineCorrectionPose> &poses,
    std::vector<OnlineCorrectionStationAxis> &axes,
    std::vector<int> &segments)
{
    //打开文件
    std::ifstream input(path);
    poses.clear();
    axes.clear();
    segments.clear();
    if (!input) {
        return false;
    }

    std::string line;
    while (std::getline(input, line)) {
        if (isCommentOrEmpty(line)) {
            continue;
        }

        std::istringstream stream(line);
        OnlineCorrectionPose pose;
        OnlineCorrectionStationAxis axis;
        if (!(stream >> pose.xMm >> pose.yMm >> pose.zMm
              >> pose.rxDeg >> pose.ryDeg >> pose.rzDeg)) {
            continue;
        }
        pose.valid = true;

        if (stream >> axis.s1Deg >> axis.s2Deg) {
            axis.valid = true;
        }

        int segment = 0;
        stream >> segment;
        poses.push_back(pose);
        axes.push_back(axis);
        segments.push_back(segment);
    }

    return !poses.empty();
}

int COnlineCorrectionTrajectoryPlanner::findNearestPointIndex(
    int beginIndex,
    int endIndex,
    const std::vector<OnlineCorrectionPoint> &points,
    const OnlineCorrectionPoint &reference)
{
    if (points.empty() || !reference.valid) {
        return -1;
    }

    beginIndex = std::max(0, beginIndex);
    endIndex = std::min(endIndex, static_cast<int>(points.size()));
    if (beginIndex >= endIndex) {
        return -1;
    }

    double bestDistance = std::numeric_limits<double>::max();
    int bestIndex = -1;
    for (int index = beginIndex; index < endIndex; ++index) {
        if (!points[index].valid) {
            continue;
        }
        const double currentDistance = distanceMm(points[index], reference);
        if (currentDistance < bestDistance) {
            bestDistance = currentDistance;
            bestIndex = index;
        }
    }

    return bestIndex;
}

OnlineCorrectionPlannedTrajectory COnlineCorrectionTrajectoryPlanner::loadTrajectory(
    const OnlineCorrectionTrajectoryRequest &request)
{
    OnlineCorrectionPlannedTrajectory result;
    result.segmentIndex = request.segmentIndex;
    result.allPathPoints = readPathPoints(
        request.workPathFile,
        request.minPathPointDistanceMm,
        result.allPathPoses,
        result.allPathStationAxes);
    std::vector<int> teachSegments;
    if (!readTeachPoses(request.teachPoseFile, result.teachPoses,
                       result.stationAxes, teachSegments)) {
        result.message = "teach pose file is missing or empty";
        return result;
    }
    if (result.allPathPoints.empty()) {
        result.message = "work path file is missing or empty";
        return result;
    }

    const int poseCount = static_cast<int>(result.teachPoses.size());
    const auto segmentBegin = std::find(
        teachSegments.begin(), teachSegments.end(), request.segmentIndex);
    result.startTeachIndex = request.startTeachIndex >= 0
        ? request.startTeachIndex
        : static_cast<int>(segmentBegin - teachSegments.begin());
    result.endTeachIndex = request.endTeachIndex >= 0
        ? request.endTeachIndex
        : result.startTeachIndex;
    if (request.endTeachIndex < 0 && result.startTeachIndex < poseCount) {
        while (result.endTeachIndex + 1 < poseCount
               && teachSegments[result.endTeachIndex + 1]
                   == teachSegments[result.startTeachIndex]) {
            ++result.endTeachIndex;
        }
    }

    if (result.startTeachIndex < 0 || result.endTeachIndex < result.startTeachIndex
        || result.endTeachIndex >= poseCount) {
        result.message = "teach index range is invalid";
        return result;
    }

    const OnlineCorrectionPoint startPoint{
        result.teachPoses[result.startTeachIndex].xMm,
        result.teachPoses[result.startTeachIndex].yMm,
        result.teachPoses[result.startTeachIndex].zMm,
        true
    };
    const OnlineCorrectionPoint endPoint{
        result.teachPoses[result.endTeachIndex].xMm,
        result.teachPoses[result.endTeachIndex].yMm,
        result.teachPoses[result.endTeachIndex].zMm,
        true
    };

    int startPathIndex = findNearestPointIndex(
        0, static_cast<int>(result.allPathPoints.size()), result.allPathPoints, startPoint);
    int endPathIndex = findNearestPointIndex(
        0, static_cast<int>(result.allPathPoints.size()), result.allPathPoints, endPoint);
    if (startPathIndex < 0 || endPathIndex < 0) {
        result.message = "failed to locate segment in work path";
        return result;
    }

    result.segmentStartPathIndex = startPathIndex;
    result.segmentEndPathIndex = endPathIndex;

    if (startPathIndex <= endPathIndex) {
        result.segmentPathPoints.assign(
            result.allPathPoints.begin() + startPathIndex,
            result.allPathPoints.begin() + endPathIndex + 1);
    }
    result.allMatchedPathPoints = result.allPathPoints;
    int teachIndex = 0;
    while (teachIndex < poseCount) {
        int segmentEndIndex = teachIndex;
        while (segmentEndIndex + 1 < poseCount
               && teachSegments[segmentEndIndex + 1] == teachSegments[teachIndex]) {
            ++segmentEndIndex;
        }
        const OnlineCorrectionPoint segmentStart{
            result.teachPoses[teachIndex].xMm,
            result.teachPoses[teachIndex].yMm,
            result.teachPoses[teachIndex].zMm,
            true
        };
        const OnlineCorrectionPoint segmentEnd{
            result.teachPoses[segmentEndIndex].xMm,
            result.teachPoses[segmentEndIndex].yMm,
            result.teachPoses[segmentEndIndex].zMm,
            true
        };
        int begin = findNearestPointIndex(
            0,
            static_cast<int>(result.allPathPoints.size()),
            result.allPathPoints,
            segmentStart);
        int end = findNearestPointIndex(
            0,
            static_cast<int>(result.allPathPoints.size()),
            result.allPathPoints,
            segmentEnd);
        teachIndex = segmentEndIndex + 1;
        if (begin < 0 || end < 0) {
            continue;
        }
        if (begin <= end) {
            result.pathSegments.emplace_back(
                result.allPathPoints.begin() + begin,
                result.allPathPoints.begin() + end + 1);
        }
    }
    if (result.pathSegments.empty()) {
        result.pathSegments.push_back(result.segmentPathPoints);
    }
    result.valid = !result.segmentPathPoints.empty();
    result.message = result.valid ? "trajectory loaded" : "segment path is empty";
    return result;
}

OnlineCorrectionTrajectoryPlanResult COnlineCorrectionTrajectoryPlanner::planInitialCorrection(
    const OnlineCorrectionPlannedTrajectory &trajectory,
    const OnlineCorrectionOptions &options,
    const OnlineCorrectionSessionState &state,
    const robot_comm::RobotState &robotState,
    const vision::PositionerCalibration &positionerCalibration)
{
    // OnlineCorrectionSessionState 为跨模块值对象；修改其布局后必须同步重编译本实现。
    OnlineCorrectionTrajectoryPlanResult result;
    if (!trajectory.valid || !positionerCalibration.valid) {
        result.message = "trajectory is invalid";
        return result;
    }

    OnlineCorrectionPoint initialPoint;
    std::vector<OnlineCorrectionPoint> detectionPoints;
    if (options.startMode == OnlineCorrectionStartMode::IBVSStart) {
        const auto stationPose = vision::CPositionerKinematics::robotPoseToStationPose(
            vision::PositionerPose{
                static_cast<double>(robotState.curx_r) / 1000.0,
                static_cast<double>(robotState.cury_r) / 1000.0,
                static_cast<double>(robotState.curz_r) / 1000.0,
                static_cast<double>(robotState.curRx_r) / 10000.0,
                static_cast<double>(robotState.curRy_r) / 10000.0,
                static_cast<double>(robotState.curRz_r) / 10000.0,
                true},
            static_cast<double>(robotState.curS1) / 10000.0,
            static_cast<double>(robotState.curS2) / 10000.0,
            positionerCalibration);
        initialPoint = OnlineCorrectionPoint{
            stationPose.xMm,
            stationPose.yMm,
            stationPose.zMm,
            stationPose.valid};
    } else {
        if (state.initialDetectionSamples.empty()) {
            result.message = "initial detection points are empty";
            return result;
        }
        detectionPoints.reserve(state.initialDetectionSamples.size());
        for (const auto &sample : state.initialDetectionSamples) {
            if (!sample.point.valid) {
                continue;
            }
            OnlineCorrectionPoint point = sample.point;
            // 视觉重建点以变位机 TCP 为原点，而 `_W/_T` 以变位机当前坐标系为原点。
            point.xMm += positionerCalibration.toolTcpInLastLinkMm.x();
            point.yMm += positionerCalibration.toolTcpInLastLinkMm.y();
            point.zMm += positionerCalibration.toolTcpInLastLinkMm.z();
            detectionPoints.push_back(point);
        }
        if (detectionPoints.empty()) {
            result.message = "initial detection points are invalid";
            return result;
        }
        initialPoint = detectionPoints.back();
    }

    if (!initialPoint.valid || !std::isfinite(initialPoint.xMm)
        || !std::isfinite(initialPoint.yMm) || !std::isfinite(initialPoint.zMm)) {
        result.message = "initial point is invalid";
        return result;
    }

    std::vector<OnlineCorrectionPoint> correctedSegment = trajectory.segmentPathPoints;
    std::vector<OnlineCorrectionPose> correctedPoses = trajectory.teachPoses;
    const bool applyGlobalCorrection =
        options.startMode == OnlineCorrectionStartMode::GlobalCorrection;
    const double rzCorrection = applyGlobalCorrection
        ? yawCorrectionDeg(options.globalCorrectionRowMajor)
        : 0.0;

    if (applyGlobalCorrection) {
        for (auto &point : correctedSegment) {
            point = transformPoint(point, options.globalCorrectionRowMajor);
        }
        for (auto &pose : correctedPoses) {
            OnlineCorrectionPoint transformed = transformPoint(
                OnlineCorrectionPoint{ pose.xMm, pose.yMm, pose.zMm, pose.valid },
                options.globalCorrectionRowMajor);
            pose.xMm = transformed.xMm;
            pose.yMm = transformed.yMm;
            pose.zMm = transformed.zMm;
            pose.rzDeg += rzCorrection;
        }
    }

    if (correctedSegment.empty()) {
        result.message = "corrected segment is empty";
        return result;
    }

    // 与旧 correctTeachTraj 一致：以检测起点对齐当前参考段首点，
    // 保留完整参考段，不用旧装夹轨迹的空间最近点重新定义圆周相位。
    const OnlineCorrectionPoint &segmentFirst = correctedSegment.front();
    const double dx = initialPoint.xMm - segmentFirst.xMm;
    const double dy = initialPoint.yMm - segmentFirst.yMm;
    const double dz = initialPoint.zMm - segmentFirst.zMm;

    for (auto &point : correctedSegment) {
        point.xMm += dx;
        point.yMm += dy;
        point.zMm += dz;
        if (!point.valid || !std::isfinite(point.xMm)
            || !std::isfinite(point.yMm) || !std::isfinite(point.zMm)) {
            result.message = "corrected reference path contains invalid points";
            return result;
        }
    }

    robot_comm::RobotCorrectionCommand command;
    command.requestTeachPointUpdate = true;
    command.offlineSignal = 4;

    const int initialTeachIndex = trajectory.startTeachIndex;
    const int finalTeachIndex = trajectory.endTeachIndex;
    if (initialTeachIndex < 0 || finalTeachIndex < initialTeachIndex
        || finalTeachIndex >= static_cast<int>(correctedPoses.size())
        || finalTeachIndex >= static_cast<int>(trajectory.stationAxes.size())) {
        result.message = "initial teach index range or positioner axes are invalid";
        return result;
    }

    // 与旧 correctTeachPts 一致：_T 使用自己的段首点计算平移。
    // _W 段首点可能有采样误差，不能直接复用其偏差来移动 _T。
    const auto &templateStart = correctedPoses[initialTeachIndex];
    const double teachDx = initialPoint.xMm - templateStart.xMm;
    const double teachDy = initialPoint.yMm - templateStart.yMm;
    const double teachDz = initialPoint.zMm - templateStart.zMm;
    for (int index = initialTeachIndex; index <= finalTeachIndex; ++index) {
        OnlineCorrectionPose pose = correctedPoses[index];
        const OnlineCorrectionStationAxis &axis = trajectory.stationAxes[index];
        pose.xMm += teachDx;
        pose.yMm += teachDy;
        pose.zMm += teachDz;
        if (!pose.valid
            || !std::isfinite(pose.xMm) || !std::isfinite(pose.yMm)
            || !std::isfinite(pose.zMm) || !std::isfinite(pose.rxDeg)
            || !std::isfinite(pose.ryDeg) || !std::isfinite(pose.rzDeg)
            || !std::isfinite(axis.s1Deg) || !std::isfinite(axis.s2Deg)) {
            result.message = "teach template contains invalid pose or positioner angles";
            return result;
        }

        // 各点保留模板外轴角度，分别换算机器人目标，避免把基坐标平移
        // 直接复制到不同外轴角度下；全部转换成功后才返回可发送批次。
        const auto robotPose = vision::CPositionerKinematics::stationPoseToRobotPose(
            vision::PositionerPose{
                pose.xMm, pose.yMm, pose.zMm,
                pose.rxDeg, pose.ryDeg, pose.rzDeg, pose.valid},
            axis.s1Deg,
            axis.s2Deg,
            positionerCalibration);
        if (!robotPose.valid
            || !std::isfinite(robotPose.xMm) || !std::isfinite(robotPose.yMm)
            || !std::isfinite(robotPose.zMm) || !std::isfinite(robotPose.rxDeg)
            || !std::isfinite(robotPose.ryDeg) || !std::isfinite(robotPose.rzDeg)) {
            result.message = "failed to transform teach template to robot base";
            return result;
        }

        robot_comm::TeachPointUpdateCommand teachPoint{
            toUm(robotPose.xMm), toUm(robotPose.yMm), toUm(robotPose.zMm),
            toDeg10000(robotPose.rxDeg), toDeg10000(robotPose.ryDeg),
            toDeg10000(robotPose.rzDeg), true};
        if (index == initialTeachIndex
            && options.startMode != OnlineCorrectionStartMode::IBVSStart) {
            // 反扫完成时的 D 回包已包含 RobotBase 焊枪姿态；仅首个 O 点
            // 继承该姿态，避免把 RobotBase 欧拉角误当作 _T 变位机坐标。
            teachPoint.rxDeg10000 = robotState.curRx_r;
            teachPoint.ryDeg10000 = robotState.curRy_r;
            teachPoint.rzDeg10000 = robotState.curRz_r;
        }
        command.teachPoints.push_back(teachPoint);
        command.stationAxes.push_back(robot_comm::StationAxisUpdateCommand{
            toDeg10000(axis.s1Deg), toDeg10000(axis.s2Deg), true});
    }

    command.pointCount = static_cast<int>(command.teachPoints.size());
    command.valid = command.pointCount > 0;
    command.message = command.valid ? "teach point update command generated" : "no teach point generated";

    result.valid = command.valid;
    result.initialPoint = initialPoint;
    result.correctedSegmentPath = std::move(correctedSegment);
    result.command = std::move(command);
    if (!result.valid) {
        result.message = result.command.message;
        return result;
    }

    result.message = "initial point aligned; reference path and teach template corrected; points="
        + std::to_string(result.command.pointCount);
    return result;
}

OnlineCorrectionTrajectoryPlanResult COnlineCorrectionTrajectoryPlanner::planInitialCorrection(
    const OnlineCorrectionPlannedTrajectory &trajectory,
    const OnlineCorrectionOptions &options,
    const OnlineCorrectionSessionState &state,
    const robot_comm::RobotState &robotState)
{
    vision::PositionerCalibration calibration;
    calibration.valid = true;
    calibration.stationBaseInRobotBase = Eigen::Affine3d::Identity();
    return planInitialCorrection(
        trajectory, options, state, robotState, calibration);
}

} // namespace i2r_rlc::control
