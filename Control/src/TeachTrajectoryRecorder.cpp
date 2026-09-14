/*******************************************************************************
 * @file        TeachTrajectoryRecorder.cpp
 * @brief       旧式示教轨迹保存记录器实现。
 ******************************************************************************/
#include <cmath>
#include <fstream>
#include <iomanip>

#include <regex>

#include "TeachTrajectoryRecorder.h"

namespace i2r_rlc::control {
namespace {

constexpr double kMinTrajectoryPointDistanceMm = 0.5;
constexpr double kTeachPointDistanceMm = 2.0;

bool copyFileNoOverwrite(
    const std::filesystem::path &source,
    const std::filesystem::path &target,
    std::string *errorMessage)
{
    if (std::filesystem::exists(target)) {
        if (errorMessage != nullptr) {
            *errorMessage = "目标文件已存在，未覆盖：" + target.string();
        }
        return false;
    }

    std::error_code error;
    std::filesystem::copy_file(
        source,
        target,
        std::filesystem::copy_options::none,
        error);
    if (error) {
        if (errorMessage != nullptr) {
            *errorMessage = "复制文件失败：" + error.message();
        }
        return false;
    }
    return true;
}

} // namespace

CTeachTrajectoryRecorder::CTeachTrajectoryRecorder(
    std::filesystem::path tempDirectory)
    : m_tempDirectory(std::move(tempDirectory))
{
}

TeachTrajectoryRecorderSnapshot CTeachTrajectoryRecorder::configureTempDirectory(
    const std::filesystem::path &tempDirectory)
{
    m_tempDirectory = tempDirectory;
    return makeSnapshot(true, "示教轨迹临时目录已配置。");
}

TeachTrajectoryRecorderSnapshot CTeachTrajectoryRecorder::startRecording()
{
    std::string errorMessage;
    if (!ensureTempFiles(&errorMessage)) {
        return fail(errorMessage);
    }

    {
        std::ofstream stream(seamTrajectoryPath(), std::ios::trunc);
        if (!stream) {
            return fail("无法清空 SeamTraj.txt。");
        }
    }
    {
        std::ofstream stream(seamTeachPath(), std::ios::trunc);
        if (!stream) {
            return fail("无法清空 SeamTeach.txt。");
        }
    }
    {
        std::ofstream stream(allPointsPath(), std::ios::trunc);
        if (!stream) {
            return fail("无法清空 AllPts.txt。");
        }
    }

    m_recordingEnabled = true;
    m_segmentActive = false;
    m_lastPose = Pose{};
    m_testCount = 0;
    m_trajectoryPointCount = 0;
    m_teachPointCount = 0;
    m_allPointCount = 0;
    m_latestSaveSignal = 0;
    m_lastTrajectoryTick = std::chrono::steady_clock::time_point{};
    return makeSnapshot(true, "示教轨迹临时文件已清空，等待机器人 savetraj 信号。");
}

TeachTrajectoryRecorderSnapshot CTeachTrajectoryRecorder::onRobotStateChanged(
    const robot_comm::RobotState &state)
{
    if (!m_recordingEnabled) {
        return makeSnapshot(true, "示教轨迹记录未启动。");
    }

    m_latestSaveSignal = state.s_Bsavetraj;
    const Pose pose = poseFromState(state);
    if (state.s_Bsavetraj == 1) {
        const double distance = distanceFromLast(pose);
        if (distance > kMinTrajectoryPointDistanceMm) {
            double timeDeltaSeconds = 0.0;
            const bool newSegment = !m_segmentActive;
            if (m_segmentActive) {
                ++m_testCount;
                const auto now = std::chrono::steady_clock::now();
                if (m_testCount == 1 || m_lastTrajectoryTick == std::chrono::steady_clock::time_point{}) {
                    timeDeltaSeconds = 0.0;
                } else {
                    timeDeltaSeconds =
                        std::chrono::duration<double>(now - m_lastTrajectoryTick).count();
                }
                m_lastTrajectoryTick = now;
            } else {
                m_segmentActive = true;
                m_testCount = 0;
                m_lastTrajectoryTick = std::chrono::steady_clock::now();
            }

            if (!appendTrajectoryPoint(pose, timeDeltaSeconds)) {
                return fail("写入 SeamTraj.txt 失败。");
            }
            if (newSegment
                || distance > kTeachPointDistanceMm) {
                if (!appendTeachPoint(pose)) {
                    return fail("写入 SeamTeach.txt 失败。");
                }
            }
            m_lastPose = pose;
        }
    } else if (m_segmentActive) {
        if (!appendTeachPoint(pose)) {
            return fail("写入末端 SeamTeach.txt 失败。");
        }
        if (!appendTrajectoryPoint(pose, 0.0)) {
            return fail("写入末端 SeamTraj.txt 失败。");
        }
        m_lastPose = pose;
        m_segmentActive = false;
        m_testCount = 0;
        m_lastTrajectoryTick = std::chrono::steady_clock::time_point{};
    }

    if (state.sStart == 1 && !appendAllPoint(state)) {
        return fail("写入 AllPts.txt 失败。");
    }

    return makeSnapshot(true, "示教轨迹记录状态已更新。");
}

TeachTrajectoryRecorderSnapshot CTeachTrajectoryRecorder::saveCapturedTrajectory(
    const TeachTrajectorySaveRequest &request)
{
    if (!isSafeBaseName(request.baseName)) {
        return fail("示教轨迹名称非法：请只填写不含后缀的英文/数字/下划线/短横线名称。");
    }
    if (request.baseName.size() >= 2) {
        const std::string suffix = request.baseName.substr(request.baseName.size() - 2);
        if (suffix == "_W" || suffix == "_T" || suffix == "_w" || suffix == "_t") {
            return fail("示教轨迹名称只填写基名，不要包含 _W 或 _T 后缀。");
        }
    }

    const std::filesystem::path outputDirectory(request.outputDirectory);
    if (outputDirectory.empty()) {
        return fail("请选择示教轨迹输出目录。");
    }
    std::error_code error;
    std::filesystem::create_directories(outputDirectory, error);
    if (error) {
        return fail("创建输出目录失败：" + error.message());
    }

    if (!fileHasLegacyContent(seamTrajectoryPath())) {
        return fail("SeamTraj.txt 为空，请先运行机器人示教轨迹保存程序。");
    }
    if (!fileHasLegacyContent(seamTeachPath())) {
        return fail("SeamTeach.txt 为空，请先运行机器人示教轨迹保存程序。");
    }

    const std::filesystem::path workPath =
        outputDirectory / (request.baseName + "_W.txt");
    const std::filesystem::path teachPath =
        outputDirectory / (request.baseName + "_T.txt");
    std::string errorMessage;
    if (!copyFileNoOverwrite(seamTrajectoryPath(), workPath, &errorMessage)) {
        return fail(errorMessage);
    }
    if (!copyFileNoOverwrite(seamTeachPath(), teachPath, &errorMessage)) {
        std::filesystem::remove(workPath, error);
        return fail(errorMessage);
    }

    auto snapshot = makeSnapshot(true, "保存示教轨迹成功。");
    snapshot.outputWorkPathFile = workPath.string();
    snapshot.outputTeachPoseFile = teachPath.string();
    return snapshot;
}

TeachTrajectoryRecorderSnapshot CTeachTrajectoryRecorder::reset()
{
    m_recordingEnabled = false;
    m_segmentActive = false;
    m_lastPose = Pose{};
    m_testCount = 0;
    m_latestSaveSignal = 0;
    m_lastTrajectoryTick = std::chrono::steady_clock::time_point{};
    return makeSnapshot(true, "示教轨迹记录已重置。");
}

TeachTrajectoryRecorderSnapshot CTeachTrajectoryRecorder::snapshot() const
{
    return makeSnapshot(true, "示教轨迹记录状态。");
}

TeachTrajectoryRecorderSnapshot CTeachTrajectoryRecorder::makeSnapshot(
    const bool succeeded,
    std::string message) const
{
    TeachTrajectoryRecorderSnapshot snapshot;
    snapshot.succeeded = succeeded;
    snapshot.recordingEnabled = m_recordingEnabled;
    snapshot.segmentActive = m_segmentActive;
    snapshot.latestSaveSignal = m_latestSaveSignal;
    snapshot.trajectoryPointCount = m_trajectoryPointCount;
    snapshot.teachPointCount = m_teachPointCount;
    snapshot.allPointCount = m_allPointCount;
    snapshot.tempDirectory = m_tempDirectory.string();
    snapshot.seamTrajectoryFile = seamTrajectoryPath().string();
    snapshot.seamTeachFile = seamTeachPath().string();
    snapshot.allPointsFile = allPointsPath().string();
    snapshot.message = std::move(message);
    return snapshot;
}

TeachTrajectoryRecorderSnapshot CTeachTrajectoryRecorder::fail(
    std::string message) const
{
    return makeSnapshot(false, std::move(message));
}

CTeachTrajectoryRecorder::Pose CTeachTrajectoryRecorder::poseFromState(
    const robot_comm::RobotState &state) const
{
    Pose pose;
    pose.xMm = static_cast<double>(state.curx_r) / 1000.0;
    pose.yMm = static_cast<double>(state.cury_r) / 1000.0;
    pose.zMm = static_cast<double>(state.curz_r) / 1000.0;
    pose.rxDeg = static_cast<double>(state.curRx_r) / 10000.0;
    pose.ryDeg = static_cast<double>(state.curRy_r) / 10000.0;
    pose.rzDeg = static_cast<double>(state.curRz_r) / 10000.0;
    pose.s1Deg = static_cast<double>(state.curS1) / 10000.0;
    pose.s2Deg = static_cast<double>(state.curS2) / 10000.0;
    return pose;
}

double CTeachTrajectoryRecorder::distanceFromLast(const Pose &pose) const
{
    const double dx = pose.xMm - m_lastPose.xMm;
    const double dy = pose.yMm - m_lastPose.yMm;
    const double dz = pose.zMm - m_lastPose.zMm;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool CTeachTrajectoryRecorder::ensureTempFiles(std::string *errorMessage) const
{
    if (m_tempDirectory.empty()) {
        if (errorMessage != nullptr) {
            *errorMessage = "示教轨迹临时目录未配置。";
        }
        return false;
    }
    std::error_code error;
    std::filesystem::create_directories(m_tempDirectory, error);
    if (error) {
        if (errorMessage != nullptr) {
            *errorMessage = "创建示教轨迹临时目录失败：" + error.message();
        }
        return false;
    }
    return true;
}

bool CTeachTrajectoryRecorder::appendTrajectoryPoint(
    const Pose &pose,
    const double timeDeltaSeconds)
{
    std::ofstream stream(seamTrajectoryPath(), std::ios::app);
    if (!stream) {
        return false;
    }
    stream << std::fixed
           << std::setprecision(3) << pose.xMm << ' '
           << std::setprecision(3) << pose.yMm << ' '
           << std::setprecision(3) << pose.zMm << ' '
           << std::setprecision(4) << pose.rxDeg << ' '
           << std::setprecision(4) << pose.ryDeg << ' '
           << std::setprecision(4) << pose.rzDeg << ' '
           << std::setprecision(4) << pose.s1Deg << ' '
           << std::setprecision(4) << pose.s2Deg << ' '
           << std::setprecision(6) << timeDeltaSeconds << '\n';
    if (!stream) {
        return false;
    }
    ++m_trajectoryPointCount;
    return true;
}

bool CTeachTrajectoryRecorder::appendTeachPoint(const Pose &pose)
{
    std::ofstream stream(seamTeachPath(), std::ios::app);
    if (!stream) {
        return false;
    }
    stream << std::fixed
           << std::setprecision(3) << pose.xMm << ' '
           << std::setprecision(3) << pose.yMm << ' '
           << std::setprecision(3) << pose.zMm << ' '
           << std::setprecision(4) << pose.rxDeg << ' '
           << std::setprecision(4) << pose.ryDeg << ' '
           << std::setprecision(4) << pose.rzDeg << ' '
           << std::setprecision(4) << pose.s1Deg << ' '
           << std::setprecision(4) << pose.s2Deg << '\n';
    if (!stream) {
        return false;
    }
    ++m_teachPointCount;
    return true;
}

bool CTeachTrajectoryRecorder::appendAllPoint(
    const robot_comm::RobotState &state)
{
    std::ofstream stream(allPointsPath(), std::ios::app);
    if (!stream) {
        return false;
    }
    stream << state.curx_r << ' '
           << state.cury_r << ' '
           << state.curz_r << ' '
           << state.curRx_r << ' '
           << state.curRy_r << ' '
           << state.curRz_r << ' '
           << state.curS1 << ' '
           << state.curS2 << '\n';
    if (!stream) {
        return false;
    }
    ++m_allPointCount;
    return true;
}

std::filesystem::path CTeachTrajectoryRecorder::seamTrajectoryPath() const
{
    return m_tempDirectory / "SeamTraj.txt";
}

std::filesystem::path CTeachTrajectoryRecorder::seamTeachPath() const
{
    return m_tempDirectory / "SeamTeach.txt";
}

std::filesystem::path CTeachTrajectoryRecorder::allPointsPath() const
{
    return m_tempDirectory / "AllPts.txt";
}

bool CTeachTrajectoryRecorder::isSafeBaseName(const std::string &baseName)
{
    static const std::regex pattern("^[A-Za-z0-9_-]+$");
    return !baseName.empty()
        && std::regex_match(baseName, pattern)
        && baseName.find(".") == std::string::npos
        && baseName.find("/") == std::string::npos
        && baseName.find("\\") == std::string::npos;
}

bool CTeachTrajectoryRecorder::fileHasLegacyContent(
    const std::filesystem::path &path)
{
    std::error_code error;
    return std::filesystem::exists(path, error)
        && !error
        && std::filesystem::file_size(path, error) > 3
        && !error;
}

} // namespace i2r_rlc::control
