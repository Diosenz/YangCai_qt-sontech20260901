/*******************************************************************************
 * @file        TeachTrajectoryRecorder.h
 * @brief       旧式示教轨迹保存记录器。
 * @details     根据 RobotState::s_Bsavetraj 复刻旧 RobotCtrl 自动记录
 *              SeamTraj/SeamTeach/AllPts 的文件语义。
 ******************************************************************************/
#pragma once

#include <chrono>
#include <filesystem>
#include <string>

#include "RobotState.h"

namespace i2r_rlc::control {

/**
 * @brief 示教轨迹记录状态快照。
 */
struct TeachTrajectoryRecorderSnapshot {
    bool succeeded = true;
    bool recordingEnabled = false;
    bool segmentActive = false;
    long latestSaveSignal = 0;
    int trajectoryPointCount = 0;
    int teachPointCount = 0;
    int allPointCount = 0;
    std::string tempDirectory;
    std::string seamTrajectoryFile;
    std::string seamTeachFile;
    std::string allPointsFile;
    std::string outputWorkPathFile;
    std::string outputTeachPoseFile;
    std::string message;
};

/**
 * @brief 保存示教轨迹请求。
 */
struct TeachTrajectorySaveRequest {
    std::string outputDirectory;
    std::string baseName;
};

/**
 * @brief 与机器人JBI程序联动，保存示教轨迹。
 *
 * 位置输入单位来自 RobotState 协议整数：um；姿态与变位机轴为 deg*10000。
 * 临时 `_W` 文件输出单位为 mm/deg，并在末列写入相邻采样时间间隔，单位 s。
 */
class CTeachTrajectoryRecorder final {
public:
    explicit CTeachTrajectoryRecorder(std::filesystem::path tempDirectory = {});

    /**
     * @brief 设置临时文件目录。
     * @param tempDirectory 目录路径。
     */
    TeachTrajectoryRecorderSnapshot configureTempDirectory(
        const std::filesystem::path &tempDirectory);

    /**
     * @brief 清空临时文件并开始接收 s_Bsavetraj 自动记录。
     */
    TeachTrajectoryRecorderSnapshot startRecording();

    /**
     * @brief 输入最新机器人状态，必要时追加临时轨迹文件。
     * @param state 机器人状态快照。
     */
    TeachTrajectoryRecorderSnapshot onRobotStateChanged(
        const robot_comm::RobotState &state);

    /**
     * @brief 将临时 SeamTraj/SeamTeach 保存为 `<base>_W.txt` 与 `<base>_T.txt`。
     */
    TeachTrajectoryRecorderSnapshot saveCapturedTrajectory(
        const TeachTrajectorySaveRequest &request);

    /**
     * @brief 停止记录并清空内存锁存状态；不删除已写临时文件。
     */
    TeachTrajectoryRecorderSnapshot reset();

    /**
     * @brief 当前状态快照。
     */
    TeachTrajectoryRecorderSnapshot snapshot() const;

private:
    struct Pose {
        double xMm = 0.0;
        double yMm = 0.0;
        double zMm = 0.0;
        double rxDeg = 0.0;
        double ryDeg = 0.0;
        double rzDeg = 0.0;
        double s1Deg = 0.0;
        double s2Deg = 0.0;
    };

    TeachTrajectoryRecorderSnapshot makeSnapshot(
        bool succeeded,
        std::string message) const;
    TeachTrajectoryRecorderSnapshot fail(std::string message) const;

    /**
     * @brief 从机器人快照中获取位姿。
     * @param state 机器人快照。
     */
    Pose poseFromState(const robot_comm::RobotState &state) const;

    /**
     * @brief 与上一点的欧式距离。
     * @param pose 当前位姿。
     */
    double distanceFromLast(const Pose &pose) const;
    bool ensureTempFiles(std::string *errorMessage) const;
    /**
     * @brief 添加轨迹点到文件。
     * @param pose 位姿数据结构体。
     * @param timeDeltaSeconds 两点之间的时间间隔
     */
    bool appendTrajectoryPoint(const Pose &pose, double timeDeltaSeconds);
    bool appendTeachPoint(const Pose &pose);
    bool appendAllPoint(const robot_comm::RobotState &state);
    std::filesystem::path seamTrajectoryPath() const;
    std::filesystem::path seamTeachPath() const;
    std::filesystem::path allPointsPath() const;
    static bool isSafeBaseName(const std::string &baseName);

    /**
     * @brief 检查文件目录是否存在
     * @path 目录路径名称
     */
    static bool fileHasLegacyContent(const std::filesystem::path &path);

    std::filesystem::path m_tempDirectory;
    bool m_recordingEnabled = false;
    bool m_segmentActive = false;
    Pose m_lastPose;
    int m_testCount = 0;
    int m_trajectoryPointCount = 0;
    int m_teachPointCount = 0;
    int m_allPointCount = 0;
    long m_latestSaveSignal = 0;
    std::chrono::steady_clock::time_point m_lastTrajectoryTick;
};

} // namespace i2r_rlc::control
