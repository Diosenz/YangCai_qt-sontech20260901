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
 * @file        OnlineCorrectionTrajectoryPlanner.h
 * @brief       在线纠偏轨迹修正服务
 * @details     提供示教轨迹读取、分段、起点匹配和示教点修正的纯 C++ 接口。
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
#ifndef I2R_RLC_CONTROL_ONLINE_CORRECTION_TRAJECTORY_PLANNER_H
#define I2R_RLC_CONTROL_ONLINE_CORRECTION_TRAJECTORY_PLANNER_H

#include <string>
#include <vector>

#include "OnlineCorrectionTypes.h"

namespace i2r_rlc::control {

/**
 * @brief 示教轨迹分段请求。
 */
struct OnlineCorrectionTrajectoryRequest {
    /// @brief 工作轨迹文件路径，文本文件中每行至少包含 x/y/z，单位为 mm。
    std::string workPathFile;
    /// @brief 示教位姿文件路径，每行包含位置 mm、姿态 deg，可选变位机轴 deg。
    std::string teachPoseFile;
    /// @brief 当前轨迹分段索引。
    int segmentIndex = 0;
    /// @brief 起始示教点索引，负数表示按 segmentIndex 自动推断。
    int startTeachIndex = -1;
    /// @brief 结束示教点索引，负数表示按 startTeachIndex 自动推断。
    int endTeachIndex = -1;
    /// @brief 工作轨迹点最小保留间距，单位为 mm，0 表示不抽稀。
    double minPathPointDistanceMm = 0.0;
};

/**
 * @brief 解析和分段后的示教轨迹。
 */
struct OnlineCorrectionPlannedTrajectory {
    /// @brief true 表示轨迹和分段均解析成功。
    bool valid = false;
    /// @brief 当前轨迹分段索引。
    int segmentIndex = 0;
    /// @brief 当前分段起始示教点索引。
    int startTeachIndex = 0;
    /// @brief 当前分段结束示教点索引。
    int endTeachIndex = 0;
    /// @brief 全部工作轨迹点，坐标单位为 mm。
    std::vector<OnlineCorrectionPoint> allPathPoints;
    /// @brief 当前分段轨迹点，坐标单位为 mm。
    std::vector<OnlineCorrectionPoint> segmentPathPoints;
    /// @brief 旧 AllTpMatched 等价轨迹，坐标单位为 mm。
    std::vector<OnlineCorrectionPoint> allMatchedPathPoints;
    /// @brief 旧 AllTp_segement_line_and_curve 等价分段轨迹，坐标单位为 mm。
    std::vector<std::vector<OnlineCorrectionPoint>> pathSegments;
    /// @brief 示教机器人位姿，位置单位 mm，姿态单位 deg。
    std::vector<OnlineCorrectionPose> teachPoses;
    /// @brief 与 teachPoses 对应的变位机轴角，单位为 deg。
    std::vector<OnlineCorrectionStationAxis> stationAxes;
    /// @brief 解析说明或错误消息。
    std::string message;
};

/**
 * @brief 轨迹修正结果。
 */
struct OnlineCorrectionTrajectoryPlanResult {
    /// @brief true 表示初始纠偏规划成功。
    bool valid = false;
    /// @brief 本次纠偏使用的初始点，单位为 mm。
    OnlineCorrectionPoint initialPoint;
    /// @brief 平移或全局变换后的当前分段轨迹，单位为 mm。
    std::vector<OnlineCorrectionPoint> correctedSegmentPath;
    /// @brief 生成给机器人通信层的示教点更新命令。
    robot_comm::RobotCorrectionCommand command;
    /// @brief 规划说明或错误消息。
    std::string message;
};

/**
 * @brief 在线纠偏轨迹修正服务。
 *
 * 本类承接旧 `OnlineTeachTrajectoryInitial`、`OnlineReverseDetectMatch`、
 * `correctTeachTraj` 和 `correctTeachPts` 中已经进入 change 主线的计算语义。
 */
class COnlineCorrectionTrajectoryPlanner final {
public:
    /**
     * @brief 从文本文件加载工作轨迹和示教位姿，并按示教点定位分段。
     * @param request 文件路径、分段索引和抽稀参数。
     * @return 解析后的轨迹，失败时 valid 为 false 且 message 描述原因。
     */
    static OnlineCorrectionPlannedTrajectory loadTrajectory(
        const OnlineCorrectionTrajectoryRequest &request);

    /**
     * @brief 规划初始纠偏命令。
     * @param trajectory 已加载的示教轨迹。
     * @param options 起点模式、gap 检测和全局修正矩阵配置。
     * @param state 控制器当前检测样本状态。
     * @param robotState 当前机器人状态，IBVSStart 会读取 TCP 坐标。
     * @return 纠偏轨迹和机器人命令，失败时 valid 为 false。
     */
    static OnlineCorrectionTrajectoryPlanResult planInitialCorrection(
        const OnlineCorrectionPlannedTrajectory &trajectory,
        const OnlineCorrectionOptions &options,
        const OnlineCorrectionSessionState &state,
        const robot_comm::RobotState &robotState,
        const vision::PositionerCalibration &positionerCalibration);

    /**
     * @brief 兼容旧测试调用，使用单位标定；生产路径必须传入真实 RBCALIB。
     */
    static OnlineCorrectionTrajectoryPlanResult planInitialCorrection(
        const OnlineCorrectionPlannedTrajectory &trajectory,
        const OnlineCorrectionOptions &options,
        const OnlineCorrectionSessionState &state,
        const robot_comm::RobotState &robotState);

    /**
     * @brief 在给定范围内查找距离参考点最近的轨迹点。
     * @param beginIndex 起始索引，包含。
     * @param endIndex 结束索引，不包含。
     * @param points 候选点集合，坐标单位为 mm。
     * @param reference 参考点，坐标单位为 mm。
     * @return 最近点索引；输入无效时返回 -1。
     */
    static int findNearestPointIndex(
        int beginIndex,
        int endIndex,
        const std::vector<OnlineCorrectionPoint> &points,
        const OnlineCorrectionPoint &reference);

private:
    /**
     * @brief 读取工作轨迹点文本文件。
     * @param path 文件路径。
     * @param minDistanceMm 最小保留间距，单位为 mm。
     * @return 有效轨迹点集合，坐标单位为 mm。
     */
    static std::vector<OnlineCorrectionPoint> readPathPoints(
        const std::string &path,
        double minDistanceMm);

    /**
     * @brief 读取示教位姿和可选变位机轴角。
     * @param path 文件路径。
     * @param poses 输出示教位姿，位置单位 mm，姿态单位 deg。
     * @param axes 输出变位机轴角，单位 deg。
     * @return 至少读取到一个有效位姿时返回 true。
     */
    static bool readTeachPoses(
        const std::string &path,
        std::vector<OnlineCorrectionPose> &poses,
        std::vector<OnlineCorrectionStationAxis> &axes);
};

} // namespace i2r_rlc::control

#endif // I2R_RLC_CONTROL_ONLINE_CORRECTION_TRAJECTORY_PLANNER_H
