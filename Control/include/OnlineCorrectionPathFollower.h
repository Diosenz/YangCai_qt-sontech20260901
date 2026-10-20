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
 * @file        OnlineCorrectionPathFollower.h
 * @brief       在线纠偏实时路径跟随服务
 * @details     承接 CorrectTaughtPathO 的活动 PID、越过判断和 gap 调速语义。
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
#ifndef I2R_RLC_CONTROL_ONLINE_CORRECTION_PATH_FOLLOWER_H
#define I2R_RLC_CONTROL_ONLINE_CORRECTION_PATH_FOLLOWER_H

#include <array>
#include <deque>
#include <vector>

#include "OnlineCorrectionTypes.h"

namespace i2r_rlc::control {

struct OnlineCorrectionFollowResult {
    bool valid = false;
    bool completed = false;
    /// @brief 仅普通实时命令成功生成时填充的目标几何快照。
    OnlineCorrectionFollowDiagnostic diagnostic;
    /// @brief 本次实时计算使用的 `_W` 示教点索引，供运行诊断和回归测试使用。
    int taughtPointIndex = -1;
    robot_comm::RobotRealtimeCorrectionCommand command;
    std::string message;
};

/**
 * @brief 根据在线检测轨迹计算机器人实时纠偏增量。
 */
class COnlineCorrectionPathFollower final {
public:
    /**
     * @brief 计算下一条 P 实时纠偏命令。
     *
     * 仅在 Online worker 串行调用；函数会消费已越过、重复或方向异常的检测点，
     * 并更新十点 PI/PID 历史。
     */
    OnlineCorrectionFollowResult update(
        const robot_comm::RobotState &robotState,
        const OnlineCorrectionOptions &options,
        const vision::PositionerCalibration &calibration,
        const std::vector<OnlineCorrectionPoint> &teachTrajectory,
        std::vector<OnlineCorrectionDetectionSample> *detectionQueue,
        bool finishing);

    /// @brief 清空检测方向、控制历史和上次输出。
    void reset();

private:
    std::array<double, 10> m_errorDistanceHistory{};
    std::array<double, 10> m_directionXHistory{};
    std::array<double, 10> m_directionYHistory{};
    std::array<double, 10> m_directionZHistory{};
    std::array<double, 10> m_weldDirectionXHistory{};
    std::array<double, 10> m_weldDirectionYHistory{};
    std::array<double, 10> m_weldDirectionZHistory{};
    int m_historyCount = 0;
    int m_weldDirectionHistoryCount = 0;
    int m_controlCount = 0;
    int m_sameTargetCount = 0;
    OnlineCorrectionPoint m_lastTarget;
    double m_lastErrorDistanceMm = 0.0;
    double m_lastBaseDxUm = 0.0;
    double m_lastBaseDyUm = 0.0;
    double m_lastBaseDzUm = 0.0;
    long m_lastDxUm = 0;
    long m_lastDyUm = 0;
    long m_lastDzUm = 0;
    int m_teachPointCursor = -1;
};

} // namespace i2r_rlc::control

#endif // I2R_RLC_CONTROL_ONLINE_CORRECTION_PATH_FOLLOWER_H
