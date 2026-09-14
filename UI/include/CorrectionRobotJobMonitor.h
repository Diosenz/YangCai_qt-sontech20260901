/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        CorrectionRobotJobMonitor.h
 * @brief       纠偏流程机器人 JBI 作业完成边沿监视器
 * @details     复刻旧工程 `job_done` 语义：必须先观察到机器人程序运行，
 *              再观察到程序停止，才认为本次 JBI 已经完成。
 ******************************************************************************/
#ifndef I2R_RLC_UI_CORRECTION_ROBOT_JOB_MONITOR_H
#define I2R_RLC_UI_CORRECTION_ROBOT_JOB_MONITOR_H

#include "RobotState.h"

namespace i2r_rlc::ui {

/**
 * @brief 机器人 JBI 作业完成边沿监视器。
 *
 * 本类只判断 `sStart` 的运行到停止边沿，不发送机器人命令，也不读取 UI。
 */
class CCorrectionRobotJobMonitor final {
public:
    /**
     * @brief 开始监视新发出的 JBI 作业。
     */
    void arm();

    /**
     * @brief 取消当前监视。
     */
    void reset();

    /**
     * @brief 输入机器人状态并判断作业是否完成。
     * @param state 最新机器人状态快照。
     * @return 已先看到 `sStart==1` 且当前回到 `sStart==0` 时返回 true。
     */
    bool update(const robot_comm::RobotState& state);

    /**
     * @brief 是否正在监视一个作业。
     */
    bool armed() const;

    /**
     * @brief 是否已经观察到本次作业进入运行态。
     */
    bool hasSeenRunning() const;

private:
    bool m_armed = false;
    bool m_seenRunning = false;
};

} // namespace i2r_rlc::ui

#endif // I2R_RLC_UI_CORRECTION_ROBOT_JOB_MONITOR_H
