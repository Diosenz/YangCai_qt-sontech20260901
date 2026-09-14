/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        CorrectionRobotJobMonitor.cpp
 * @brief       纠偏流程机器人 JBI 作业完成边沿监视器实现
 ******************************************************************************/
#include "CorrectionRobotJobMonitor.h"

namespace i2r_rlc::ui {

void CCorrectionRobotJobMonitor::arm()
{
    m_armed = true;
    m_seenRunning = false;
}

void CCorrectionRobotJobMonitor::reset()
{
    m_armed = false;
    m_seenRunning = false;
}

bool CCorrectionRobotJobMonitor::update(const robot_comm::RobotState& state)
{
    if (!m_armed) {
        return false;
    }
    if (state.sStart != 0) {
        m_seenRunning = true;
        return false;
    }
    if (!m_seenRunning) {
        return false;
    }

    reset();
    return true;
}

bool CCorrectionRobotJobMonitor::armed() const
{
    return m_armed;
}

bool CCorrectionRobotJobMonitor::hasSeenRunning() const
{
    return m_seenRunning;
}

} // namespace i2r_rlc::ui
