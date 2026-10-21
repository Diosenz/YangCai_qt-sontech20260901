/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      MotomanMh12InverseKinematics.h
 * @brief     由真实 TCP 位姿反解 Motoman MH12 六轴关节角。
 * @details   保留 SWv1 中“控制柜 TCP -> 软件机器人模型逆解”的换算边界，
 *            输出可直接作为 RobotSim S/L/U/R/B/T 关节值的弧度数组。
 */

#ifndef I2R_RLC_DIGITAL_TWIN_MOTOMAN_MH12_INVERSE_KINEMATICS_H
#define I2R_RLC_DIGITAL_TWIN_MOTOMAN_MH12_INVERSE_KINEMATICS_H

#include <array>
#include <string>

namespace i2r_rlc::digital_twin {

/**
 * @brief 真实机器人 TCP 在机器人基坐标系下的位姿。
 */
struct MotomanMh12TcpPose {
    double xMm = 0.0;
    double yMm = 0.0;
    double zMm = 0.0;
    double rxDeg = 0.0;
    double ryDeg = 0.0;
    double rzDeg = 0.0;
};

/**
 * @brief MH12 逆解结果。
 */
struct MotomanMh12InverseKinematicsResult {
    /// @brief true 表示已找到满足关节限位的逆解。
    bool succeeded = false;
    /// @brief 关节顺序为 S/L/U/R/B/T，单位 rad。
    std::array<double, 6> jointAnglesRad{};
    /// @brief 失败时的简短原因，成功时为空。
    std::string message;
};

/**
 * @brief Motoman MH12 有状态解析逆解器。
 *
 * 求解器会保留上一帧成功解，用于在多组合法逆解中保持关节分支连续。
 * 对象应固定由一个工作线程使用，不支持并发调用。
 */
class CMotomanMh12InverseKinematics final {
public:
    CMotomanMh12InverseKinematics();

    /**
     * @brief 由当前真实 TCP 位姿计算六轴关节角。
     * @param tcpPose TCP 位姿，平移单位 mm，RPY 单位 deg。
     * @return 逆解结果；失败时保持上一帧成功解不变。
     */
    MotomanMh12InverseKinematicsResult solve(
        const MotomanMh12TcpPose &tcpPose);

    /**
     * @brief 将逆解分支参考重置为 SWv1 的 MH12 软件零位。
     */
    void reset();

private:
    std::array<double, 6> m_previousInternalJointAnglesRad{};
};

} // namespace i2r_rlc::digital_twin

#endif // I2R_RLC_DIGITAL_TWIN_MOTOMAN_MH12_INVERSE_KINEMATICS_H
