/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsOnlineService.h
 * @brief     IBVS 在线验证服务。
 * @details   本文件定义将 IBVS 纯算法串成可启动、可停止、可单帧诊断的服务边界。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 15 日
 */

#ifndef I2R_RLC_IBVS_IBVS_ONLINE_SERVICE_H
#define I2R_RLC_IBVS_IBVS_ONLINE_SERVICE_H

#include "ibvs/IbvsTextConfigReader.h"
#include "ibvs/IIbvsService.h"

namespace i2r_rlc::ibvs {

/**
 * @brief IBVS 在线验证服务实现。
 *
 * 本类不访问 QWidget、真实相机 SDK 或 RobotComm；调用方负责把图像和外部
 * 观测转换为 `IbvsOnlineFrameInput`。本服务只生成速度和命令草案，不发送
 * 真实机器人命令，便于在独立模块内验证求解链路。
 */
class IbvsOnlineService final : public IIbvsService
{
public:
    IbvsOnlineService();

    IbvsStartResult start() override;
    IbvsStartResult configure(const IbvsServiceConfig& config) override;
    IbvsStartResult startDryRun() override;
    IbvsStartResult stop() override;
    IbvsOnlineStepResult processFrame(const IbvsOnlineFrameInput& input) override;
    IbvsServiceState state() const override;
    IbvsOnlineStepResult lastDiagnostics() const override;

private:
    static std::string makeDryRunCommand(const IbvsVelocityVector& velocityMps);
    static bool isFiniteFeature(const IbvsFeatureVector& feature);

    IbvsStartResult makeStartResult(const std::string& message) const;
    void setError(const std::string& message);

    IbvsServiceState m_state = IbvsServiceState::Idle;
    IbvsRuntimeTextConfig m_runtimeConfig;
    IbvsPositionConstraint m_positionConstraint;
    bool m_hasPositionConstraint = false;
    IbvsOnlineStepResult m_lastDiagnostics;
};

} // namespace i2r_rlc::ibvs

#endif // I2R_RLC_IBVS_IBVS_ONLINE_SERVICE_H
