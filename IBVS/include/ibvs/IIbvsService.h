/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IIbvsService.h
 * @brief     IBVS 服务抽象接口。
 * @details   本文件定义 UI 或运行时组装层可依赖的 IBVS 服务状态和启动入口。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 8 日
 */

#ifndef I2R_RLC_IBVS_I_IBVS_SERVICE_H
#define I2R_RLC_IBVS_I_IBVS_SERVICE_H

#include <optional>
#include <string>

#include "ibvs/IbvsSensorFusion.h"
#include "ibvs/IbvsTypes.h"
#include "ibvs/IbvsUnifiedVelocitySolver.h"

namespace i2r_rlc::ibvs {

/**
 * @brief IBVS 服务状态。
 */
enum class IbvsServiceState {
    Unavailable,
    Idle,
    Configured,
    DryRunReady,
    RunningDryRun,
    Running,
    Error
};

/**
 * @brief IBVS 服务启动结果。
 */
struct IbvsStartResult {
    IbvsServiceState state = IbvsServiceState::Unavailable;
    std::string message;
};

/**
 * @brief IBVS 在线服务配置输入。
 *
 * 路径由上层传入，IBVS 服务只读取 txt 并转成算法值对象；本结构不持有
 * UI、Camera 或 RobotComm 对象。
 */
struct IbvsServiceConfig
{
    std::string runtimeDefaultsPath;
    std::string workpieceProfilesPath;
    std::string workpieceId;
    int workpieceType = 4;
};

/**
 * @brief 单帧在线验证输入。
 *
 * 调用方负责提供已经转换好的算法值对象；图像检测、设备采集和外部状态同步
 * 在服务边界之外的适配层完成。
 */
struct IbvsOnlineFrameInput
{
    int frameIndex = -1;
    IbvsFeatureVector currentFeature = IbvsFeatureVector::Zero();
    IbvsFeatureVector fallbackDesiredFeature = IbvsFeatureVector::Zero();
    IbvsJacobianMatrix cameraJacobian = IbvsJacobianMatrix::Zero();
    std::optional<IbvsTorchObservation> torchObservation;
    std::optional<IbvsLvsObservation> lvsObservation;
    std::optional<IbvsLvsConstraint> lvsConstraint;
    std::optional<IbvsPositionConstraint> positionConstraint;
};

/**
 * @brief IBVS dry-run 单步结果。
 */
struct IbvsOnlineStepResult
{
    IbvsError error;
    IbvsServiceState state = IbvsServiceState::Unavailable;
    int frameIndex = -1;
    IbvsFeatureVector currentFeature = IbvsFeatureVector::Zero();
    IbvsFeatureVector desiredFeature = IbvsFeatureVector::Zero();
    Eigen::Vector4d featureErrorPx = Eigen::Vector4d::Zero();
    IbvsVelocityVector velocityMps = IbvsVelocityVector::Zero();
    bool velocityLimited = false;
    bool usedLvsPrior = false;
    bool usedLvsConstraint = false;
    bool usedPositionConstraint = false;
    int rowCount = 0;
    double dampingMu = 0.0;
    std::string dryRunCommand;
    std::string message;
};

/**
 * @brief IBVS 服务接口。
 */
class IIbvsService {
public:
    virtual ~IIbvsService() = default;

    /**
     * @brief 启动或声明服务状态。
     */
    virtual IbvsStartResult start() = 0;

    /**
     * @brief 加载运行配置并进入 configured 状态。
     */
    virtual IbvsStartResult configure(const IbvsServiceConfig& config) = 0;

    /**
     * @brief 启动不发送真实机器人命令的在线验证流程。
     */
    virtual IbvsStartResult startDryRun() = 0;

    /**
     * @brief 停止服务，重复调用必须安全。
     */
    virtual IbvsStartResult stop() = 0;

    /**
     * @brief 处理一帧在线验证输入并生成速度和命令草案。
     */
    virtual IbvsOnlineStepResult processFrame(
        const IbvsOnlineFrameInput& input) = 0;

    /**
     * @brief 返回当前服务状态。
     */
    virtual IbvsServiceState state() const = 0;

    /**
     * @brief 返回最近一次在线验证诊断。
     */
    virtual IbvsOnlineStepResult lastDiagnostics() const = 0;
};

} // namespace i2r_rlc::ibvs

#endif // I2R_RLC_IBVS_I_IBVS_SERVICE_H
