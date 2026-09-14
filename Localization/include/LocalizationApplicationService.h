/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      LocalizationApplicationService.h
 * @brief     定位 UI 应用层门面。
 * @details   本文件定义 Localization 模块内部的应用层编排入口，用于把真实
 *            ICP 服务、局部修正 pipeline、全局 ICP pipeline 和流程状态机组合为
 *            UI 可调用的离线验证能力。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 16 日
 */

#ifndef I2R_RLC_LOCALIZATION_LOCALIZATION_APPLICATION_SERVICE_H
#define I2R_RLC_LOCALIZATION_LOCALIZATION_APPLICATION_SERVICE_H

#include <vector>

#include <QString>

#include "LocalizationGlobalIcpPipeline.h"
#include "LocalizationLocalCorrectionPipeline.h"
#include "LocalizationService.h"
#include "LocalizationWorkflowController.h"

namespace i2r_rlc::localization {

/**
 * @brief 局部修正请求。
 */
struct LocalizationLocalReplayRequest {
    QString pointCloudRootPath;
    int minimumWindowFrameCount = 20;
};

/**
 * @brief 全局定位请求。
 */
struct LocalizationGlobalDryRunRequest {
    qint64 frameId = -1;
    IcpDofMask dofMask;
};

/**
 * @brief 运行时全局定位请求。
 *
 * 该请求由在线/离线纠偏等真实流程传入已采集的真实扫描点，不允许回退到
 * dry-run 示例点云。
 */
struct LocalizationGlobalRuntimeRequest {
    qint64 frameId = -1;
    IcpDofMask dofMask;
    std::vector<Point3D_mm> realScanPoints_mm;
    LocalizationVirtualReferenceSnapshot virtualReferenceSnapshot;
};

/**
 * @brief 运行时逐段全局定位请求。
 *
 * 与 SWv1 updateModelOfflineICP() 的逐段语义一致：每段携带真实点云、
 * 该段虚拟参考点云与段级自由度掩码，由 GlobalIcpPipeline 逐段配准后组合。
 */
struct LocalizationGlobalRuntimeSegmentedRequest {
    qint64 frameId = -1;
    /// @brief 逐段输入，段序与工件 position_face_dof_list 一致。
    std::vector<GlobalIcpSegmentRequest> segments;
    LocalizationVirtualReferenceSnapshot virtualReferenceSnapshot;
    IcpDofMask defaultDofMask;
    IcpParameters defaultParameters;
    RigidTransform initialGuess;
};

/**
 * @brief 定位应用层运行结果。
 */
struct LocalizationApplicationResult {
    LocalizationWorkflowUpdate update;
    bool hasLocalCorrectionResult = false;
    LocalCorrectionPipelineResult localCorrectionResult;
    bool hasGlobalIcpResult = false;
    GlobalIcpPipelineResult globalIcpResult;
    QString replayDirectoryPath;
};

/**
 * @brief Localization 模块应用层门面。
 *
 * 该门面为 UI 提供可触发的定位和离线入口。它不依赖 Control/Vision，
 * 不连接 RobotSim，不发送真实机器人命令，也不写回工件模型。
 */
class LocalizationApplicationService final {
public:
    LocalizationApplicationService();

    /**
     * @brief 启动或声明定位服务状态。
     */
    LocalizationStartResult start();

    /**
     * @brief 执行一次局部修正。
     */
    LocalizationApplicationResult runLocalCorrectionReplay(
        const LocalizationLocalReplayRequest& request);

    /**
     * @brief 执行一次全局定位。
     */
    LocalizationApplicationResult runGlobalIcpDryRun(
        const LocalizationGlobalDryRunRequest& request);

    /**
     * @brief 使用调用方提供的虚拟参考快照执行一次全局定位。
     *
     * 该入口用于 UI 或流程编排层把 RobotSim/DigitalTwin 已查询到的参考数据
     * 注入定位流程；未连接仿真时仍应调用无快照重载，保持显式失败语义。
     */
    LocalizationApplicationResult runGlobalIcpDryRun(
        const LocalizationGlobalDryRunRequest& request,
        const LocalizationVirtualReferenceSnapshot& virtualReferenceSnapshot);

    /**
     * @brief 使用真实扫描点执行一次全局定位。
     *
     * 该入口用于机器人 JBI 已完成扫描后的在线一键全自动流程；缺少真实点云或
     * 虚拟参考时会显式失败，不使用 dry-run 数据兜底。
     */
    LocalizationApplicationResult runGlobalIcpRuntime(
        const LocalizationGlobalRuntimeRequest& request);

    /**
     * @brief 使用逐段输入执行一次全局定位。
     *
     * segments 为空时回退为单段（全部真实点 + 虚拟参考快照）。
     */
    LocalizationApplicationResult runGlobalIcpRuntimeSegmented(
        const LocalizationGlobalRuntimeSegmentedRequest& request);

    /**
     * @brief 清除定位流程状态。
     */
    LocalizationWorkflowUpdate clear();

    /**
     * @brief 当前定位流程状态。
     */
    LocalizationWorkflowState workflowState() const;

private:
    LocalizationApplicationResult failApplicationResult(
        LocalizationMode mode,
        const QString& message);

    LocalizationService m_localizationService;
    LocalizationLocalCorrectionPipeline m_localCorrectionPipeline;
    LocalizationGlobalIcpPipeline m_globalIcpPipeline;
    LocalizationWorkflowController m_workflowController;
};

} // namespace i2r_rlc::localization

#endif // I2R_RLC_LOCALIZATION_LOCALIZATION_APPLICATION_SERVICE_H
