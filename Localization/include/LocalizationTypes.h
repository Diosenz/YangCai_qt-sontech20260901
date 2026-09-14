/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      LocalizationTypes.h
 * @brief     定位模块公共值对象与诊断结构。
 * @details   本文件定义全局 ICP、局部修正、虚拟参考输入和流程结果共享的数据类型。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 11 日
 */

#ifndef I2R_RLC_LOCALIZATION_LOCALIZATION_TYPES_H
#define I2R_RLC_LOCALIZATION_LOCALIZATION_TYPES_H

#include <array>
#include <vector>

#include <QMetaType>
#include <QString>
#include <QtGlobal>

namespace i2r_rlc::localization {

/**
 * @brief 定位任务类型。
 *
 * 全局定位对应离线扫描后的整件工件位姿修正；局部定位对应在线识别或焊接过程中的
 * 滑窗式局部修正。具体 PCL 点云和 ICP 实现应留在 localization 模块内部。
 */
enum class LocalizationMode {
    Global,
    Local
};

/**
 * @brief 定位执行状态。
 */
enum class LocalizationStatus {
    NotStarted,
    AlgorithmNotImplemented,
    Succeeded,
    Failed
};

/**
 * @brief 定位服务运行状态。
 */
enum class LocalizationServiceState {
    Unavailable,
    Idle,
    Running,
    Error
};

/**
 * @brief ICP 点云预处理策略。
 */
enum class IcpPreprocessingProfile {
    /// @brief 通用策略，按参数对真实/参考点云做轻量预处理。
    Generic,

    /// @brief 兼容旧版局部 ICP 回放数据的 FPS/SOR/FPS 预处理策略。
    CompatibilityFirstLocalIcp,

    /// @brief 兼容 SWv1 stepICPforOffline 的 FPS(1/5)+SOR 全局定位策略。
    CompatibilitySwv1GlobalIcp
};

/**
 * @brief 三维点，坐标单位为 mm。
 *
 * 定位模块的 ICP 输入统一使用变位机 TCP 或调用方声明的同一工作坐标系下的 mm 点云。
 * PCL 点类型只允许出现在 localization 私有实现中。
 */
struct Point3D_mm {
    double x_mm = 0.0;
    double y_mm = 0.0;
    double z_mm = 0.0;
};

/**
 * @brief 六自由度位姿或位姿增量。
 */
struct Pose6D {
    double x_mm = 0.0;
    double y_mm = 0.0;
    double z_mm = 0.0;
    double rx_deg = 0.0;
    double ry_deg = 0.0;
    double rz_deg = 0.0;
};

/**
 * @brief 齐次刚体变换，按行优先存储 4x4 矩阵。
 */
struct RigidTransform {
    std::array<double, 16> rowMajor = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    };
};

/**
 * @brief ICP 可优化自由度。
 *
 * 公开接口使用显式 bool 字段表达平移和旋转自由度，不向调用方暴露第三方库内部类型。
 */
struct IcpDofMask {
    bool allowX = true;
    bool allowY = true;
    bool allowZ = true;
    bool allowRx = false;
    bool allowRy = false;
    bool allowRz = true;
};

/**
 * @brief ICP 参数。
 */
struct IcpParameters {
    /// @brief 点云预处理策略。
    IcpPreprocessingProfile preprocessingProfile = IcpPreprocessingProfile::Generic;

    /// @brief 参与配准的最少真实/参考点数。
    int minPointCount = 10;

    /// @brief 最大迭代次数；小于等于 0 时保留 PCL 默认值。
    int maximumIterations = 100;

    /// @brief 最大对应距离，单位 mm；小于等于 0 时使用 PCL 默认值。
    double maxCorrespondenceDistance_mm = 10.0;

    /// @brief LM 变换收敛阈值。
    double transformationEpsilon = 1e-5;

    /// @brief 是否对真实点云执行统计离群点滤波。
    bool enableSorFilter = true;

    /// @brief SOR 近邻数量。
    int sorMeanK = 5;

    /// @brief SOR 标准差阈值。
    double sorStddevMulThresh = 2.0;

    /// @brief 目标降采样点数；小于等于 0 时不降采样。
    int downsampleTargetPointCount = 0;
};

/**
 * @brief 全局 ICP 定位请求。
 */
struct GlobalIcpRequest {
    std::vector<Point3D_mm> realPoints_mm;
    std::vector<Point3D_mm> referencePoints_mm;
    IcpDofMask dofMask;
    IcpParameters parameters;
    RigidTransform initialGuess;
};

/**
 * @brief 局部滑窗修正请求。
 */
struct LocalCorrectionRequest {
    std::vector<Point3D_mm> realWindowPoints_mm;
    std::vector<Point3D_mm> referenceWindowPoints_mm;
    IcpDofMask dofMask;
    IcpParameters parameters;
    Pose6D currentWorkpiecePose;
    int windowFrameCount = 0;
};

/**
 * @brief 局部定位离线流程请求。
 *
 * 该请求表达已经累计好的真实/虚拟窗口点云。数字孪生、视觉线程和 UI 状态门控
 * 不在此结构中表达，由上层模块负责。
 */
struct LocalCorrectionPipelineRequest {
    std::vector<Point3D_mm> realWindowPoints_mm;
    std::vector<Point3D_mm> virtualWindowPoints_mm;
    IcpDofMask dofMask;
    IcpParameters parameters;
    Pose6D currentWorkpiecePose;
    qint64 frameId = -1;
    int windowFrameCount = 0;
    int minimumWindowFrameCount = 1;
};

/**
 * @brief 虚拟参考数据请求。
 *
 * 该请求用于从数字孪生或离线仿真服务获取参考点云、虚拟激光线和参考位姿。
 */
struct LocalizationVirtualReferenceRequest {
    qint64 frameId = -1;
    QString referenceId;
    Pose6D workpiecePose;
    bool needWorkpiecePointCloud = true;
    bool needLaserLinePoints = false;
};

/**
 * @brief 虚拟参考数据快照。
 *
 * 定位流程只消费已经提供的点云快照；快照来源可以是离线文件、仿真服务或测试数据。
 */
struct LocalizationVirtualReferenceSnapshot {
    bool available = false;
    QString referenceId;
    Pose6D referencePose;
    std::vector<std::array<double, 2>> laserLineImagePoints_px;
    std::vector<Point3D_mm> workpiecePointCloud_mm;
    std::vector<Point3D_mm> laserLinePoints_mm;
    QString message;
};

/**
 * @brief 全局 ICP 分段输入。
 */
struct GlobalIcpSegmentRequest {
    QString segmentId;
    std::vector<Point3D_mm> realPoints_mm;
    std::vector<Point3D_mm> virtualReferencePoints_mm;
    IcpDofMask dofMask;
    IcpParameters parameters;
    RigidTransform initialGuess;
};

/**
 * @brief 全局 ICP 流程请求。
 *
 * `segments` 为空时，流程使用整件真实扫描点云和虚拟参考点云执行单段 ICP。
 * `segments` 非空时，每段可声明独立点云、自由度和参数。
 */
struct GlobalIcpPipelineRequest {
    qint64 frameId = -1;
    QString requestId;
    std::vector<Point3D_mm> realScanPoints_mm;
    std::vector<Point3D_mm> virtualReferencePoints_mm;
    LocalizationVirtualReferenceSnapshot virtualReferenceSnapshot;
    std::vector<GlobalIcpSegmentRequest> segments;
    IcpDofMask defaultDofMask;
    IcpParameters defaultParameters;
    RigidTransform initialGuess;
    Pose6D currentWorkpiecePose;
};

/**
 * @brief 定位诊断信息。
 */
struct LocalizationDiagnostics {
    int sourcePointCount = 0;
    int targetPointCount = 0;
    int filteredSourcePointCount = 0;
    bool converged = false;
    double fitnessScore_mm2 = 0.0;
    bool refinementAttempted = false;
    bool refinementConverged = false;
    bool refinementAccepted = false;
    double coarseFitnessScore_mm2 = 0.0;
    double refinementFitnessScore_mm2 = 0.0;
    double refinementMaxCorrespondenceDistance_mm = 0.0;
};

/**
 * @brief 定位请求。
 *
 * 该结构表达应用层可见的定位输入，不暴露 `pcl::PointCloud`。
 */
struct LocalizationRequest {
    /// @brief 定位模式。
    LocalizationMode mode = LocalizationMode::Global;

    /// @brief 关联帧 ID，单位为帧；无关联帧时为 -1。
    qint64 frameId = -1;

    /// @brief 真实观测点数量，单位为点。
    int realPointCount = 0;

    /// @brief 虚拟参考点数量，单位为点。
    int virtualPointCount = 0;

    /// @brief 请求说明，通常用于日志或调试。
    QString message;

    /// @brief 全局 ICP 请求；`mode == Global` 时由真实服务消费。
    GlobalIcpRequest globalIcp;

    /// @brief 局部修正请求；`mode == Local` 时由真实服务消费。
    LocalCorrectionRequest localCorrection;
};

/**
 * @brief 定位结果。
 *
 * 位姿修正字段表达定位算法输出的工件位姿增量。单位在字段名中明确。
 */
struct LocalizationResult {
    /// @brief 定位模式。
    LocalizationMode mode = LocalizationMode::Global;

    /// @brief 关联帧 ID，单位为帧；无关联帧时为 -1。
    qint64 frameId = -1;

    /// @brief 定位状态。
    LocalizationStatus status = LocalizationStatus::NotStarted;

    /// @brief 真实观测点数量，单位为点。
    int realPointCount = 0;

    /// @brief 虚拟参考点数量，单位为点。
    int virtualPointCount = 0;

    /// @brief 是否包含有效位姿修正。
    bool hasPoseCorrection = false;

    /// @brief X 方向位移修正，单位 mm。
    double correctionX_mm = 0.0;

    /// @brief Y 方向位移修正，单位 mm。
    double correctionY_mm = 0.0;

    /// @brief Z 方向位移修正，单位 mm。
    double correctionZ_mm = 0.0;

    /// @brief RX 方向角度修正，单位 deg。
    double correctionRx_deg = 0.0;

    /// @brief RY 方向角度修正，单位 deg。
    double correctionRy_deg = 0.0;

    /// @brief RZ 方向角度修正，单位 deg。
    double correctionRz_deg = 0.0;

    /// @brief ICP 最近邻 RMSE，单位 mm；未执行算法时为 0。
    double residual_mm = 0.0;

    /// @brief 算法迭代次数；未执行算法时为 0。
    int iterationCount = 0;

    /// @brief 配准得到的齐次变换。
    RigidTransform correctionTransform;

    /// @brief ICP 实际消费的预处理后真实点云，单位 mm，用于现场诊断和落盘。
    std::vector<Point3D_mm> preprocessedRealPoints_mm;

    /// @brief 诊断信息。
    LocalizationDiagnostics diagnostics;

    /// @brief 结果说明或失败原因。
    QString message;
};

/**
 * @brief 局部定位离线流程结果。
 *
 * `icpResult.correctionTransform` 是真实点云到虚拟点云的 ICP 变换；
 * `virtualToRealCorrectionTransform` 是其逆矩阵，可用于修正虚拟点云或由调用方左乘更新工件位姿。
 */
struct LocalCorrectionPipelineResult {
    LocalizationStatus status = LocalizationStatus::NotStarted;
    LocalizationResult icpResult;
    bool hasCorrectedVirtualPoints = false;
    std::vector<Point3D_mm> correctedVirtualWindowPoints_mm;
    RigidTransform virtualToRealCorrectionTransform;
    double beforeResidual_mm = 0.0;
    double afterResidual_mm = 0.0;
    QString message;
};

/**
 * @brief 全局 ICP 分段结果。
 */
struct GlobalIcpSegmentResult {
    QString segmentId;
    LocalizationResult icpResult;
    RigidTransform virtualToRealCorrectionTransform;
    bool hasCorrectedVirtualPoints = false;
    std::vector<Point3D_mm> correctedVirtualReferencePoints_mm;
    /// @brief 应用本段累计初值后的最近邻 RMSE，单位 mm。
    double beforeResidual_mm = 0.0;
    /// @brief 应用本段最终变换后的最近邻 RMSE，单位 mm。
    double afterResidual_mm = 0.0;
    QString message;
};

/**
 * @brief 全局 ICP 流程结果。
 */
struct GlobalIcpPipelineResult {
    LocalizationStatus status = LocalizationStatus::NotStarted;
    std::vector<GlobalIcpSegmentResult> segmentResults;
    RigidTransform combinedVirtualToRealCorrectionTransform;
    bool hasCorrectedVirtualPoints = false;
    std::vector<Point3D_mm> correctedVirtualReferencePoints_mm;
    /// @brief 初始变换下各定位面按真实点数加权的最近邻 RMSE，单位 mm。
    double beforeResidual_mm = 0.0;
    /// @brief 最终统一变换下各定位面按真实点数加权的最近邻 RMSE，单位 mm。
    double afterResidual_mm = 0.0;
    /// @brief 初始变换下各定位面等权平均的最近邻 RMSE，单位 mm。
    double beforeUnweightedResidual_mm = 0.0;
    /// @brief 最终统一变换下各定位面等权平均的最近邻 RMSE，单位 mm。
    double afterUnweightedResidual_mm = 0.0;
    QString message;
};

/**
 * @brief 定位服务启动结果。
 */
struct LocalizationStartResult {
    LocalizationServiceState state = LocalizationServiceState::Unavailable;
    QString message;
};

} // namespace i2r_rlc::localization

Q_DECLARE_METATYPE(i2r_rlc::localization::LocalizationRequest)
Q_DECLARE_METATYPE(i2r_rlc::localization::LocalizationResult)

#endif // I2R_RLC_LOCALIZATION_LOCALIZATION_TYPES_H
