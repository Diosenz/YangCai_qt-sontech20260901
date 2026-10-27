/**
 * @file LocalizationSurfaceValidation.h
 * @brief 基于真实母材表面的候选约束、留出验收与新帧度量。
 * @details 本接口不提取图像、不求解 ICP、不写场景，不接受预测焊点作实测。
 * 输入云必须由调用方确认同一工件父级系、同代次及真实图像支持；单位 mm。
 * 冻结虚拟激光切片的刚体变换只是候选近似，不能称为 CAD 重渲染验证。
 */
#ifndef I2R_RLC_LOCALIZATION_LOCALIZATION_SURFACE_VALIDATION_H
#define I2R_RLC_LOCALIZATION_LOCALIZATION_SURFACE_VALIDATION_H

#include <array>
#include <vector>

#include "LocalizationTypes.h"

namespace i2r_rlc::localization::surface_validation {

/** @brief 有界离线几何门槛；缺省为工程参数，尚非现场精度或运动许可。 */
struct Options {
    int minimumPointCount = 24;
    int maximumPointCount = 2048;
    int normalNeighborCount = 12;
    double minimumNormalSupportRatio = 0.6;
    double maximumNormalPlanarityRatio = 0.1;
    double minimumNeighborhoodSecondVarianceMm2 = 0.01;
    /// @brief 中心化 XYZ 与转角边缘位移同单位后的相对奇异值门槛。
    double minimumRelativeModeStrength = 0.05;
    double robustKeepRatio = 0.8;
    double coverageDistanceMm = 3.0;
    double minimumBidirectionalCoverage = 0.7;
    double maximumResidualMm = 5.5;
    double minimumImprovementRatio = 0.05;
    double maximumHoldoutWorseningMm = 0.1;
    /// @brief 上层连续拒绝后显式授权；仅放宽训练覆盖，留出/新帧门限保持原值。
    bool allowTrainingCoverageRecovery = false;
    /// @brief 仅在线后验：当前CAD切片距冻结近邻切向足迹的有限外延上限，mm。
    /// 不扩大真实点覆盖距离；调用方仍须限制会话、曝光及位姿版本。
    double maximumModelContinuationMm = 10.0;
    /// @brief 当前CAD切片对冻结面法向一致的容差，须小于恶化容差。
    double maximumModelPlaneDeviationMm = 0.05;
};

/** @brief 两向最近邻诊断；无材料身份时不代表真实对应或绝对定位精度。 */
struct Metrics {
    bool valid = false;
    double realToVirtualMeanMm = 0.0;
    double virtualToRealMeanMm = 0.0;
    double robustSymmetricResidualMm = 0.0;
    double realCoverageRatio = 0.0;
    double virtualCoverageRatio = 0.0;
};

/** @brief 新帧配对证据不足与有效配对必须保留不同的诊断原因。 */
enum class FreshSurfaceComparisonStatus {
    InvalidInput,
    InsufficientFrozenSupport,
    InsufficientCurrentModelSupport,
    InsufficientRealSupport,
    Succeeded
};

/**
 * @brief 同一新帧真实点对冻结局部面的配对误差，长度单位 mm。
 * @details 面仅从写回前训练虚拟云估计；同一局部面分别保持原位和应用候选，
 * 两个误差使用完全相同的新帧点及面身份，不比较不同时间的切片最近邻均值。
 * 这是有界局部平面近似，不是 CAD 重新求交；支持不足必须拒绝，不能外推整件。
 */
struct PairedSurfaceMetrics {
    bool valid = false;
    int supportedPointCount = 0;
    double supportedRatio = 0.0;
    double previousResidualMm = 0.0;
    double currentResidualMm = 0.0;
    FreshSurfaceComparisonStatus status = FreshSurfaceComparisonStatus::InvalidInput;
    int frozenPatchCount = 0;
    int currentModelSupportedPointCount = 0;
    double currentModelSupportedRatio = 0.0;
};

/**
 * @brief 在后台线程比较同帧新实测与修正前后同一冻结局部面，不重新求解位姿。
 * @details 局部面须有二维展开、平面性和有限近邻支持；只累计原位及候选下均在
 * coverageDistanceMm 邻域内的配对点，至少 minimumPointCount 点及覆盖比例通过。
 * 新实测云不能用于拟合面、调整候选或扩张支持范围；非法输入返回 valid=false。
 */
PairedSurfaceMetrics compareFreshSurface(const std::vector<Point3D_mm> &freshReal,
    const std::vector<Point3D_mm> &frozenVirtual, const RigidTransform &correction,
    const Options &options);

/**
 * @brief 仅在线后验：用当前重新求交的CAD切片证明冻结面的有限切向续展。
 * @details 法向仍只由冻结训练虚拟云拟合，真实新帧及当前切片不拟合面；
 * 当前虚拟点须满足严格法向一致，距训练近邻PCA切向矩形足迹的欧氏外延
 * 不得超过冻结预算；新实测仍须接近当前虚拟点，不放宽真实覆盖距离。
 * 同一冻结面用于修正前后误差，真实点不须同时落入旧面中心的覆盖球。
 * 这是有限局部平面的一致性检查，不能证明CAD材料身份或无限延长表面。
 */
PairedSurfaceMetrics compareFreshSurfaceWithCurrentReference(
    const std::vector<Point3D_mm> &freshReal,
    const std::vector<Point3D_mm> &currentVirtual,
    const std::vector<Point3D_mm> &frozenVirtual,
    const RigidTransform &correction, const Options &options);

/**
 * @brief 时序训练与留出点云及生产 ICP 已求得的虚拟到真实候选。
 * @details 调用方必须按不同完整帧分块，禁止将同帧密集点随机拆分冒充独立验证。
 * 本结构不携带焊点或仿真预测点，无法自行证明工件/表面身份。
 */
struct Request {
    std::vector<Point3D_mm> trainingReal_mm;
    std::vector<Point3D_mm> trainingVirtual_mm;
    std::vector<Point3D_mm> holdoutReal_mm;
    std::vector<Point3D_mm> holdoutVirtual_mm;
    RigidTransform candidateVirtualToReal;
    Options options;
};

/** @brief 弱模式冻结后的候选；accepted 仍须上层单步/累计预算与事务验收。 */
struct Result {
    bool accepted = false;
    /// @brief true 表示以更强训练收益补偿轻微覆盖不足，不能冒充普通验收。
    bool trainingCoverageRecoveryUsed = false;
    bool projected = false;
    bool hasObservableCorrection = false;
    int observableRank = 0;
    /// @brief 两云各自排序谱的逐项较小值，并非共同模式的对应谱；秩另列。
    std::array<double, 4> relativeModeStrengths{};
    RigidTransform observableVirtualToReal;
    Metrics trainingBefore;
    Metrics trainingAfter;
    Metrics holdoutBefore;
    Metrics holdoutAfter;
    QString message;
};

/**
 * @brief 仅计算有限两向残差和覆盖率，供后验新帧表面校核复用。
 * @details 无设备访问和可变共享状态；可在后台线程调用。非法/超预算输入返回无效。
 */
Metrics measure(const std::vector<Point3D_mm> &real,
    const std::vector<Point3D_mm> &reference, const Options &options);

/**
 * @brief 冻结两云共同弱模式，检查投影后训练收益与完整帧留出残差/覆盖。
 * @details 不重新拟合候选；不把最近邻对应当作材料对应。近共线、法向支持不足、
 * 非 XYZ+Rz 刚体候选或不合格数据均保守拒绝，不触发控制/场景副作用。
 */
Result evaluate(const Request &request);

} // namespace i2r_rlc::localization::surface_validation
#endif // I2R_RLC_LOCALIZATION_LOCALIZATION_SURFACE_VALIDATION_H
