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
 * @file        OnlineCorrectionTypes.h
 * @brief       在线纠偏流程数据结构
 * @details     定义旧 OnlineCor change 主链路在新工程中的状态、输入事件和输出结果。
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
#ifndef I2R_RLC_CONTROL_ONLINE_CORRECTION_TYPES_H
#define I2R_RLC_CONTROL_ONLINE_CORRECTION_TYPES_H

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "geometry/PositionerKinematics.h"
#include "OnlineCorrectionCommand.h"
#include "RobotState.h"

namespace i2r_rlc::control {

/**
 * @brief 在线纠偏起点引导方式。
 *
 * 对应旧 `COnlineCor::correctStartPtType`，后续 UI 可直接映射为下拉选项。
 */
enum class OnlineCorrectionStartMode {
    /// @brief 用反向扫描确定 `_W` 进度，并生成局部修正后的粗起点。
    InitType,
    /// @brief 对示教轨迹应用全局刚性修正矩阵。
    GlobalCorrection,
    /// @brief 以当前机器人位姿作为 IBVS 起始点。
    IBVSStart
};

/**
 * @brief 在线纠偏状态机阶段。
 */
enum class OnlineCorrectionPhase {
    /// @brief 空闲，尚未开始纠偏。
    Idle,
    /// @brief 初始视觉检测正在运行。
    InitialDetectionRunning,
    /// @brief 等待机器人反向检测完成信号。
    WaitingForReverseCompletion,
    /// @brief 正在计算初始轨迹修正。
    PreparingInitialTrajectory,
    /// @brief 首批示教点正在按顺序发送。
    SendingInitialTeachPoints,
    /// @brief 等待控制柜以 offline==5 确认示教点更新完成。
    WaitingInitialTeachPointAck,
    /// @brief O 点已确认，等待机器人到达动态起点 P001。
    WaitingForInitialApproach,
    /// @brief 已收到 P001 后的首次 ENDDET，等待控制柜进入 STARTCOR。
    WaitingForOnlineStart,
    /// @brief 在线纠偏运行中。
    OnlineRunning,
    /// @brief 当前在线分段正在收尾。
    SegmentFinishing,
    /// @brief 结束 P 已确认，等待机器人 detflag 复位到 0。
    WaitingForDetectionReset,
    /// @brief 在线检测已结束，继续记录直到机器人主程序停止。
    WaitingForRobotProgramStop,
    /// @brief 全流程完成。
    Completed,
    /// @brief 流程出现错误。
    Error
};

/**
 * @brief 在线分段结束类型，对应旧 corflag 2/3。
 */
enum class OnlineCorrectionSegmentEndKind {
    Intermediate,
    Final
};

/**
 * @brief 旧 CorrectTaughtPathO 的控制方法枚举。
 *
 * 数值保持旧工程 `ControlMethod` 语义：0 为当前活动 PID 阈值分支，
 * 1 为模糊自适应 PID，2 为保留空控制分支。
 */
enum class OnlineCorrectionControlMethod {
    LegacyPidThreshold = 0,
    LegacyFuzzyAdaptivePid = 1,
    LegacyReservedNoop = 2
};

/**
 * @brief 空间点，单位 mm。
 */
struct OnlineCorrectionPoint {
    /// @brief X 坐标，单位为 mm。
    double xMm = 0.0;
    /// @brief Y 坐标，单位为 mm。
    double yMm = 0.0;
    /// @brief Z 坐标，单位为 mm。
    double zMm = 0.0;
    /// @brief 坐标是否有效。
    bool valid = false;
};

/**
 * @brief 初始反向扫描在当前 `_W` 分段上的匹配结果。
 *
 * 该值只描述轨迹进度和局部位置残差，不声明最终焊接起点的所有权。
 * `IBVSStart` 模式由视觉伺服结果接管最终起点时不会生成该匹配。
 */
struct OnlineCorrectionInitialPathMatch {
    /// @brief true 表示反向扫描已在当前 `_W` 分段内获得可信匹配。
    bool valid = false;
    /// @brief 匹配线段起点在完整 `_W` 轨迹中的零基索引。
    int matchedPathPointIndex = -1;
    /// @brief 匹配点在线段起点到终点之间的比例，范围为 [0, 1]。
    double segmentFraction = 0.0;
    /// @brief `_W` 上的连续投影点，单位 mm。
    OnlineCorrectionPoint referencePoint;
    /// @brief 检测终点减去 referencePoint 的局部位置残差，单位 mm。
    OnlineCorrectionPoint correctionOffset;
    /// @brief 检测终点到 `_W` 投影点的距离，单位 mm。
    double distanceMm = 0.0;
    /// @brief 反向扫描方向与 `_W` 正向切线反方向的余弦值。
    double reverseDirectionCosine = 0.0;
    /// @brief true 表示扫描跨度足够，已执行方向一致性校验。
    bool directionValidated = false;
};

/**
 * @brief 位姿，位置单位 mm，姿态单位 deg。
 */
struct OnlineCorrectionPose {
    /// @brief X 坐标，单位为 mm。
    double xMm = 0.0;
    /// @brief Y 坐标，单位为 mm。
    double yMm = 0.0;
    /// @brief Z 坐标，单位为 mm。
    double zMm = 0.0;
    /// @brief RX 姿态角，单位为 deg。
    double rxDeg = 0.0;
    /// @brief RY 姿态角，单位为 deg。
    double ryDeg = 0.0;
    /// @brief RZ 姿态角，单位为 deg。
    double rzDeg = 0.0;
    /// @brief 位姿是否有效。
    bool valid = false;
};

/**
 * @brief 变位机轴角，单位 deg。
 */
struct OnlineCorrectionStationAxis {
    /// @brief 变位机第一轴角度，单位为 deg。
    double s1Deg = 0.0;
    /// @brief 变位机第二轴角度，单位为 deg。
    double s2Deg = 0.0;
    /// @brief 轴角是否有效。
    bool valid = false;
};

/**
 * @brief 单个在线检测样本。
 *
 * 点与间隙宽度放在同一结构中，保证启用间隙检测时两者按样本一一对应。
 */
struct OnlineCorrectionDetectionSample {
    /// @brief 检测事件输入为变位机 TCP 点，mm；在线内部缓存转换为末节点。
    OnlineCorrectionPoint point;
    /// @brief 可选 gap 宽度，单位为 mm，仅 enableGapDetection 时强制要求。
    std::optional<float> gapWidthMm;
    /// @brief 仿真扫描焊缝编号，来自旧 getSeamIndex()；无仿真时为 -1。
    int simulatedSeamIndex = -1;
    /// @brief 旧 sim_scan_line_change_point 等价输入，单位 mm；无效时不触发曲线切换。
    std::optional<OnlineCorrectionPoint> scanLineChangePoint;
    /// @brief true 为粗跟踪/仿真续接估计；可供在线跟随，不进入初始扫描或 ICP 拟合。
    bool estimated = false;
};

/**
 * @brief 一帧或一批视觉检测事件。
 *
 * 该事件承接旧 `D_initdata` 的填充语义，调用方负责把视觉点重建到空间坐标。
 */
struct OnlineCorrectionDetectionEvent {
    /// @brief 当前批次视觉样本，调用方按检测顺序填充。
    std::vector<OnlineCorrectionDetectionSample> samples;
    /// @brief true 表示初始检测阶段已经结束。
    bool detectionFinished = false;
    /// @brief 事件说明或诊断消息。
    std::string message;
};

/**
 * @brief 在线纠偏配置。
 */
struct OnlineCorrectionOptions {
    /// @brief 在线纠偏起点来源与初始化策略，兼容旧 correctStartPtType。
    OnlineCorrectionStartMode startMode = OnlineCorrectionStartMode::InitType;
    /// @brief true 表示初始检测样本必须携带 gap 宽度。
    bool enableGapDetection = false;
    /// @brief true 使用在线轨迹 ICP；false 直接跟随检测点，不影响起点/全局配准。
    bool enableOnlineIcp = true;
    /// @brief 正式在线显式启用；保留 ICP 候选用于诊断，实际释放原始实测点。
    bool preserveMeasuredReference = false;
    /// @brief 机器人 TCP 速度，单位 mm/s，用于保持旧纠偏周期语义。
    double stepTcpVelocityMmPerSecond = 11.0;
    /// @brief 控制周期，单位 s，旧活动路径使用 0.0043。
    double controlCycleSecond = 0.0043;
    /// @brief 检测点越过判定距离，单位 mm。
    double passedPointDistanceMm = 0.8;
    /// @brief CorrectTaughtPathO 控制方法，默认保持旧活动分支 ControlMethod==0。
    OnlineCorrectionControlMethod controlMethod =
        OnlineCorrectionControlMethod::LegacyPidThreshold;
    /// @brief 实时增量绝对值限制，单位 um。
    long maxCorrectionIncrementUm = 80;
    /// @brief 相邻实时增量允许跳变，单位 um。
    long maxCorrectionJumpUm = 60;
    /// @brief 初始检测终点到当前 `_W` 分段的最大允许距离，单位 mm。
    double initialPathMatchMaxDistanceMm = 15.0;
    /// @brief 用于估计反向扫描方向的末尾样本数量。
    int initialPathMatchDirectionSampleCount = 32;
    /// @brief 启用方向校验所需的最小扫描跨度，单位 mm。
    double initialPathMatchMinScanSpanMm = 2.0;
    /// @brief 反扫方向与 `_W` 反向切线之间允许的最小余弦值。
    double initialPathMatchMinReverseDirectionCosine = 0.25;
    /// @brief 匹配点之后必须保留的最短在线纠偏路径，单位 mm。
    double initialPathMatchMinRemainingLengthMm = 10.0;
    /// @brief 行主序 4x4 全局修正矩阵，长度固定为 16，坐标单位为 mm。
    std::array<double, 16> globalCorrectionRowMajor = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    };
};

/**
 * @brief 单次释放批的诊断值快照，仅随事件传出，不留在会话状态中。
 * @details 坐标均为变位机末节系、mm。矩阵按行展开，将检测映射到参考模型；
 *          采用时算法取其逆变换模型。候选矩阵存在不代表已用于目标轨迹。
 */
struct OnlineCorrectionIcpBatchDiagnostic {
    bool enabled = true;
    bool attempted = false;
    bool applied = false;
    bool flush = false;
    std::optional<std::array<double, 16>> detectionToModel;
    std::optional<double> rmsErrorMm;
    /// @brief 实际送入求解器的降采样检测窗口，包含历史。
    std::vector<OnlineCorrectionPoint> fitPoints;
    /// @brief 本次求解所用参考模型，已包含全局与起点修正。
    std::vector<OnlineCorrectionPoint> modelPoints;
    /// @brief 尚未释放的原始新点及实际释放给跟随器的点列。
    std::vector<OnlineCorrectionDetectionSample> pendingSamples;
    std::vector<OnlineCorrectionDetectionSample> releasedSamples;
    /// @brief 实测优先策略下未采用的模型输出，仅诊断，不能解释为已释放轨迹。
    std::vector<OnlineCorrectionDetectionSample> modelProposalSamples;
    std::string message;
};

/** @brief 跟随器实际使用的目标直线及垂足，末节系、mm；仅生成普通 P 时有效。 */
struct OnlineCorrectionFollowDiagnostic {
    bool valid = false;
    OnlineCorrectionPoint current;
    OnlineCorrectionPoint first;
    OnlineCorrectionPoint directionPoint;
    OnlineCorrectionPoint foot;
    OnlineCorrectionPoint futureFoot;
    double s1Deg = 0.0;
    double s2Deg = 0.0;
};

/** @brief 控制目标来源；预测只供实时跟随，不代表新增视觉测量。 */
enum class OnlineCorrectionTargetSource {
    Unavailable,
    Measured,
    PredictedReference
};

/**
 * @brief 控制线程持有的唯一目标快照，位置为变位机末节系、mm。
 * @details targetPointInLastLink_mm 等于实际 P 几何中的 futureFoot；不能当作
 *          原图独立点或输入异常 ICP。UI 若需父级 TCP 点应扣除 toolTcp 偏置。
 */
struct OnlineCorrectionTargetSnapshot {
    bool valid = false;
    OnlineCorrectionTargetSource source = OnlineCorrectionTargetSource::Unavailable;
    /// @brief Runtime worker 注入当前纠偏会话，仅供 UI 隔离；Controller 不生成该 ID。
    std::uint64_t sessionId = 0;
    /// @brief 同会话目标计算序号；reset 后从零重新建立。
    std::uint64_t generation = 0;
    OnlineCorrectionPoint targetPointInLastLink_mm;
    OnlineCorrectionFollowDiagnostic geometry;
    int taughtPointIndex = -1;
};
/** @brief 最近一次在线轨迹 ICP 诊断；只在控制线程更新。 */
struct OnlineCorrectionIcpStatus {
    bool enabled = false;
    bool attempted = false;
    bool applied = false;
    /// @brief 尚未输出的原始检测点数，不包含只参与拟合的历史。
    std::size_t bufferedSampleCount = 0;
    /// @brief 检测点到参考轨迹的 RMS，单位 mm，仅 applied 时有效。
    double rmsErrorMm = 0.0;
    std::string message;
};

/**
 * @brief 在线纠偏控制器持有的会话状态。
 */
struct OnlineCorrectionSessionState {
    /// @brief 当前状态机阶段。
    OnlineCorrectionPhase phase = OnlineCorrectionPhase::Idle;
    /// @brief 当前起点模式。
    OnlineCorrectionStartMode startMode = OnlineCorrectionStartMode::InitType;
    /// @brief 当前轨迹分段索引。
    int segmentIndex = 0;
    /// @brief 当前分段起始示教点索引。
    int startTeachIndex = 0;
    /// @brief 当前分段结束示教点索引。
    int endTeachIndex = 0;
    /// @brief true 表示视觉跟踪/检测应保持运行。
    bool trackerActive = false;
    /// @brief 区分在线 ICP 等待、实际求解、失败回退和关闭状态。
    OnlineCorrectionIcpStatus onlineIcp;
    /// @brief true 表示初始检测已经结束。
    bool initialDetectionFinished = false;
    /**
     * @brief true 表示已观察到 onlineReverseCompleteFlag==0，可接受下一次上升为 1。
     *
     * 防止上一轮遗留的完成电平在新一轮初始检测启动后被误判为新事件。
     */
    bool reverseCompletionArmed = false;
    /// @brief true 表示示教轨迹已成功加载并分段。
    bool onlineTeachTrajectoryInitialized = false;
    /// @brief RBCALIB 变位机标定是否有效。
    bool positionerCalibrationInitialized = false;
    /// @brief 初始检测样本缓存，变位机 TCP 系、mm；规划器负责转换一次。
    std::vector<OnlineCorrectionDetectionSample> initialDetectionSamples;
    /// @brief 在线运行阶段尚未消费的样本，变位机末节系、mm。
    std::vector<OnlineCorrectionDetectionSample> onlineDetectionSamples;
    /// @brief 初始修正后的当前分段轨迹，单位 mm。
    std::vector<OnlineCorrectionPoint> correctedSegmentPath;
    /// @brief 本轮反向扫描在 `_W` 上的进度匹配与局部残差。
    OnlineCorrectionInitialPathMatch initialPathMatch;
    /// @brief 已发送的首批示教点数量。
    int teachPointSendIndex = 0;
    /// @brief 首批示教点总数，用于只接受最终 O 命令的 offline==5 确认。
    int initialTeachPointCount = 0;
    /// @brief 当前控制目标的唯一权威快照，与检测/ICP 样本身份分离。
    OnlineCorrectionTargetSnapshot controlTarget;
    /// @brief 最近一次 getSeamIndex()。
    int simulatedSeamIndex = -1;
    /// @brief 当前锁存的分段结束类型。
    std::optional<OnlineCorrectionSegmentEndKind> pendingSegmentEndKind;
    /// @brief true 表示当前 corflag/detflag 结束边沿已经处理。
    bool segmentEndLatched = false;
    /// @brief 最近一次状态说明或错误消息。
    std::string message;
};

/**
 * @brief 在线纠偏控制器单次状态更新结果。
 */
struct OnlineCorrectionUpdate {
    /// @brief 当前事件处理是否成功。
    bool succeeded = true;
    /// @brief 此次含新 ICP 诊断，记录器不应重复记录普通状态快照。
    bool onlineIcpUpdated = false;
    /// @brief 本事件释放的各个批次，保留同次 seam 切换产生的多个结果。
    std::vector<OnlineCorrectionIcpBatchDiagnostic> onlineIcpBatches;
    /// @brief 实际参与本次 P 计算的目标几何，区别于刚释放而尚未消费的批次。
    OnlineCorrectionFollowDiagnostic followDiagnostic;
    /// @brief 事件处理后的状态机阶段。
    OnlineCorrectionPhase phase = OnlineCorrectionPhase::Idle;
    /// @brief 事件处理后的完整会话快照。
    OnlineCorrectionSessionState state;
    /// @brief 可选机器人纠偏命令，仅 valid 为 true 时发送。
    robot_comm::RobotCorrectionCommand command;
    /// @brief 可选实时 P 命令，仅 valid 时发送。
    robot_comm::RobotRealtimeCorrectionCommand realtimeCommand;
    /// @brief 本次实时命令使用的 `_W` 示教点索引，供运行诊断和回归测试使用。
    int taughtPointIndex = -1;
    /// @brief true 表示 runtime 应启动 Vision tracking。
    bool startTracking = false;
    /// @brief true 表示 runtime 应停止 Vision tracking。
    bool stopTracking = false;
    /// @brief true 表示 runtime 应加载下一分段并自动重新开始初始检测。
    bool requestNextSegment = false;
    /// @brief 事件处理说明或错误消息。
    std::string message;
};

} // namespace i2r_rlc::control

#endif // I2R_RLC_CONTROL_ONLINE_CORRECTION_TYPES_H
