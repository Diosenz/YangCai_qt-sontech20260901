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
 * @file        RuntimeTrackingTypes.h
 * @brief       运行时跟踪线程数据类型
 * @details     定义 UI runtime worker 之间传递的跟踪请求、仿真回调和结果 DTO。
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
#ifndef I2R_RLC_UI_RUNTIME_TRACKING_TYPES_H
#define I2R_RLC_UI_RUNTIME_TRACKING_TYPES_H

#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <QImage>
#include <QMetaType>
#include <QString>

#include "CameraFrame.h"
#include "DigitalTwinVisionTypes.h"
#include "geometry/PositionerKinematics.h"
#include "geometry/WeldGeometryTypes.h"
#include "geometry/WeldProjectionContextBuilder.h"
#include "OnlineCorrectionCommand.h"
#include "OnlineCorrectionTrajectoryPlanner.h"
#include "OnlineCorrectionTypes.h"
#include "RobotState.h"
#include "TrackingTriggerPolicy.h"

namespace i2r_rlc::ui {

/**
 * @brief 跟踪算法模式。
 */
enum class RuntimeTrackingMethod {
    Kcf = 0,
    TwoTemplateMkcf = 1,
    ThreeTemplateMkcf = 2
};

/**
 * @brief 当前视觉跟踪结果的消费目的。
 *
 * 在线纠偏、有匹配离线纠偏和全局定位采点共用旧 det_type==2 / KCF 检测链，
 * 但状态机必须互斥消费，避免同一帧空间点同时喂给多个业务控制器。
 */
enum class RuntimeTrackingPurpose {
    None = 0,
    OnlineCorrection = 1,
    OfflineCorrection = 2,
    GlobalLocalization = 3
};

/**
 * @brief 正式纠偏会话的业务目的。
 *
 * 该枚举刻意排除全局定位和普通实时 RobotSim 展示。纠偏 session 只能属于
 * 在线或离线流程，避免 UI 或迟到结果把全局定位帧误认为纠偏输入。
 */
enum class CorrectionPurpose {
    None = 0,
    OnlineCorrection = 1,
    OfflineCorrection = 2
};

/**
 * @brief 一次正式在线或离线纠偏的运行时隔离边界。
 *
 * sessionId 由 RuntimeCoordinator 单调生成，0 只表示无活动会话。该值对象
 * 按值跨线程传递，不持有 worker、图像缓冲区或外部服务的生命周期。
 * 在线控制器内部暂停 Vision 时只改变 visionTrackingActive，不更换 sessionId。
 */
struct CorrectionRunSession {
    quint64 sessionId = 0;
    CorrectionPurpose purpose = CorrectionPurpose::None;
    bool active = false;
    bool visionTrackingActive = false;
    int consecutiveSyncFailures = 0;
    QString lastSyncError;
};

/**
 * @brief Vision 结果返回 RuntimeCoordinator 时携带的纠偏关联键。
 *
 * 三个字段必须同时匹配当前在途帧，结果才允许进入在线/离线控制器。
 * 普通跟踪和全局定位使用默认值，不参与纠偏会话匹配。
 */
struct RuntimeVisionCorrelation {
    quint64 sessionId = 0;
    CorrectionPurpose purpose = CorrectionPurpose::None;
    qint64 frameId = -1;
};

/**
 * @brief 纠偏逐帧 RobotSim 同步的结构化状态。
 */
enum class CorrectionSyncState {
    Idle = 0,
    Querying = 1,
    ObservationReady = 2,
    VisionConsumed = 3,
    Failed = 4,
    Cleared = 5,
    StoppedAfterFailures = 6,
    /// @brief 离线扫描窗口结束；停止 KCF 并恢复实时图像，会话保留至 ICP/O 完成。
    OfflineSeamWindowClosed = 7
};

/**
 * @brief RuntimeCoordinator 向 UI 发布的纠偏同步快照。
 *
 * observation 与会话关联键按值保存。UI 只有在收到同 session、purpose、
 * frameId 的 VisionConsumed 更新后，才能把 ObservationReady 标记为已消费。
 * Cleared/Failed/StoppedAfterFailures 必须清除 UI 中未确认的 observation。
 */
struct CorrectionSyncUpdate {
    CorrectionSyncState state = CorrectionSyncState::Idle;
    CorrectionPurpose purpose = CorrectionPurpose::None;
    quint64 sessionId = 0;
    qint64 frameId = -1;
    bool hasObservation = false;
    digital_twin::DigitalTwinVisionLaserObservation observation;
    int consecutiveSyncFailures = 0;
    QString message;
};

/**
 * @brief 判断 RobotSim observation 是否可供指定纠偏帧消费。
 *
 * 这是 transport 到 Vision 边界的最小不变量：观测必须有效且 frameId 完全
 * 相等。调用方不得在失败、空响应或错帧时回退到上一帧缓存。
 */
inline bool isUsableCorrectionObservationForFrame(
    qint64 frameId,
    const digital_twin::DigitalTwinVisionLaserObservation& observation)
{
    return observation.valid && observation.frameId == frameId;
}

/**
 * @brief 全局定位使用的仿真激光参考来源。
 */
enum class GlobalLocateVirtualLaserSource {
    Occt = 0,
    RobotSim = 1
};

/**
 * @brief 全局定位配准使用的虚拟点云来源。
 */
enum class GlobalLocateVirtualPointCloudSource {
    Weldment = 0,
    RobotSimLaserScan = 1
};

/**
 * @brief 一次全局定位跟踪会话使用的先验来源。
 *
 * 来源随会话固定，避免在线纠偏临时需求改写单独定位页面保存的默认配置。
 */
struct GlobalLocalizationTrackingSources {
    GlobalLocateVirtualLaserSource laserSource =
        GlobalLocateVirtualLaserSource::Occt;
    GlobalLocateVirtualPointCloudSource pointCloudSource =
        GlobalLocateVirtualPointCloudSource::Weldment;
};

/**
 * @brief 在线跟踪运行参数。
 */
struct RuntimeTrackingOptions {
    /// @brief 旧检测类型，第一版仅支持 2。
    int detType = 2;
    /// @brief 旧 testStart 标志。
    int testStart = 0;
    /// @brief 旧虚拟状态。
    control::TrackingVirtualStatus virtualStatus =
        control::TrackingVirtualStatus::OnlineRecord;
    /// @brief KCF/MKCF 模式。
    RuntimeTrackingMethod method = RuntimeTrackingMethod::Kcf;
    /// @brief 是否启用 gap 检测；第一版只透传给 OnlineCor 状态机。
    bool enableGapDetection = false;
    /// @brief true 表示使用全局定位专用“整条激光线提取”链路（Gabor+Steger），而非在线纠偏 KCF 单点。
    bool globalLocateLineExtraction = false;
    /// @brief SENSORPARA.txt 解析后的旧投影参数。
    vision::WeldSensorCalibration sensorCalibration;
    /// @brief 自动加载的 RBCALIB 变位机标定。
    vision::PositionerCalibration positionerCalibration;
};

/**
 * @brief 在线纠偏运行配置请求。
 */
struct RuntimeOnlineCorrectionConfiguration {
    control::OnlineCorrectionOptions options;
    control::OnlineCorrectionTrajectoryRequest trajectoryRequest;
    std::string rbcalibFile;
};

/**
 * @brief 保留旧 VirtualLaserSensor 方法名的仿真回调集合。
 */
struct SimulatedVisionCallbacks {
    /**
     * @brief 同帧 observation 进入 Vision 业务前的可选时序钩子。
     *
     * 生产默认空实现；专项测试用它稳定阻塞 Vision worker，以验证停止/重启
     * 时的迟到结果隔离。钩子不得改写 frame、RobotState 或 observation。
     */
    std::function<void(qint64)> beforeSynchronizedFrame = [](qint64) {};
    /// @brief 生成当前帧仿真激光线；默认返回 false。
    std::function<bool()> generate_simu_laser1 = []() { return false; };
    /// @brief 获取仿真焊缝点；返回 false 表示无有效点。
    std::function<bool(digital_twin::DigitalTwinImagePoint2D_px *)> getSeamPoint2d =
        [](digital_twin::DigitalTwinImagePoint2D_px *) { return false; };
    /// @brief 获取当前仿真焊缝段编号；默认 -1。
    std::function<int()> getSeamIndex = []() { return -1; };
    /// @brief 切换到下一条仿真焊缝；默认空实现。
    std::function<void()> toNextWeldSeam = []() {};
    /// @brief 获取当前仿真激光线点列；默认空点列。
    std::function<std::vector<digital_twin::DigitalTwinImagePoint2D_px>()>
        getCurrentVirtualLine =
            []() { return std::vector<digital_twin::DigitalTwinImagePoint2D_px>{}; };
    /// @brief 旧 sim_scan_line_change_point 等价空间点，单位 mm；默认无效。
    std::function<std::optional<digital_twin::DigitalTwinPoint3D_mm>()>
        getScanLineChangePoint =
            []() { return std::optional<digital_twin::DigitalTwinPoint3D_mm>{}; };
};

/**
 * @brief Runtime 为显示附加的同帧上下文，来自已匹配的在途帧。
 *
 * 原图与仿真观测按值传递。仅发给 UI 的结果携带此数据，业务采点不依赖显示。
 */
struct RuntimeTrackingFrameContext {
    camera::CameraFrame sourceFrame;
    digital_twin::DigitalTwinVisionLaserObservation observation;
};

/**
 * @brief 视觉跟踪线程单帧输出。
 */
struct RuntimeTrackingResult {
    /// @brief true 表示得到可投影的有效检测点。
    bool valid = false;
    /// @brief true 表示当前帧满足旧 OpencvThread 触发条件。
    bool triggerAccepted = false;
    /// @brief 帧号，单位 frame。
    qint64 frameId = -1;
    /// @brief 正式纠偏结果的会话/purpose/frame 关联；其他路径保持默认值。
    RuntimeVisionCorrelation correlation;
    /// @brief 纠偏显示必须具备的完整上下文；禁止缺失时改用 UI 最新相机帧。
    std::optional<RuntimeTrackingFrameContext> frameContext;
    /// @brief 焊缝图像点 X，单位 pixel。
    double imageXPixel = 0.0;
    /// @brief 焊缝图像点 Y，单位 pixel。
    double imageYPixel = 0.0;
    /// @brief 空间检测样本；无有效投影时 point.valid=false。
    control::OnlineCorrectionDetectionSample sample;
    /// @brief 整条真实激光线重建的空间点列（全局定位专用），单位 mm。
    /// 与在线纠偏的单点 sample 互斥：非空时 sample.point 仅作参考。
    std::vector<control::OnlineCorrectionPoint> linePoints;
    /// @brief Steger 提取的真实激光亚像素中心线（全局定位专用），单位 pixel。
    std::vector<digital_twin::DigitalTwinImagePoint2D_px>
        realLaserLineImagePoints_px;
    /// @brief 当前帧机器人 posflag（定位段标志），用于按段切分真实点云。
    int posflag = 0;
    /// @brief 当前帧采集瞬间的 traflag；1 表示位于 STARTDETPRG 检测窗口。
    int traflag = 0;
    /// @brief 当前帧采集瞬间的机器人 JBI 行号。
    int robotJobLine = 0;
    /// @brief 全局定位面序号；0 表示定位面外，正数为第几个 STARTPOSSEG。
    int localizationSegmentIndex = -1;
    /// @brief 当前 getSeamIndex() 结果，无仿真时为 -1。
    int simulatedSeamIndex = -1;
    /// @brief 旧 sim_scan_line_change_point 等价输入，单位 mm。
    std::optional<control::OnlineCorrectionPoint> scanLineChangePoint;
    /// @brief 算法效果图，可为空。
    QImage overlayImage;
    /// @brief 诊断说明。
    QString message;
};

} // namespace i2r_rlc::ui

Q_DECLARE_METATYPE(i2r_rlc::ui::CorrectionPurpose)
Q_DECLARE_METATYPE(i2r_rlc::ui::CorrectionRunSession)
Q_DECLARE_METATYPE(i2r_rlc::ui::RuntimeVisionCorrelation)
Q_DECLARE_METATYPE(i2r_rlc::ui::CorrectionSyncState)
Q_DECLARE_METATYPE(i2r_rlc::ui::CorrectionSyncUpdate)

#endif // I2R_RLC_UI_RUNTIME_TRACKING_TYPES_H
