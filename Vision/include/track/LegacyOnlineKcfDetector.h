/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        LegacyOnlineKcfDetector.h
 * @brief       旧在线 KCF 跟踪服务
 * @details     声明面向旧喷嘴跟踪流程的 KCF 在线检测封装。
 ******************************************************************************/
#ifndef I2R_RLC_VISION_TRACK_LEGACY_ONLINE_KCF_DETECTOR_H
#define I2R_RLC_VISION_TRACK_LEGACY_ONLINE_KCF_DETECTOR_H

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "DigitalTwinVisionTypes.h"
#include "feature/PointCloudGapWidthDetector.h"
#include "geometry/WeldGeometryTypes.h"
#include "track/KcfTracker.h"

namespace i2r_rlc::vision {

/**
 * @brief 旧在线 KCF 与 gap 检测参数。
 */
struct LegacyOnlineKcfOptions {
    KcfFusionOptions fusion;
    bool enableGapDetection = false;
    PointCloudGapWidthOptions gapWidthOptions;
};

/**
 * @brief 检测器阶段快照，随本帧结果按值传递，不拥有跟踪器状态。
 */
struct CLegacyOnlineKcfDiagnostic {
    bool attempted = false;
    bool templateReinitialized = false;
    bool gaborSucceeded = false;
    int framesSinceInitialize = 0;
    cv::Size imageSize;
    cv::Rect enhancementRoi;
    cv::Point2d templateAnchor;
    cv::Point2d simulatedPoint;
    cv::Point2d responsePoint;
    cv::Point2d beforeFusionPoint;
    cv::Point2d finalPoint;
    CKcfUpdateDiagnostic tracker;
};

/**
 * @brief 旧在线 KCF 单帧输出。
 */
struct LegacyOnlineKcfResult {
    bool valid = false;
    CLegacyOnlineKcfDiagnostic diagnostic;
    /// @brief 原始图像坐标，单位 pixel；由模板内焊缝锚点恢复，不等同于 ROI 中心。
    cv::Point2d weldPointPixel;
    WeldSpacePoint stationPoint;
    double gapWidthMm = 0.0;
    bool gapValid = false;
    double gapConfidence = 0.0;
    cv::Mat processedImage;
    cv::Rect trackedRoi;
    std::string message;
};

/**
 * @brief 旧 OpencvThread 活动路径中的标准 KCF/Gabor 在线检测器。
 *
 * 类内部持有 KCF 跨帧状态；runtime 与离线 replay 必须复用此实现。
 */
class CLegacyOnlineKcfDetector final {
public:
    CLegacyOnlineKcfDetector();

    void reset();

    LegacyOnlineKcfResult process(
        const cv::Mat &gray8,
        const digital_twin::DigitalTwinVisionLaserObservation &observation,
        const WeldProjectionContext &projectionContext,
        const LegacyOnlineKcfOptions &options);

private:
    std::unique_ptr<CKcfTracker> m_tracker;
    /// @brief 模板内焊缝位置除以模板宽高；重建模板时更新，跟踪期间保持不变。
    cv::Point2d m_templateWeldAnchor;
    /// @brief 模板所属原图尺寸；分辨率变化时清除旧坐标和滤波状态。
    cv::Size m_templateImageSize;
    bool m_initialized = false;
    int m_framesSinceInitialize = 0;
};

} // namespace i2r_rlc::vision

#endif
