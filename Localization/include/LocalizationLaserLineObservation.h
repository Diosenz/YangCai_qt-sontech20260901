/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      LocalizationLaserLineObservation.h
 * @brief     定位模块自有的线结构光观测处理入口。
 * @details   本文件定义从仿真激光线先验到真实中心线、再到局部点云窗口的值对象。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 17 日
 */

#ifndef I2R_RLC_LOCALIZATION_LOCALIZATION_LASER_LINE_OBSERVATION_H
#define I2R_RLC_LOCALIZATION_LOCALIZATION_LASER_LINE_OBSERVATION_H

#include <deque>
#include <vector>

#include <QString>
#include <QtGlobal>

#ifdef I2R_RLC_ENABLE_OPENCV
#include <opencv2/core.hpp>
#endif

#include "LocalizationPointProjector.h"
#include "LocalizationTypes.h"

namespace i2r_rlc::localization {

/**
 * @brief 定位模块内部使用的二维激光像素点。
 */
struct LocalizationLaserImagePoint_px {
    double x_px = 0.0;
    double y_px = 0.0;
    qint64 frameId = -1;
    double confidence = 1.0;
};

/**
 * @brief DigitalTwin/RobotSim 给定位模块的虚拟激光先验。
 */
struct LocalizationLaserPrior {
    qint64 frameId = -1;
    std::vector<LocalizationLaserImagePoint_px> virtualLaserLine_px;
    bool hasVirtualWeldPoint = false;
    LocalizationLaserImagePoint_px virtualWeldPoint_px;
    int roiExpand_px = 250;
    double roiTopScale = 0.6;
    double roiBottomScale = 0.1;
};

/**
 * @brief 中心线提取参数。
 */
struct LocalizationCenterlineExtractionOptions {
    int intensityThreshold = 64;
    int gaussianKernelSize = 5;
    double gaussianSigma = 6.0;
    bool enableGabor = true;
    int gaborKernelSize = 11;
    double gaborSigma = 3.0;
    double gaborLambda = 9.0;
    double gaborPsi = 0.0;
    double gaborGamma = 1.0;
};

/**
 * @brief 真实中心线提取结果。
 */
struct LocalizationCenterlineExtractionResult {
    bool succeeded = false;
    qint64 frameId = -1;
    int roiX_px = 0;
    int roiY_px = 0;
    int roiWidth_px = 0;
    int roiHeight_px = 0;
    double quality = 0.0;
    std::vector<LocalizationLaserImagePoint_px> centerline_px;
    std::vector<LocalizationLaserImagePoint_px> roiVirtualLaserLine_px;
    QString failureReason;
};

/**
 * @brief 局部点云滑窗策略。
 */
struct LocalizationPointCloudWindowOptions {
    int realPointStride = 5;
    int virtualPointStride = 1;
    int slidingWindowFrameCount = 20;
    int maxPointCount = 0;
};

/**
 * @brief 单帧局部点云窗口追加请求。
 */
struct LocalizationPointCloudFrameRequest {
    qint64 frameId = -1;
    LocalizationCenterlineExtractionResult centerline;
    ImageProjectionRequest realProjection;
    std::vector<Point3D_mm> virtualFramePoints_mm;
    LocalizationPointCloudWindowOptions options;
};

/**
 * @brief 局部定位点云窗口快照。
 */
struct LocalizationPointCloudWindowSnapshot {
    bool available = false;
    QString message;
    int windowFrameCount = 0;
    std::vector<int> realFramePointCounts;
    std::vector<int> virtualFramePointCounts;
    std::vector<Point3D_mm> realWindowPoints_mm;
    std::vector<Point3D_mm> virtualWindowPoints_mm;
};

#ifdef I2R_RLC_ENABLE_OPENCV
/**
 * @brief 根据虚拟激光先验提取真实激光中心线。
 *
 * 该实现参考旧 `virtualSceneThreadFunc` 链路：由虚拟焊缝点/虚拟线构造 ROI，
 * 用虚拟线方向估计 Gabor 方向，然后执行 Hessian/Steger 风格中心线提取。
 */
class LocalizationLaserLineExtractor {
public:
    /**
     * @brief 从灰度或彩色图像中提取真实中心线。
     */
    static LocalizationCenterlineExtractionResult extractCenterline(
        const cv::Mat& image,
        const LocalizationLaserPrior& prior,
        const LocalizationCenterlineExtractionOptions& options);
};
#endif

/**
 * @brief 维护真实/虚拟局部点云滑窗。
 */
class LocalizationLaserPointCloudWindow {
public:
    /**
     * @brief 追加单帧中心线和虚拟点云，并按窗口上限丢弃旧帧。
     */
    LocalizationPointCloudWindowSnapshot appendFrame(
        const LocalizationPointCloudFrameRequest& request);

    /**
     * @brief 返回当前窗口快照。
     */
    LocalizationPointCloudWindowSnapshot snapshot() const;

    /**
     * @brief 清空滑窗状态。
     */
    void clear();

private:
    struct FrameEntry {
        qint64 frameId = -1;
        std::vector<Point3D_mm> realPoints_mm;
        std::vector<Point3D_mm> virtualPoints_mm;
    };

    std::deque<FrameEntry> m_frames;
};

} // namespace i2r_rlc::localization

#endif // I2R_RLC_LOCALIZATION_LOCALIZATION_LASER_LINE_OBSERVATION_H
