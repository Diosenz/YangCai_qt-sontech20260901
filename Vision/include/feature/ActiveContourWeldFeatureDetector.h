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
 * @file        ActiveContourWeldFeatureDetector.h
 * @brief       主动轮廓焊缝特征检测接口
 * @details     提供 T 型、对接、搭接、gap、端点和仿真激光定位等焊缝语义检测能力。
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
#ifndef I2R_RLC_VISION_FEATURE_ACTIVE_CONTOUR_WELD_FEATURE_DETECTOR_H
#define I2R_RLC_VISION_FEATURE_ACTIVE_CONTOUR_WELD_FEATURE_DETECTOR_H

#include <deque>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "feature/WeldFeatureTypes.h"
#include "geometry/LineGeometryTools.h"
#include "stripe/ActiveContourStripeExtractor.h"

namespace i2r_rlc::vision {

/**
 * @brief 主动轮廓焊缝检测的目标焊缝模式。
 *
 * `Automatic` 会在 T 型、对接和搭接候选之间选择置信度最高的结果；
 * 显式枚举值则只运行对应焊缝类型。
 */
enum class ActiveContourSeamMode {
    /// @brief 自动比较 T 型、对接和搭接候选，选择最高置信度结果。
    Automatic = 0,
    /// @brief 对接焊缝模式。
    Butt = 1,
    /// @brief T 型焊缝模式。
    TJoint = 2,
    /// @brief 搭接焊缝模式。
    Lap = 3
};

/**
 * @brief 主动轮廓检测参数。
 *
 * 参数来自旧主动轮廓检测链路的经验值，并用显式字段替代旧全局变量。
 * 所有距离单位均为 pixel，灰度阈值范围为 0~255。
 */
struct ActiveContourDetectionOptions : public ActiveContourStripeOptions {
    /// @brief T 型、对接和搭接动态 ROI 首帧半宽，单位为 pixel。
    int firstFrameRoiHalfWidthPixel = 300;

    /// @brief T 型动态 ROI 左右半宽，单位为 pixel。
    int tJointRoiHalfWidthPixel = 160;

    /// @brief 对接动态 ROI 左右半宽，单位为 pixel。
    int buttRoiHalfWidthPixel = 150;

    /// @brief 搭接动态 ROI 左右半宽，单位为 pixel。
    int lapRoiHalfWidthPixel = 160;

    /// @brief T 型折线切分阈值，单位为 pixel。
    double tJointSplitDistancePixel = 5.0;

    /// @brief 对接折线切分阈值，单位为 pixel。
    double buttSplitDistancePixel = 6.0;

    /// @brief 搭接折线切分阈值，单位为 pixel。
    double lapSplitDistancePixel = 10.0;

    /// @brief RANSAC 迭代次数，单位为 iteration。
    int ransacIterations = 30;

    /// @brief T 型 RANSAC 点线距离阈值，单位为 pixel。
    double tJointRansacDistancePixel = 3.0;

    /// @brief 对接 RANSAC 点线距离阈值，单位为 pixel。
    double buttRansacDistancePixel = 4.0;

    /// @brief 搭接 RANSAC 点线距离阈值，单位为 pixel。
    double lapRansacDistancePixel = 6.0;

    /// @brief RANSAC 最小内点比例，范围为 0~1。
    double ransacMinInlierRatio = 0.80;

    /// @brief gap 扫描判定亮点的灰度阈值，范围为 0~255。
    int gapBrightThresholdGrayValue = 180;

    /// @brief 对接 gap 默认从端点向内偏移的距离，单位为 pixel。
    int buttGapEndpointMarginPixel = 50;

    /// @brief 对接 gap 扫描额外外扩距离，单位为 pixel。
    int buttGapScanPaddingPixel = 20;

    /// @brief T 型焊缝点允许相对上一帧跳变的最大 x 距离，单位为 pixel。
    double maxTJointJumpPixel = 100.0;

    /// @brief 进入稳定期后搭接点允许相对上一帧跳变的最大 x 距离，单位为 pixel。
    double maxLapJumpPixel = 30.0;

    /// @brief 搭接点启用跳变过滤的最小帧数，单位为 frame。
    int lapJumpFilterStartFrame = 60;

    /// @brief 多帧备用历史长度，单位为 frame。
    int historySize = 6;

    /// @brief 低于该质量时输出 invalid，范围为 0~1。
    double minConfidence = 0.10;
};

/**
 * @brief 主动轮廓检测跨帧状态。
 *
 * 调用方拥有本结构并按相机流生命周期复用。
 */
struct ActiveContourDetectionState : public ActiveContourStripeState {
    /// @brief 最近一次有效焊缝点，单位 pixel。
    WeldFeaturePoint lastWeldPoint;

    /// @brief 最近若干帧有效点，用于遮挡、反光或跳变时备用，坐标单位为 pixel。
    std::deque<WeldFeaturePoint> recentWeldPoints;

};

/**
 * @brief 主动轮廓焊缝检测结果。
 * @details
 * 结果坐标均为 pixel。
 */
struct ActiveContourDetectionResult {
    /// @brief true 表示得到可用于下游纠偏的焊缝点。
    bool valid = false;

    /// @brief 当前识别出的焊缝类型。
    WeldSeamType seamType = WeldSeamType::Unknown;

    /// @brief 主焊缝点，单位 pixel。
    WeldFeaturePoint weldPoint;

    /// @brief 对接 gap 左端点，单位 pixel。
    WeldFeaturePoint leftGapPoint;

    /// @brief 对接 gap 右端点，单位 pixel。
    WeldFeaturePoint rightGapPoint;

    /// @brief 多段拟合线交点或搭接候选特征点，坐标单位为 pixel。
    std::vector<WeldFeaturePoint> featurePoints;

    /// @brief 当前 ROI 曲线拟合出的线段，坐标单位为 pixel。
    std::vector<WeldLineSegment> fittedLines;

    /// @brief T 型分支输出的两侧直线参数，按 k1,b1,k2,b2 保存；k 无量纲，b 单位为 pixel。
    std::vector<double> slopeParameters;

    /// @brief 本帧主动轮廓曲线。
    ActiveContourCurve activeCurve;

    /// @brief 结果置信度，范围 0~1。
    double confidence = 0.0;

    /// @brief true 表示本帧使用了历史点备用。
    bool usedHistoryFallback = false;

    /// @brief 状态说明，不使用平台相关字符串类型；仅用于诊断。
    std::string message;
};

/**
 * @brief OpenCV-only 主动轮廓焊缝检测器。
 *
 * 本类消费 `CActiveContourStripeExtractor` 的曲线输出，只迁移旧主动轮廓
 * 后半段的 T/对接/搭接分支和多帧备用语义。
 * 本类不持有线程或设备资源，调用方负责在合适的 worker 线程中调用。
 */
class CActiveContourWeldFeatureDetector final {
public:
    /**
     * @brief 基于主动轮廓曲线检测焊缝点。
     *
     * @param gray8 输入灰度图，必须为 `CV_8UC1`。
     * @param mode 焊缝模式；Automatic 时选择最高置信度候选。
     * @param options 检测参数。
     * @param state 跨帧状态；必须由调用方持有并复用。
     * @return 主动轮廓焊缝检测结果。
     * @note `state` 按相机流生命周期复用，用于保存宽度、曲线和历史焊缝点。
     * @warning `state=nullptr`、输入图像无效或初始化/迭代失败时返回无效结果。
     */
    static ActiveContourDetectionResult detect(
        const cv::Mat &gray8,
        ActiveContourSeamMode mode,
        const ActiveContourDetectionOptions &options,
        ActiveContourDetectionState *state);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_FEATURE_ACTIVE_CONTOUR_WELD_FEATURE_DETECTOR_H

