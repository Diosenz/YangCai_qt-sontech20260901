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
 * @file        WeldVisionAlgorithmPipeline.h
 * @brief       焊缝视觉算法管线接口
 * @details     编排预处理、条纹提取和焊缝特征检测子模块，承接旧 ImagePro type 路由的现代入口。
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
#ifndef I2R_RLC_VISION_PIPELINE_WELD_VISION_ALGORITHM_PIPELINE_H
#define I2R_RLC_VISION_PIPELINE_WELD_VISION_ALGORITHM_PIPELINE_H

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "feature/ActiveContourWeldFeatureDetector.h"
#include "feature/WeldFeatureDetector.h"
#include "feature/WeldStartPointDetector.h"
#include "stripe/IsffaStripeDetector.h"
#include "stripe/SamStripeDetector.h"
#include "stripe/StegerStripeDetector.h"

namespace i2r_rlc::vision {

/**
 * @brief 焊接视觉任务类型。
 *
 * 枚举用于替代旧 `ImagePro` 中裸 `int type` 的生产入口。旧 type 只通过
 * `WeldVisionTask::fromLegacyImageProType` 显式映射到这些语义任务。
 */
enum class WeldVisionTaskKind {
    ActiveContour = 0,
    Endpoint = 1,
    SamCandidate = 2,
    IsffaCandidate = 3,
    StegerCandidate = 4,
    Unsupported = 5
};

/**
 * @brief 起点/终点检测策略。
 *
 * `AreaChange` 对应旧 `StartPtDet_Area` 的区域变化思想；`CurveDistance`
 * 对应旧 `StartPtDet` 中曲线极值和线交点距离判断思想。实际端点状态机由
 * `CWeldStartPointDetector` 承接，管线这里只保存兼容路由选择。
 */
enum class WeldVisionEndpointMethod {
    AreaChange = 0,
    CurveDistance = 1
};

/**
 * @brief 算法管线状态。
 */
enum class WeldVisionPipelineStatus {
    NotRun = 0,
    Succeeded = 1,
    NoWeldFound = 2,
    LowConfidence = 3,
    InvalidInput = 4,
    UnsupportedTask = 5,
    Error = 6
};

/**
 * @brief 单帧 OpenCV 算法输入。
 *
 * 输入图像必须是 `CV_8UC1` 灰度图，坐标单位为 pixel。UI 图像类型、相机
 * buffer 和文件读取都应在调用层完成，不进入本结构。
 */
struct WeldVisionFrameInput {
    /// @brief 输入灰度图，必须为 `CV_8UC1`。
    cv::Mat gray8;

    /// @brief 帧 ID，单位为 frame；无外部序号时可为 -1。
    std::int64_t frameId = -1;

    /// @brief 输入来源路径或诊断名称，不参与算法决策。
    std::string sourcePath;
};

/**
 * @brief 焊接视觉算法任务描述。
 *
 * 该结构是旧 `type`、显式焊缝模式和端点策略的现代承接层。生产代码应优先
 * 直接设置 `kind` 和语义字段；只有兼容旧流程时才调用旧 type 映射函数。
 */
struct WeldVisionTask {
    /// @brief 现代任务类型。
    WeldVisionTaskKind kind = WeldVisionTaskKind::ActiveContour;

    /// @brief 主动轮廓焊缝模式。
    ActiveContourSeamMode activeContourMode = ActiveContourSeamMode::Automatic;

    /// @brief 端点检测策略。
    WeldVisionEndpointMethod endpointMethod = WeldVisionEndpointMethod::AreaChange;

    /// @brief 旧 `ImagePro` type；非兼容入口可保持 -1。
    int legacyImageProType = 3;

    /**
     * @brief 将旧 `ImagePro::GetDetImagePt(type)` 路由映射为现代任务。
     *
     * @param type 旧 type：0=SAM、1=ISFFA、2=Steger、3=Snake、4=端点。
     * @param seamMode 显式焊缝模式；Automatic 表示自动测试 T/对接/搭接并取最优。
     * @param endpointMethodCode 旧端点策略：0=区域变化，其他=曲线距离。
     * @return 现代管线任务描述；未知 type 会映射为 Unsupported。
     */
    static WeldVisionTask fromLegacyImageProType(
        int type,
        ActiveContourSeamMode seamMode = ActiveContourSeamMode::Automatic,
        int endpointMethodCode = 0);
};

/**
 * @brief 算法管线参数。
 *
 * 本结构只保存各子模块参数和编排策略。所有跨帧状态放入
 * `WeldVisionPipelineState`，避免旧工程全局变量模式。
 */
struct WeldVisionPipelineOptions {
    /// @brief 当前帧执行的算法任务。
    WeldVisionTask task;

    /// @brief 主动轮廓参数。
    ActiveContourDetectionOptions activeContourOptions;

    /// @brief 传统 feature detector 参数，用于端点兼容路径的中心线/feature 派生。
    WeldFeatureDetectionOptions featureOptions;

    /// @brief SAM/ISFFA/Steger 条纹候选参数。
    StripeDetectionOptions stripeOptions;

    /// @brief 完整起点/终点检测参数。
    WeldStartPointOptions startPointOptions;

    /// @brief 算法管线要求的最小图像宽度，单位 pixel。
    int minImageWidthPixel = 16;

    /// @brief 算法管线要求的最小图像高度，单位 pixel。
    int minImageHeightPixel = 16;
};

/**
 * @brief 算法管线跨帧状态。
 *
 * 调用方按相机流生命周期持有本结构。主动轮廓、SAM、ISFFA、Steger 和
 * 端点检测的跨帧状态都在这里显式保存，替代旧 `ImagePro` 的全局变量。
 */
struct WeldVisionPipelineState {
    /// @brief 主动轮廓跨帧状态。
    ActiveContourDetectionState activeContourState;

    /// @brief SAM 条纹路径跨帧状态。
    StripeDetectionState samStripeState;

    /// @brief ISFFA 条纹路径跨帧状态。
    StripeDetectionState isffaStripeState;

    /// @brief Steger 条纹路径跨帧状态。
    StripeDetectionState stegerStripeState;

    /// @brief 起点/终点检测跨帧状态。
    WeldStartPointState startPointState;

    /// @brief 已处理帧数，单位为 frame。
    int processedFrameCount = 0;
};

/**
 * @brief 算法管线输出。
 *
 * 结果坐标单位均为 pixel。`message` 记录机器可读失败原因或阶段说明，供
 * replay 报告和 UI 诊断使用。
 */
struct WeldVisionPipelineResult {
    /// @brief 当前任务。
    WeldVisionTask task;

    /// @brief 管线状态。
    WeldVisionPipelineStatus status = WeldVisionPipelineStatus::NotRun;

    /// @brief true 表示得到可用于下游纠偏或检测流程的焊缝结果。
    bool valid = false;

    /// @brief 主焊缝点，单位 pixel。
    WeldFeaturePoint weldPoint;

    /// @brief 起点候选，单位 pixel。
    WeldFeaturePoint startPoint;

    /// @brief 终点候选，单位 pixel。
    WeldFeaturePoint endPoint;

    /// @brief 当前识别或请求的焊缝类型。
    WeldSeamType seamType = WeldSeamType::Unknown;

    /// @brief 主动轮廓检测结果。
    ActiveContourDetectionResult activeContourResult;

    /// @brief SAM/ISFFA/Steger 条纹候选结果。
    StripeCenterlineResult stripeResult;

    /// @brief 完整起点/终点检测结果。
    WeldStartPointResult startPointResult;

    /// @brief 传统 feature detector 结果；当前主要用于端点兼容路径。
    WeldFeatureResult featureResult;

    /// @brief 输出中心线或曲线点，单位 pixel。
    std::vector<cv::Point2d> curveOrCenterlinePoints;

    /// @brief 结果置信度，范围 0~1。
    double confidence = 0.0;

    /// @brief 处理耗时，单位 ms。
    double processingTimeMs = 0.0;

    /// @brief 状态说明。
    std::string message;
};

/**
 * @brief OpenCV-only 焊接视觉算法管线。
 *
 * 本类是旧 `ImagePro` 路由的现代承接层，只负责任务映射、状态持有和结果
 * 聚合。具体算法由 feature、stripe、geometry 等子模块完成。
 */
class CWeldVisionAlgorithmPipeline final {
public:
    /**
     * @brief 处理一帧灰度图。
     *
     * @param input 单帧输入，必须提供 `CV_8UC1` 灰度图。
     * @param options 任务和子模块参数。
     * @param state 跨帧状态；必须由调用方持有并复用。
     * @return 管线结果；失败时 `valid=false` 且 `status/message` 说明原因。
     */
    static WeldVisionPipelineResult process(
        const WeldVisionFrameInput &input,
        const WeldVisionPipelineOptions &options,
        WeldVisionPipelineState *state);
};

/**
 * @brief 将管线状态转换为英文 key。
 * @param status 管线状态枚举。
 * @return 稳定英文 key，用于日志、CSV 和测试断言。
 */
std::string weldVisionPipelineStatusToKey(WeldVisionPipelineStatus status);

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_PIPELINE_WELD_VISION_ALGORITHM_PIPELINE_H

