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
 * @file        WeldVisionAlgorithmPipeline.cpp
 * @brief       焊缝视觉算法管线实现
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
#include <algorithm>
#include <chrono>
#include <cmath>

#include "pipeline/WeldVisionAlgorithmPipeline.h"

namespace i2r_rlc::vision {

namespace {

/**
 * @brief 将任意置信度值限制到 pipeline 统一使用的 0~1 范围。
 * @param value 输入置信度或比例值。
 * @return 裁剪后的比例值，范围为 0~1。
 */
double clampRatio(double value)
{
    return std::max(0.0, std::min(1.0, value));
}

/**
 * @brief 将主动轮廓模式映射为通用焊缝语义类型。
 * @param mode 主动轮廓检测模式。
 * @return pipeline 输出使用的焊缝类型；Automatic 映射为 Unknown。
 */
WeldSeamType seamTypeFromActiveContourMode(ActiveContourSeamMode mode)
{
    switch (mode)
    {
    case ActiveContourSeamMode::Butt:
        return WeldSeamType::Butt;
    case ActiveContourSeamMode::TJoint:
        return WeldSeamType::TJoint;
    case ActiveContourSeamMode::Lap:
        return WeldSeamType::Lap;
    case ActiveContourSeamMode::Automatic:
        return WeldSeamType::Unknown;
    }

    return WeldSeamType::Unknown;
}

/**
 * @brief 判断 pipeline 当前阶段是否支持输入灰度图。
 * @param gray8 输入图像。
 * @return true 表示图像非空且类型为 `CV_8UC1`。
 */
bool isSupportedGrayInput(const cv::Mat &gray8)
{
    return !gray8.empty() && gray8.type() == CV_8UC1;
}

/**
 * @brief 构造主动轮廓曲线端点对应的焊缝特征点。
 * @param point 曲线端点，单位 pixel。
 * @param label 输出标签，调用方保证生命周期覆盖赋值过程。
 * @param confidence 点置信度，输入会被裁剪到 0~1。
 * @return 可直接写入 pipeline 结果的有效特征点。
 */
WeldFeaturePoint makeCurveEndpoint(
    const cv::Point2d &point,
    const char *label,
    double confidence)
{
    WeldFeaturePoint endpoint;
    endpoint.xPixel = point.x;
    endpoint.yPixel = point.y;
    endpoint.confidence = clampRatio(confidence);
    endpoint.valid = true;
    endpoint.label = label;
    return endpoint;
}

/**
 * @brief 计算两个焊缝特征点的欧氏距离。
 * @param lhs 第一个特征点，坐标单位 pixel。
 * @param rhs 第二个特征点，坐标单位 pixel。
 * @return 两点距离，单位 pixel。
 */
double distancePixel(const WeldFeaturePoint &lhs, const WeldFeaturePoint &rhs)
{
    const double dx = lhs.xPixel - rhs.xPixel;
    const double dy = lhs.yPixel - rhs.yPixel;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief 构造普通中心线点对应的焊缝特征点。
 * @param point 中心线点，单位 pixel。
 * @param label 输出标签，调用方保证生命周期覆盖赋值过程。
 * @param confidence 点置信度，输入会被裁剪到 0~1。
 * @return 可用于 feature 分发器的有效特征点。
 */
WeldFeaturePoint makeCenterlinePoint(
    const cv::Point2d &point,
    const char *label,
    double confidence)
{
    WeldFeaturePoint featurePoint;
    featurePoint.xPixel = point.x;
    featurePoint.yPixel = point.y;
    featurePoint.confidence = clampRatio(confidence);
    featurePoint.valid = true;
    featurePoint.label = label;
    return featurePoint;
}

/**
 * @brief 将主动轮廓结果复制到 pipeline 对外结果。
 * @param activeResult 主动轮廓检测结果。
 * @param result 待填充的 pipeline 结果，必须非空。
 */
void fillFromActiveContour(
    const ActiveContourDetectionResult &activeResult,
    WeldVisionPipelineResult *result)
{
    result->activeContourResult = activeResult;
    result->seamType = activeResult.seamType;
    result->weldPoint = activeResult.weldPoint;
    result->confidence = clampRatio(activeResult.confidence);
    result->curveOrCenterlinePoints = activeResult.activeCurve.curvePoints;
}

/**
 * @brief 构造失败状态的 pipeline 结果。
 * @param task 原始任务配置。
 * @param status 失败状态码。
 * @param message 失败说明。
 * @return 带任务快照和状态说明的无效结果。
 */
WeldVisionPipelineResult makeInvalidResult(
    const WeldVisionTask &task,
    WeldVisionPipelineStatus status,
    const std::string &message)
{
    WeldVisionPipelineResult result;
    result.task = task;
    result.status = status;
    result.message = message;
    return result;
}

/**
 * @brief 将 stripe 中心线结果转换为 pipeline 中间结果。
 * @param task 原始任务配置。
 * @param stripeResult 条纹检测输出。
 * @param messagePrefix 状态说明前缀，用于区分不同 stripe 后端。
 * @return pipeline 结果；stripe 无效时保持失败状态。
 */
WeldVisionPipelineResult makeStripeResult(
    const WeldVisionTask &task,
    const StripeCenterlineResult &stripeResult,
    const char *messagePrefix)
{
    WeldVisionPipelineResult result;
    result.task = task;
    result.stripeResult = stripeResult;
    result.curveOrCenterlinePoints = stripeResult.centerlinePoints;
    result.confidence = clampRatio(stripeResult.qualityScore);
    if (!stripeResult.centerlinePoints.empty())
    {
        const std::size_t middleIndex = stripeResult.centerlinePoints.size() / 2U;
        result.weldPoint = makeCenterlinePoint(
            stripeResult.centerlinePoints[middleIndex],
            "stripe_centerline_point",
            result.confidence);
        result.startPoint = makeCenterlinePoint(
            stripeResult.centerlinePoints.front(),
            "stripe_start_point",
            result.confidence);
        result.endPoint = makeCenterlinePoint(
            stripeResult.centerlinePoints.back(),
            "stripe_end_point",
            result.confidence);
    }
    result.valid = stripeResult.valid;
    result.status = stripeResult.valid
        ? WeldVisionPipelineStatus::Succeeded
        : WeldVisionPipelineStatus::NoWeldFound;
    result.message = std::string(messagePrefix) + ": " + stripeResult.message;
    return result;
}

/**
 * @brief 执行 SAM/ISFFA/Steger 任一条纹候选检测分支。
 * @param input 当前帧输入，要求已经是 `CV_8UC1`。
 * @param options pipeline 选项，使用其中的 task 和 stripeOptions。
 * @param state pipeline 跨帧状态，保存各 stripe 后端的独立状态。
 * @return 条纹候选转换后的 pipeline 结果；不支持的 task 返回无效结果。
 */
WeldVisionPipelineResult processStripeCandidate(
    const WeldVisionFrameInput &input,
    const WeldVisionPipelineOptions &options,
    WeldVisionPipelineState *state)
{
    switch (options.task.kind)
    {
    case WeldVisionTaskKind::SamCandidate:
        return makeStripeResult(
            options.task,
            CSamStripeDetector::detect(
                input.gray8,
                options.stripeOptions,
                &state->samStripeState),
            "SAM candidate");
    case WeldVisionTaskKind::IsffaCandidate:
        return makeStripeResult(
            options.task,
            CIsffaStripeDetector::detect(
                input.gray8,
                options.stripeOptions,
                &state->isffaStripeState),
            "ISFFA candidate");
    case WeldVisionTaskKind::StegerCandidate:
        return makeStripeResult(
            options.task,
            CStegerStripeDetector::detect(
                input.gray8,
                options.stripeOptions,
                &state->stegerStripeState),
            "Steger candidate");
    default:
        return makeInvalidResult(
            options.task,
            WeldVisionPipelineStatus::UnsupportedTask,
            "not a stripe candidate task");
    }
}

/**
 * @brief 执行主动轮廓焊缝点检测分支。
 * @param input 当前帧输入，要求已经是 `CV_8UC1`。
 * @param options pipeline 选项，使用主动轮廓模式和参数。
 * @param state pipeline 跨帧状态，保存主动轮廓曲线和宽度估计。
 * @return 主动轮廓分支结果；失败时保留底层 message 便于诊断。
 */
WeldVisionPipelineResult processActiveContour(
    const WeldVisionFrameInput &input,
    const WeldVisionPipelineOptions &options,
    WeldVisionPipelineState *state)
{
    WeldVisionPipelineResult result;
    result.task = options.task;

    ActiveContourDetectionResult activeResult =
        CActiveContourWeldFeatureDetector::detect(
            input.gray8,
            options.task.activeContourMode,
            options.activeContourOptions,
            &state->activeContourState);

    fillFromActiveContour(activeResult, &result);
    if (activeResult.valid)
    {
        result.valid = true;
        result.status = WeldVisionPipelineStatus::Succeeded;
        result.message = "active contour weld point detected";
        return result;
    }

    result.valid = false;
    result.status = activeResult.confidence > 0.0
        ? WeldVisionPipelineStatus::LowConfidence
        : WeldVisionPipelineStatus::NoWeldFound;
    result.message = "active contour failed: " + activeResult.message;
    return result;
}

/**
 * @brief 执行焊缝起点/终点检测分支。
 * @param input 当前帧输入，要求已经是 `CV_8UC1`。
 * @param options pipeline 选项，包含起点方法和主动轮廓参数。
 * @param state pipeline 跨帧状态，保存起点检测器状态。
 * @return 起终点检测结果，并同步主动轮廓曲线和焊缝点候选。
 */
WeldVisionPipelineResult processEndpoint(
    const WeldVisionFrameInput &input,
    const WeldVisionPipelineOptions &options,
    WeldVisionPipelineState *state)
{
    WeldVisionPipelineResult result;
    result.task = options.task;

    WeldStartPointOptions startOptions = options.startPointOptions;
    startOptions.seamMode = options.task.activeContourMode;
    startOptions.method = options.task.endpointMethod == WeldVisionEndpointMethod::AreaChange
        ? WeldStartPointMethod::AreaChange
        : WeldStartPointMethod::CurveDistance;
    startOptions.activeContourOptions = options.activeContourOptions;

    const WeldStartPointResult startResult =
        CWeldStartPointDetector::detect(
            input.gray8,
            startOptions,
            &state->startPointState);
    result.startPointResult = startResult;
    result.activeContourResult = startResult.activeContourResult;
    result.seamType = startResult.seamType;
    result.weldPoint = startResult.weldPoint;
    result.startPoint = startResult.startPoint;
    result.endPoint = startResult.endPoint;
    result.confidence = clampRatio(startResult.confidence);
    result.curveOrCenterlinePoints =
        startResult.activeContourResult.activeCurve.curvePoints;
    result.valid = startResult.valid;
    result.status = result.valid
        ? WeldVisionPipelineStatus::Succeeded
        : WeldVisionPipelineStatus::NoWeldFound;
    result.message = startResult.message;
    return result;
}

} // namespace

/**
 * @brief 将旧 ImagePro 类型编号转换为 pipeline 任务描述。
 * @param type 旧工程任务编号。
 * @param seamMode 主动轮廓焊缝模式。
 * @param endpointMethodCode 旧起点方法编号，0 表示面积变化，其余表示曲线距离。
 * @return 规范化后的 pipeline 任务。
 */
WeldVisionTask WeldVisionTask::fromLegacyImageProType(
    int type,
    ActiveContourSeamMode seamMode,
    int endpointMethodCode)
{
    WeldVisionTask task;
    task.legacyImageProType = type;
    task.activeContourMode = seamMode;
    task.endpointMethod = endpointMethodCode == 0
        ? WeldVisionEndpointMethod::AreaChange
        : WeldVisionEndpointMethod::CurveDistance;

    switch (type)
    {
    case 0:
        task.kind = WeldVisionTaskKind::SamCandidate;
        break;
    case 1:
        task.kind = WeldVisionTaskKind::IsffaCandidate;
        break;
    case 2:
        task.kind = WeldVisionTaskKind::StegerCandidate;
        break;
    case 3:
        task.kind = WeldVisionTaskKind::ActiveContour;
        break;
    case 4:
        task.kind = WeldVisionTaskKind::Endpoint;
        break;
    default:
        task.kind = WeldVisionTaskKind::Unsupported;
        break;
    }

    return task;
}

/**
 * @brief 处理单帧焊缝视觉任务。
 * @param input 单帧图像输入，当前要求为 `CV_8UC1`。
 * @param options pipeline 参数和任务配置。
 * @param state 调用方持有的跨帧状态，必须非空。
 * @return 单帧 pipeline 输出，包含状态、耗时和分支结果快照。
 * @warning 本函数不做相机采集、文件读取或 UI 更新；调用方负责线程边界。
 */
WeldVisionPipelineResult CWeldVisionAlgorithmPipeline::process(
    const WeldVisionFrameInput &input,
    const WeldVisionPipelineOptions &options,
    WeldVisionPipelineState *state)
{
    const auto startedAt = std::chrono::steady_clock::now();
    auto finish = [startedAt](WeldVisionPipelineResult result) {
        const auto finishedAt = std::chrono::steady_clock::now();
        result.processingTimeMs =
            std::chrono::duration<double, std::milli>(finishedAt - startedAt).count();
        return result;
    };

    if (state == nullptr)
    {
        return finish(makeInvalidResult(
            options.task,
            WeldVisionPipelineStatus::InvalidInput,
            "pipeline state is null"));
    }

    if (!isSupportedGrayInput(input.gray8))
    {
        return finish(makeInvalidResult(
            options.task,
            WeldVisionPipelineStatus::InvalidInput,
            input.gray8.empty() ? "input image is empty" : "input image must be CV_8UC1"));
    }

    if (input.gray8.cols < options.minImageWidthPixel
        || input.gray8.rows < options.minImageHeightPixel)
    {
        return finish(makeInvalidResult(
            options.task,
            WeldVisionPipelineStatus::InvalidInput,
            "input image is smaller than pipeline minimum size"));
    }

    WeldVisionPipelineResult result;
    switch (options.task.kind)
    {
    case WeldVisionTaskKind::ActiveContour:
        result = processActiveContour(input, options, state);
        break;
    case WeldVisionTaskKind::Endpoint:
        result = processEndpoint(input, options, state);
        break;
    case WeldVisionTaskKind::SamCandidate:
    case WeldVisionTaskKind::IsffaCandidate:
    case WeldVisionTaskKind::StegerCandidate:
        result = processStripeCandidate(input, options, state);
        break;
    case WeldVisionTaskKind::Unsupported:
        result = makeInvalidResult(
            options.task,
            WeldVisionPipelineStatus::UnsupportedTask,
            "unknown legacy ImagePro task");
        break;
    }

    result.task = options.task;
    if (result.seamType == WeldSeamType::Unknown)
    {
        result.seamType = seamTypeFromActiveContourMode(options.task.activeContourMode);
    }
    ++state->processedFrameCount;
    return finish(result);
}

/**
 * @brief 将 pipeline 状态码转换为稳定英文 key。
 * @param status pipeline 状态码。
 * @return 适合日志、测试断言和 UI 状态映射的英文 key。
 */
std::string weldVisionPipelineStatusToKey(WeldVisionPipelineStatus status)
{
    switch (status)
    {
    case WeldVisionPipelineStatus::NotRun:
        return "NotRun";
    case WeldVisionPipelineStatus::Succeeded:
        return "Succeeded";
    case WeldVisionPipelineStatus::NoWeldFound:
        return "NoWeldFound";
    case WeldVisionPipelineStatus::LowConfidence:
        return "LowConfidence";
    case WeldVisionPipelineStatus::InvalidInput:
        return "InvalidInput";
    case WeldVisionPipelineStatus::UnsupportedTask:
        return "UnsupportedTask";
    case WeldVisionPipelineStatus::Error:
        return "Error";
    }

    return "Unknown";
}

} // namespace i2r_rlc::vision
