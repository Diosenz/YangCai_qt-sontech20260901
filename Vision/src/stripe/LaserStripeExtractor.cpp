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
 * @file        LaserStripeExtractor.cpp
 * @brief       激光条纹提取实现
 * @details     提供激光条纹、骨架、中心线和主动轮廓曲线提取能力，不承担焊缝语义分类。
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
#include <cmath>
#include <limits>

#include <opencv2/imgproc.hpp>

#include "stripe/LaserStripeExtractor.h"

namespace i2r_rlc::vision {

namespace {

/**
 * @brief 写入预处理状态，允许调用方传入空指针表示忽略状态。
 * @param status 可选输出状态指针，生命周期由调用方持有。
 * @param value 本次条纹提取阶段的状态码。
 */
void setStatus(VisionPreprocessStatus *status, VisionPreprocessStatus value)
{
    if (status != nullptr)
    {
        *status = value;
    }
}

/**
 * @brief 判断输入是否为非空 8 位单通道灰度图。
 * @param grayImage 待检查图像，像素单位为 gray。
 * @return 满足 CLaserStripeExtractor 输入约束时返回 true。
 */
bool isValidGrayImage(const cv::Mat &grayImage)
{
    return !grayImage.empty() && grayImage.type() == CV_8UC1;
}

/**
 * @brief 校验阈值提取参数的基本范围。
 * @param options 阈值、相对比例、列峰数量和 ROI 扩展配置。
 * @return 参数不会导致越界或无意义扫描时返回 true。
 */
bool isValidOptions(const LaserStripeThresholdOptions &options)
{
    return options.thresholdGrayValue >= 0
        && options.thresholdGrayValue <= 255
        && options.relativeThresholdRatio >= 0.0
        && options.relativeThresholdRatio <= 1.0
        && options.minForegroundPixelsPerColumn > 0
        && options.roiPaddingPixel >= 0
        && options.maxPeaksPerColumn > 0;
}

/**
 * @brief 根据固定阈值和相对最大灰度阈值计算最终二值化阈值。
 * @param grayImage 输入灰度图，必须为 CV_8UC1。
 * @param options 阈值策略配置。
 * @return 夹紧到 [0,255] 的有效阈值。
 */
int calculateEffectiveThreshold(const cv::Mat &grayImage,
                                const LaserStripeThresholdOptions &options)
{
    double minValue = 0.0;
    double maxValue = 0.0;
    cv::minMaxLoc(grayImage, &minValue, &maxValue);

    int thresholdValue = options.thresholdGrayValue;
    if (options.useRelativeThreshold)
    {
        const int relativeThreshold = static_cast<int>(
            std::floor(maxValue * options.relativeThresholdRatio));
        thresholdValue = std::max(thresholdValue, relativeThreshold);
    }

    return std::max(0, std::min(255, thresholdValue));
}

/**
 * @brief 构造单列局部峰值记录。
 * @param xPixel 峰值所在列坐标，单位为 pixel。
 * @param yPixel 峰值所在行坐标，单位为 pixel。
 * @param grayValue 峰值灰度，范围为 [0,255]。
 * @return 带归一化得分的列峰描述。
 */
LaserStripeColumnPeak makePeak(int xPixel, int yPixel, int grayValue)
{
    LaserStripeColumnPeak peak;
    peak.xPixel = xPixel;
    peak.yPixel = yPixel;
    peak.grayValue = grayValue;
    peak.scoreRatio = static_cast<double>(grayValue) / 255.0;
    return peak;
}

} // namespace

cv::Mat CLaserStripeExtractor::createMask(
    const cv::Mat &grayImage,
    const LaserStripeThresholdOptions &options,
    VisionPreprocessStatus *status)
{
    if (grayImage.empty())
    {
        setStatus(status, VisionPreprocessStatus::EmptyInput);
        return cv::Mat();
    }

    if (grayImage.type() != CV_8UC1)
    {
        setStatus(status, VisionPreprocessStatus::InvalidGrayImage);
        return cv::Mat();
    }

    if (!isValidOptions(options))
    {
        setStatus(status, VisionPreprocessStatus::InvalidParameter);
        return cv::Mat();
    }

    const int thresholdValue = calculateEffectiveThreshold(grayImage, options);

    cv::Mat mask;
    cv::threshold(grayImage, mask, thresholdValue, 255, cv::THRESH_BINARY);

    setStatus(status, VisionPreprocessStatus::Succeeded);
    return mask;
}

std::vector<LaserStripeColumnPeak> CLaserStripeExtractor::extractColumnPeaks(
    const cv::Mat &grayImage,
    int xPixel,
    const LaserStripeThresholdOptions &options,
    VisionPreprocessStatus *status)
{
    std::vector<LaserStripeColumnPeak> peaks;

    if (grayImage.empty())
    {
        setStatus(status, VisionPreprocessStatus::EmptyInput);
        return peaks;
    }

    if (grayImage.type() != CV_8UC1)
    {
        setStatus(status, VisionPreprocessStatus::InvalidGrayImage);
        return peaks;
    }

    if (!isValidOptions(options) || xPixel < 0 || xPixel >= grayImage.cols)
    {
        setStatus(status, VisionPreprocessStatus::InvalidParameter);
        return peaks;
    }

    if (grayImage.rows < 3)
    {
        setStatus(status, VisionPreprocessStatus::Succeeded);
        return peaks;
    }

    const int thresholdValue = calculateEffectiveThreshold(grayImage, options);
    for (int yPixel = 1; yPixel < grayImage.rows - 1; ++yPixel)
    {
        const int previousGray = grayImage.at<unsigned char>(yPixel - 1, xPixel);
        const int currentGray = grayImage.at<unsigned char>(yPixel, xPixel);
        const int nextGray = grayImage.at<unsigned char>(yPixel + 1, xPixel);

        if (currentGray < thresholdValue)
        {
            continue;
        }

        if (currentGray >= previousGray && currentGray >= nextGray)
        {
            peaks.push_back(makePeak(xPixel, yPixel, currentGray));
        }
    }

    std::sort(peaks.begin(),
              peaks.end(),
              [](const LaserStripeColumnPeak &left,
                 const LaserStripeColumnPeak &right) {
                  return left.grayValue > right.grayValue;
              });

    if (static_cast<int>(peaks.size()) > options.maxPeaksPerColumn)
    {
        peaks.resize(options.maxPeaksPerColumn);
    }

    setStatus(status, VisionPreprocessStatus::Succeeded);
    return peaks;
}

std::vector<LaserStripeColumnProfile>
CLaserStripeExtractor::calculateColumnProfiles(
    const cv::Mat &grayImage,
    const cv::Mat &mask,
    const LaserStripeThresholdOptions &options,
    VisionPreprocessStatus *status)
{
    std::vector<LaserStripeColumnProfile> profiles;

    if (grayImage.empty() || mask.empty())
    {
        setStatus(status, VisionPreprocessStatus::EmptyInput);
        return profiles;
    }

    if (!isValidGrayImage(grayImage)
        || mask.type() != CV_8UC1
        || mask.size() != grayImage.size())
    {
        setStatus(status, VisionPreprocessStatus::InvalidGrayImage);
        return profiles;
    }

    if (!isValidOptions(options))
    {
        setStatus(status, VisionPreprocessStatus::InvalidParameter);
        return profiles;
    }

    profiles.reserve(static_cast<std::size_t>(grayImage.cols));
    for (int xPixel = 0; xPixel < grayImage.cols; ++xPixel)
    {
        LaserStripeColumnProfile profile;
        profile.xPixel = xPixel;

        for (int yPixel = 0; yPixel < grayImage.rows; ++yPixel)
        {
            if (mask.at<unsigned char>(yPixel, xPixel) == 0)
            {
                continue;
            }

            if (profile.foregroundPixelCount == 0)
            {
                profile.firstYPixel = yPixel;
            }

            profile.lastYPixel = yPixel;
            ++profile.foregroundPixelCount;
        }

        profile.hasStripe = profile.foregroundPixelCount
            >= options.minForegroundPixelsPerColumn;
        if (profile.hasStripe)
        {
            profile.stripeWidthPixel =
                profile.lastYPixel - profile.firstYPixel + 1;
            profile.foregroundRatio =
                static_cast<double>(profile.foregroundPixelCount)
                / static_cast<double>(grayImage.rows);
            profile.peakCandidates = extractColumnPeaks(
                grayImage,
                xPixel,
                options,
                nullptr);
        }

        profiles.push_back(profile);
    }

    setStatus(status, VisionPreprocessStatus::Succeeded);
    return profiles;
}

LaserStripeRoiEstimate CLaserStripeExtractor::estimateRoi(
    const std::vector<LaserStripeColumnProfile> &columnProfiles,
    int imageWidthPixel,
    int imageHeightPixel,
    const LaserStripeThresholdOptions &options)
{
    LaserStripeRoiEstimate estimate;

    if (columnProfiles.empty()
        || imageWidthPixel <= 0
        || imageHeightPixel <= 0
        || !isValidOptions(options))
    {
        return estimate;
    }

    int minXPixel = std::numeric_limits<int>::max();
    int maxXPixel = std::numeric_limits<int>::min();
    int minYPixel = std::numeric_limits<int>::max();
    int maxYPixel = std::numeric_limits<int>::min();
    int widthSumPixel = 0;
    int minWidthPixel = std::numeric_limits<int>::max();
    int maxWidthPixel = 0;

    for (const LaserStripeColumnProfile &profile : columnProfiles)
    {
        if (!profile.hasStripe)
        {
            continue;
        }

        ++estimate.validColumnCount;
        widthSumPixel += profile.stripeWidthPixel;
        minWidthPixel = std::min(minWidthPixel, profile.stripeWidthPixel);
        maxWidthPixel = std::max(maxWidthPixel, profile.stripeWidthPixel);
        minXPixel = std::min(minXPixel, profile.xPixel);
        maxXPixel = std::max(maxXPixel, profile.xPixel);
        minYPixel = std::min(minYPixel, profile.firstYPixel);
        maxYPixel = std::max(maxYPixel, profile.lastYPixel);
    }

    if (estimate.validColumnCount == 0)
    {
        return estimate;
    }

    const int paddingPixel = options.roiPaddingPixel;
    minXPixel = std::max(0, minXPixel - paddingPixel);
    minYPixel = std::max(0, minYPixel - paddingPixel);
    maxXPixel = std::min(imageWidthPixel - 1, maxXPixel + paddingPixel);
    maxYPixel = std::min(imageHeightPixel - 1, maxYPixel + paddingPixel);

    estimate.roi.xPixel = minXPixel;
    estimate.roi.yPixel = minYPixel;
    estimate.roi.widthPixel = maxXPixel - minXPixel + 1;
    estimate.roi.heightPixel = maxYPixel - minYPixel + 1;
    estimate.validColumnRatio =
        static_cast<double>(estimate.validColumnCount)
        / static_cast<double>(imageWidthPixel);
    estimate.meanStripeWidthPixel =
        static_cast<double>(widthSumPixel)
        / static_cast<double>(estimate.validColumnCount);
    estimate.minStripeWidthPixel = minWidthPixel;
    estimate.maxStripeWidthPixel = maxWidthPixel;
    estimate.confidenceRatio = std::max(
        0.0,
        std::min(1.0, estimate.validColumnRatio));

    return estimate;
}

LaserStripeExtractionResult CLaserStripeExtractor::extract(
    const cv::Mat &grayImage,
    const LaserStripeThresholdOptions &options,
    VisionPreprocessStatus *status)
{
    LaserStripeExtractionResult result;

    result.mask = createMask(grayImage, options, status);
    if (result.mask.empty())
    {
        return result;
    }

    result.foregroundPixelCount = cv::countNonZero(result.mask);
    result.foregroundRatio = static_cast<double>(result.foregroundPixelCount)
        / static_cast<double>(result.mask.rows * result.mask.cols);

    VisionPreprocessStatus profileStatus = VisionPreprocessStatus::Succeeded;
    result.columnProfiles = calculateColumnProfiles(
        grayImage,
        result.mask,
        options,
        &profileStatus);
    if (profileStatus != VisionPreprocessStatus::Succeeded)
    {
        setStatus(status, profileStatus);
        result = LaserStripeExtractionResult();
        return result;
    }

    result.roiEstimate = estimateRoi(
        result.columnProfiles,
        grayImage.cols,
        grayImage.rows,
        options);

    setStatus(status, VisionPreprocessStatus::Succeeded);
    return result;
}

} // namespace i2r_rlc::vision
