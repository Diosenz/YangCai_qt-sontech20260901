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
 * @file        WeldSkeletonFeatureTools.cpp
 * @brief       焊接骨架特征提取工具
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
#include <queue>

#include <opencv2/imgproc.hpp>

#include "stripe/LaserLineResponseTools.h"
#include "stripe/WeldSkeletonFeatureTools.h"

namespace i2r_rlc::vision {

namespace {

/**
 * @brief 将骨架质量比例裁剪到 0~1。
 * @param value 输入比例或置信度。
 * @return 裁剪后的值，范围为 0~1。
 */
double clampRatio(double value)
{
    return std::max(0.0, std::min(1.0, value));
}

/**
 * @brief 判断骨架工具是否支持输入图像。
 * @param image 输入图像。
 * @return true 表示图像非空且类型为 `CV_8UC1`。
 */
bool isValidGrayImage(const cv::Mat &image)
{
    return !image.empty() && image.type() == CV_8UC1;
}

/**
 * @brief 校验通用骨架提取参数。
 * @param options 通用骨架提取参数。
 * @return true 表示阈值、面积、迭代次数和置信度范围均合法。
 */
bool isValidOptions(const WeldSkeletonExtractionOptions &options)
{
    return options.thresholdValue >= 0
        && options.thresholdValue <= 255
        && options.minComponentArea >= 0
        && options.maxIterations > 0
        && options.minConfidence >= 0.0
        && options.minConfidence <= 1.0;
}

/**
 * @brief 根据输入模式构造二值骨架候选 mask。
 * @param grayOrMask8 输入灰度图或二值 mask，类型为 `CV_8UC1`。
 * @param options 阈值和输入模式参数。
 * @return `CV_8UC1` 二值 mask，前景值为 255。
 */
cv::Mat makeBinaryMask(
    const cv::Mat &grayOrMask8,
    const WeldSkeletonExtractionOptions &options)
{
    cv::Mat mask;
    if (options.inputAlreadyBinary)
    {
        cv::threshold(grayOrMask8, mask, 0, 255, cv::THRESH_BINARY);
        return mask;
    }

    cv::threshold(
        grayOrMask8,
        mask,
        static_cast<double>(options.thresholdValue),
        255.0,
        cv::THRESH_BINARY);
    return mask;
}

/**
 * @brief 删除面积过小的二值连通域。
 * @param binaryMask8 输入二值 mask，类型为 `CV_8UC1`。
 * @param minComponentArea 最小保留面积，单位 pixel。
 * @return 过滤后的二值 mask；阈值小于等于 1 时返回副本。
 */
cv::Mat removeSmallComponents(const cv::Mat &binaryMask8, int minComponentArea)
{
    if (minComponentArea <= 1)
    {
        return binaryMask8.clone();
    }

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int componentCount = cv::connectedComponentsWithStats(
        binaryMask8,
        labels,
        stats,
        centroids,
        8,
        CV_32S);

    cv::Mat filtered = cv::Mat::zeros(binaryMask8.size(), CV_8UC1);
    for (int label = 1; label < componentCount; ++label)
    {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        if (area < minComponentArea)
        {
            continue;
        }

        filtered.setTo(255, labels == label);
    }
    return filtered;
}

/**
 * @brief 填充外轮廓连通域。
 * @param binaryMask8 输入二值 mask，类型为 `CV_8UC1`。
 * @param minComponentArea 最小保留轮廓面积，单位 pixel。
 * @return 填充后的二值 mask，前景值为 255。
 */
cv::Mat fillComponents(const cv::Mat &binaryMask8, int minComponentArea)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(
        binaryMask8,
        contours,
        cv::RETR_EXTERNAL,
        cv::CHAIN_APPROX_SIMPLE);

    cv::Mat filled = cv::Mat::zeros(binaryMask8.size(), CV_8UC1);
    for (const std::vector<cv::Point> &contour : contours)
    {
        if (static_cast<int>(std::abs(cv::contourArea(contour)))
            < minComponentArea)
        {
            continue;
        }

        std::vector<std::vector<cv::Point>> oneContour;
        oneContour.push_back(contour);
        cv::drawContours(filled, oneContour, 0, cv::Scalar(255), cv::FILLED);
    }
    return filled;
}

/**
 * @brief 统计 mask 前景像素数量。
 * @param mask 输入二值 mask。
 * @return 非零像素数量，单位 pixel。
 */
int foregroundCount(const cv::Mat &mask)
{
    return cv::countNonZero(mask);
}

/**
 * @brief 统计骨架像素的 8 邻域前景数量。
 * @param mask 输入二值 mask。
 * @param xPixel 查询列坐标，单位 pixel。
 * @param yPixel 查询行坐标，单位 pixel。
 * @return 8 邻域内非零像素数量。
 */
int neighborCount(const cv::Mat &mask, int xPixel, int yPixel)
{
    int count = 0;
    for (int dy = -1; dy <= 1; ++dy)
    {
        for (int dx = -1; dx <= 1; ++dx)
        {
            if (dx == 0 && dy == 0)
            {
                continue;
            }

            const int x = xPixel + dx;
            const int y = yPixel + dy;
            if (x < 0 || y < 0 || x >= mask.cols || y >= mask.rows)
            {
                continue;
            }

            if (mask.at<unsigned char>(y, x) > 0)
            {
                ++count;
            }
        }
    }
    return count;
}

/**
 * @brief 统计二值 mask 的前景连通分量数量。
 * @param mask 输入二值 mask。
 * @return 前景连通分量数量；空前景返回 0。
 */
int connectedComponentCount(const cv::Mat &mask)
{
    if (foregroundCount(mask) == 0)
    {
        return 0;
    }

    cv::Mat labels;
    return cv::connectedComponents(mask, labels, 8, CV_32S) - 1;
}

/**
 * @brief 校验激光骨架主流程参数。
 * @param options 激光骨架提取参数。
 * @return true 表示阈值、形态学、LOG、中心线和 Hough fallback 参数均合法。
 */
bool isValidLaserSkeletonOptions(const LaserSkeletonExtractionOptions &options)
{
    return options.thresholdValue >= 0
        && options.thresholdValue <= 255
        && options.minComponentArea >= 0
        && options.closingRadiusPixel >= 0
        && options.maxRegionThicknessPixel > 0
        && options.minRegionAspectRatio >= 1.0
        && options.maxRegionAreaPixel >= 0
        && options.regionPruningPeakHalfWidthPixel >= 0
        && options.maxThinningIterations > 0
        && options.logColumnStepPixel > 0
        && options.laserWidthPixel > 0
        && options.laserWidthMaxPixel >= options.laserWidthPixel
        && options.minLogSupportColumns >= 0
        && options.minBranchMeanGrayValue >= 0
        && options.minBranchMeanGrayValue <= 255
        && options.minCenterPointCount > 1
        && options.minConfidence >= 0.0
        && options.minConfidence <= 1.0
        && options.houghThreshold > 0
        && options.houghMinLineLengthPixel > 0.0
        && options.houghMaxLineGapPixel >= 0.0;
}

std::vector<int> makeLaserWidthByRow(
    int rowCount,
    const LaserSkeletonExtractionOptions &options)
{
    std::vector<int> laserWidthByRow(
        static_cast<std::size_t>(rowCount),
        std::max(1, options.laserWidthPixel));
    return laserWidthByRow;
}

void addPeakBand(
    cv::Mat *mask,
    int col,
    int row,
    const LaserSkeletonExtractionOptions &options)
{
    const int halfColumn = std::max(1, options.logColumnStepPixel / 2);
    const int halfRow = std::max(
        1,
        options.regionPruningPeakHalfWidthPixel);
    const int x1 = std::max(0, col - halfColumn);
    const int x2 = std::min(mask->cols - 1, col + halfColumn);
    const int y1 = std::max(0, row - halfRow);
    const int y2 = std::min(mask->rows - 1, row + halfRow);
    cv::rectangle(
        *mask,
        cv::Rect(cv::Point(x1, y1), cv::Point(x2 + 1, y2 + 1)),
        cv::Scalar(255),
        cv::FILLED);
}

cv::Mat pruneLaserLikeRegions(
    const cv::Mat &grayImage,
    const cv::Mat &thresholdMask,
    const LaserSkeletonExtractionOptions &options)
{
    if (!options.enableLaserRegionPruning)
    {
        return thresholdMask.clone();
    }

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int labelCount = cv::connectedComponentsWithStats(
        thresholdMask,
        labels,
        stats,
        centroids,
        8,
        CV_32S);
    if (labelCount <= 1)
    {
        return thresholdMask.clone();
    }

    const std::vector<int> laserWidthByRow =
        makeLaserWidthByRow(grayImage.rows, options);
    LaserLogPeakOptions logOptions;
    logOptions.rowMinPixel = 0;
    logOptions.rowMaxPixel = grayImage.rows;
    logOptions.columnStepPixel = options.logColumnStepPixel;
    logOptions.requiredPeakGrayValue = 255;
    const LaserLogPeakResult logResult = CLaserLineResponseTools::detectLogPeaks(
        grayImage,
        laserWidthByRow,
        logOptions,
        nullptr);

    std::vector<int> supportByLabel(static_cast<std::size_t>(labelCount), 0);
    cv::Mat peakBands = cv::Mat::zeros(thresholdMask.size(), CV_8UC1);
    if (logResult.valid)
    {
        for (int col = 0;
             col < static_cast<int>(logResult.maxRowByColumn.size());
             col += options.logColumnStepPixel)
        {
            const int row = logResult.maxRowByColumn[static_cast<std::size_t>(col)];
            if (row < 0 || row >= labels.rows || col < 0 || col >= labels.cols)
            {
                continue;
            }

            const int label = labels.at<int>(row, col);
            if (label <= 0 || label >= labelCount)
            {
                continue;
            }
            ++supportByLabel[static_cast<std::size_t>(label)];
            addPeakBand(&peakBands, col, row, options);
        }
    }

    cv::Mat filtered = cv::Mat::zeros(thresholdMask.size(), CV_8UC1);
    for (int label = 1; label < labelCount; ++label)
    {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        if (area < options.minComponentArea)
        {
            continue;
        }

        const int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
        const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
        const int longAxis = std::max(width, height);
        const int shortAxis = std::max(1, std::min(width, height));
        const double aspectRatio =
            static_cast<double>(longAxis) / static_cast<double>(shortAxis);
        const double thickness =
            static_cast<double>(area) / static_cast<double>(std::max(1, longAxis));
        const int support = supportByLabel[static_cast<std::size_t>(label)];
        const bool hasLogSupport = support >= options.minLogSupportColumns;
        const bool largeRegion =
            options.maxRegionAreaPixel > 0
            && area > options.maxRegionAreaPixel;
        const bool bulkyLargeRegion =
            largeRegion
            && aspectRatio < std::max(4.0, options.minRegionAspectRatio * 2.5);
        const bool wideRegion =
            thickness > static_cast<double>(options.maxRegionThicknessPixel)
            || bulkyLargeRegion;

        cv::Mat componentMask = labels == label;
        if (!wideRegion && aspectRatio >= options.minRegionAspectRatio)
        {
            filtered.setTo(255, componentMask);
            continue;
        }

        if (hasLogSupport)
        {
            cv::Mat componentPeakBand;
            cv::bitwise_and(peakBands, peakBands, componentPeakBand, componentMask);
            filtered.setTo(255, componentPeakBand);
        }
    }

    if (foregroundCount(filtered) == 0)
    {
        return thresholdMask.clone();
    }

    cv::Mat connected;
    const int kernelSize =
        std::max(3, 2 * std::max(1, options.regionPruningPeakHalfWidthPixel / 2) + 1);
    cv::morphologyEx(
        filtered,
        connected,
        cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, 3)));
    return connected;
}

cv::Mat keepLogSupportedComponents(
    const cv::Mat &grayImage,
    const cv::Mat &skeletonMask,
    const LaserSkeletonExtractionOptions &options)
{
    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int labelCount = cv::connectedComponentsWithStats(
        skeletonMask,
        labels,
        stats,
        centroids,
        8,
        CV_32S);
    if (labelCount <= 2)
    {
        return skeletonMask.clone();
    }

    const std::vector<int> laserWidthByRow =
        makeLaserWidthByRow(grayImage.rows, options);
    LaserLogPeakOptions logOptions;
    logOptions.rowMinPixel = 0;
    logOptions.rowMaxPixel = grayImage.rows;
    logOptions.columnStepPixel = options.logColumnStepPixel;
    logOptions.requiredPeakGrayValue = 255;
    const LaserLogPeakResult logResult = CLaserLineResponseTools::detectLogPeaks(
        grayImage,
        laserWidthByRow,
        logOptions,
        nullptr);

    std::vector<int> supportByLabel(static_cast<std::size_t>(labelCount), 0);
    if (logResult.valid)
    {
        for (int col = 0;
             col < static_cast<int>(logResult.maxRowByColumn.size());
             col += options.logColumnStepPixel)
        {
            const int row = logResult.maxRowByColumn[col];
            if (row < 0 || row >= labels.rows)
            {
                continue;
            }

            const int label = labels.at<int>(row, col);
            if (label > 0 && label < labelCount)
            {
                ++supportByLabel[static_cast<std::size_t>(label)];
            }
        }
    }

    int bestSupport = 0;
    for (int label = 1; label < labelCount; ++label)
    {
        bestSupport = std::max(
            bestSupport,
            supportByLabel[static_cast<std::size_t>(label)]);
    }

    cv::Mat filtered = cv::Mat::zeros(skeletonMask.size(), CV_8UC1);
    for (int label = 1; label < labelCount; ++label)
    {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        const double meanGray = cv::mean(grayImage, labels == label)[0];
        const int support = supportByLabel[static_cast<std::size_t>(label)];
        const bool enoughSupport =
            support >= options.minLogSupportColumns
            || (bestSupport > 0 && support == bestSupport);
        if (area >= options.minComponentArea
            && meanGray >= options.minBranchMeanGrayValue
            && enoughSupport)
        {
            filtered.setTo(255, labels == label);
        }
    }

    if (foregroundCount(filtered) == 0)
    {
        return skeletonMask.clone();
    }
    return filtered;
}

std::vector<cv::Point2d> centerPointsByColumn(const cv::Mat &skeletonMask)
{
    std::vector<cv::Point2d> points;
    for (int col = 0; col < skeletonMask.cols; ++col)
    {
        double sumRow = 0.0;
        int count = 0;
        for (int row = 0; row < skeletonMask.rows; ++row)
        {
            if (skeletonMask.at<unsigned char>(row, col) > 0)
            {
                sumRow += static_cast<double>(row);
                ++count;
            }
        }

        if (count > 0)
        {
            points.push_back(cv::Point2d(
                static_cast<double>(col),
                sumRow / static_cast<double>(count)));
        }
    }
    return points;
}

std::vector<int> logPeakRowsByColumn(
    const cv::Mat &grayImage,
    const LaserSkeletonExtractionOptions &options)
{
    std::vector<int> rows(static_cast<std::size_t>(grayImage.cols), -1);
    const std::vector<int> laserWidthByRow =
        makeLaserWidthByRow(grayImage.rows, options);
    LaserLogPeakOptions logOptions;
    logOptions.rowMinPixel = 0;
    logOptions.rowMaxPixel = grayImage.rows;
    logOptions.columnStepPixel = options.logColumnStepPixel;
    logOptions.requiredPeakGrayValue = 255;
    const LaserLogPeakResult logResult = CLaserLineResponseTools::detectLogPeaks(
        grayImage,
        laserWidthByRow,
        logOptions,
        nullptr);
    if (!logResult.valid)
    {
        return rows;
    }

    for (int col = 0;
         col < grayImage.cols
         && col < static_cast<int>(logResult.maxRowByColumn.size());
         ++col)
    {
        rows[static_cast<std::size_t>(col)] =
            logResult.maxRowByColumn[static_cast<std::size_t>(col)];
    }
    return rows;
}

double orderedPathCandidateScore(
    const cv::Mat &grayImage,
    const std::vector<int> &logRowsByColumn,
    int col,
    int row,
    const LaserSkeletonExtractionOptions &options)
{
    double score = static_cast<double>(grayImage.at<unsigned char>(row, col));
    if (col >= 0 && col < static_cast<int>(logRowsByColumn.size()))
    {
        const int logRow = logRowsByColumn[static_cast<std::size_t>(col)];
        if (logRow >= 0)
        {
            const int distance = std::abs(row - logRow);
            const int supportHalfWidth =
                std::max(2, options.laserWidthMaxPixel);
            if (distance <= supportHalfWidth)
            {
                score += 255.0
                    * (1.0
                       - static_cast<double>(distance)
                           / static_cast<double>(supportHalfWidth + 1));
            }
        }
    }
    return score;
}

std::vector<cv::Point2d> orderedPathPointsByColumn(
    const cv::Mat &grayImage,
    const cv::Mat &skeletonMask,
    const LaserSkeletonExtractionOptions &options)
{
    // 旧 ImageProFeng 的 SkeletonExtraction 不是对同列骨架求平均，而是在
    // 分叉、重叠段筛选后逐列取一个主路径点。这里用连续性 + 灰度/LOG 支撑
    // 做工程等价选择，避免 V/T/角接结构被压成水平伪中心线。
    std::vector<std::vector<int>> rowsByColumn(
        static_cast<std::size_t>(skeletonMask.cols));
    for (int row = 0; row < skeletonMask.rows; ++row)
    {
        for (int col = 0; col < skeletonMask.cols; ++col)
        {
            if (skeletonMask.at<unsigned char>(row, col) > 0)
            {
                rowsByColumn[static_cast<std::size_t>(col)].push_back(row);
            }
        }
    }

    const std::vector<int> logRowsByColumn =
        logPeakRowsByColumn(grayImage, options);
    std::vector<cv::Point2d> selected;
    selected.reserve(static_cast<std::size_t>(skeletonMask.cols));
    int previousRow = -1;
    int previousCol = -1;
    const int baseJumpTolerance =
        std::max(12, options.laserWidthMaxPixel * 3);
    for (int col = 0; col < skeletonMask.cols; ++col)
    {
        const std::vector<int> &rows =
            rowsByColumn[static_cast<std::size_t>(col)];
        if (rows.empty())
        {
            continue;
        }

        int bestRow = rows.front();
        double bestCost = std::numeric_limits<double>::max();
        for (int row : rows)
        {
            const double supportScore = orderedPathCandidateScore(
                grayImage,
                logRowsByColumn,
                col,
                row,
                options);
            double continuityPenalty = 0.0;
            if (previousRow >= 0)
            {
                const int colGap = std::max(1, col - previousCol);
                continuityPenalty =
                    static_cast<double>(std::abs(row - previousRow))
                    / static_cast<double>(colGap);
            }

            const double cost = continuityPenalty * 8.0 - supportScore / 32.0;
            if (cost < bestCost)
            {
                bestCost = cost;
                bestRow = row;
            }
        }

        if (previousRow >= 0)
        {
            const int colGap = std::max(1, col - previousCol);
            const int allowedJump = baseJumpTolerance + colGap * 2;
            if (std::abs(bestRow - previousRow) > allowedJump)
            {
                const double bestSupport = orderedPathCandidateScore(
                    grayImage,
                    logRowsByColumn,
                    col,
                    bestRow,
                    options);
                if (bestSupport
                    < static_cast<double>(options.minBranchMeanGrayValue))
                {
                    continue;
                }
            }
        }

        selected.push_back(cv::Point2d(
            static_cast<double>(col),
            static_cast<double>(bestRow)));
        previousRow = bestRow;
        previousCol = col;
    }
    return selected;
}

std::vector<cv::Point2d> refineCenterPoints(
    const cv::Mat &grayImage,
    const std::vector<cv::Point2d> &centerPoints,
    const LaserSkeletonExtractionOptions &options)
{
    if (centerPoints.empty())
    {
        return centerPoints;
    }

    std::vector<int> peakRowsByColumn(
        static_cast<std::size_t>(grayImage.cols),
        -1);
    for (const cv::Point2d &point : centerPoints)
    {
        const int col = static_cast<int>(std::round(point.x));
        const int row = static_cast<int>(std::round(point.y));
        if (col >= 0 && col < grayImage.cols && row >= 0 && row < grayImage.rows)
        {
            peakRowsByColumn[static_cast<std::size_t>(col)] = row;
        }
    }

    const std::vector<int> laserWidthByRow =
        makeLaserWidthByRow(grayImage.rows, options);
    const LaserCenterOfMassResult centerOfMass =
        CLaserLineResponseTools::refineCenterOfMass(
            grayImage,
            peakRowsByColumn,
            laserWidthByRow,
            options.laserWidthMaxPixel,
            nullptr);
    if (!centerOfMass.valid)
    {
        return centerPoints;
    }

    std::vector<cv::Point2d> refinedPoints;
    for (const cv::Point2d &point : centerPoints)
    {
        const int col = static_cast<int>(std::round(point.x));
        if (col < 0
            || col >= static_cast<int>(centerOfMass.refinedRowByColumn.size()))
        {
            continue;
        }

        const double refinedRow =
            centerOfMass.refinedRowByColumn[static_cast<std::size_t>(col)];
        if (refinedRow >= 0.0 && refinedRow < grayImage.rows)
        {
            refinedPoints.push_back(cv::Point2d(point.x, refinedRow));
        }
    }

    return refinedPoints.empty() ? centerPoints : refinedPoints;
}

cv::Mat makeMaskFromPoints(
    const cv::Size &size,
    const std::vector<cv::Point2d> &points)
{
    cv::Mat mask = cv::Mat::zeros(size, CV_8UC1);
    for (const cv::Point2d &point : points)
    {
        const int col = static_cast<int>(std::round(point.x));
        const int row = static_cast<int>(std::round(point.y));
        if (col >= 0 && col < size.width && row >= 0 && row < size.height)
        {
            mask.at<unsigned char>(row, col) = 255;
        }
    }
    return mask;
}

std::vector<cv::Point2d> extractHoughCenterPoints(
    const cv::Mat &binaryMask,
    const LaserSkeletonExtractionOptions &options)
{
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(
        binaryMask,
        lines,
        1.0,
        CV_PI / 180.0,
        options.houghThreshold,
        options.houghMinLineLengthPixel,
        options.houghMaxLineGapPixel);
    if (lines.empty())
    {
        return {};
    }

    std::sort(
        lines.begin(),
        lines.end(),
        [](const cv::Vec4i &left, const cv::Vec4i &right) {
            const double leftLength = std::hypot(
                static_cast<double>(left[2] - left[0]),
                static_cast<double>(left[3] - left[1]));
            const double rightLength = std::hypot(
                static_cast<double>(right[2] - right[0]),
                static_cast<double>(right[3] - right[1]));
            return leftLength > rightLength;
        });

    std::vector<double> rowSum(static_cast<std::size_t>(binaryMask.cols), 0.0);
    std::vector<int> rowCount(static_cast<std::size_t>(binaryMask.cols), 0);
    const int maxLines = std::min(2, static_cast<int>(lines.size()));
    for (int index = 0; index < maxLines; ++index)
    {
        const cv::Vec4i &line = lines[static_cast<std::size_t>(index)];
        cv::LineIterator iterator(
            binaryMask,
            cv::Point(line[0], line[1]),
            cv::Point(line[2], line[3]),
            8);
        for (int step = 0; step < iterator.count; ++step, ++iterator)
        {
            const cv::Point point = iterator.pos();
            if (point.x < 0 || point.x >= binaryMask.cols
                || point.y < 0 || point.y >= binaryMask.rows)
            {
                continue;
            }

            rowSum[static_cast<std::size_t>(point.x)] += point.y;
            ++rowCount[static_cast<std::size_t>(point.x)];
        }
    }

    std::vector<cv::Point2d> centerPoints;
    for (int col = 0; col < binaryMask.cols; ++col)
    {
        const int count = rowCount[static_cast<std::size_t>(col)];
        if (count > 0)
        {
            centerPoints.push_back(cv::Point2d(
                static_cast<double>(col),
                rowSum[static_cast<std::size_t>(col)]
                    / static_cast<double>(count)));
        }
    }
    return centerPoints;
}

int transitionCount(
    unsigned char p2,
    unsigned char p3,
    unsigned char p4,
    unsigned char p5,
    unsigned char p6,
    unsigned char p7,
    unsigned char p8,
    unsigned char p9)
{
    const unsigned char neighbors[9] = {
        p2, p3, p4, p5, p6, p7, p8, p9, p2
    };

    int transitions = 0;
    for (int index = 0; index < 8; ++index)
    {
        if (neighbors[index] == 0 && neighbors[index + 1] != 0)
        {
            ++transitions;
        }
    }
    return transitions;
}

void zhangSuenIteration(cv::Mat *binary01, int pass, bool *changed)
{
    std::vector<cv::Point> pixelsToClear;
    for (int y = 1; y < binary01->rows - 1; ++y)
    {
        for (int x = 1; x < binary01->cols - 1; ++x)
        {
            if (binary01->at<unsigned char>(y, x) == 0)
            {
                continue;
            }

            const unsigned char p2 = binary01->at<unsigned char>(y - 1, x);
            const unsigned char p3 = binary01->at<unsigned char>(y - 1, x + 1);
            const unsigned char p4 = binary01->at<unsigned char>(y, x + 1);
            const unsigned char p5 = binary01->at<unsigned char>(y + 1, x + 1);
            const unsigned char p6 = binary01->at<unsigned char>(y + 1, x);
            const unsigned char p7 = binary01->at<unsigned char>(y + 1, x - 1);
            const unsigned char p8 = binary01->at<unsigned char>(y, x - 1);
            const unsigned char p9 = binary01->at<unsigned char>(y - 1, x - 1);

            const int neighborSum =
                p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
            const int transitions =
                transitionCount(p2, p3, p4, p5, p6, p7, p8, p9);
            if (neighborSum < 2 || neighborSum > 6 || transitions != 1)
            {
                continue;
            }

            const bool removeInFirstPass =
                p2 * p4 * p6 == 0 && p4 * p6 * p8 == 0;
            const bool removeInSecondPass =
                p2 * p4 * p8 == 0 && p2 * p6 * p8 == 0;
            if ((pass == 0 && removeInFirstPass)
                || (pass == 1 && removeInSecondPass))
            {
                pixelsToClear.push_back(cv::Point(x, y));
            }
        }
    }

    for (const cv::Point &point : pixelsToClear)
    {
        binary01->at<unsigned char>(point.y, point.x) = 0;
    }

    if (!pixelsToClear.empty() && changed != nullptr)
    {
        *changed = true;
    }
}

} // namespace

WeldSkeletonFeatureResult CWeldSkeletonFeatureTools::extract(
    const cv::Mat &grayOrMask8,
    const WeldSkeletonExtractionOptions &options)
{
    WeldSkeletonFeatureResult result;
    if (!isValidGrayImage(grayOrMask8))
    {
        result.message = grayOrMask8.empty()
            ? "empty input"
            : "input must be CV_8UC1";
        return result;
    }

    if (!isValidOptions(options))
    {
        result.message = "invalid options";
        return result;
    }

    const cv::Mat binaryMask = removeSmallComponents(
        makeBinaryMask(grayOrMask8, options),
        options.minComponentArea);
    const int foregroundPixelCount = foregroundCount(binaryMask);
    if (foregroundPixelCount <= 0)
    {
        result.message = "empty foreground";
        return result;
    }

    result.skeletonMask = thinBinaryMask(binaryMask, options.maxIterations);
    result.skeletonPoints = collectSkeletonPoints(result.skeletonMask);
    result.centerLinePoints = centerPointsByColumn(result.skeletonMask);
    result.refinedCenterPoints = result.centerLinePoints;
    result.connectedComponentCount = connectedComponentCount(result.skeletonMask);

    for (const cv::Point2d &point : result.skeletonPoints)
    {
        const int x = static_cast<int>(point.x);
        const int y = static_cast<int>(point.y);
        const int neighbors = neighborCount(result.skeletonMask, x, y);
        if (neighbors == 1)
        {
            ++result.endpointCount;
        }
        else if (neighbors >= 3)
        {
            ++result.branchPointCount;
        }
    }

    result.validPixelRatio = clampRatio(
        static_cast<double>(result.skeletonPoints.size())
        / static_cast<double>(foregroundPixelCount));
    const double componentScore =
        result.connectedComponentCount == 1 ? 1.0 : 0.35;
    const double endpointScore =
        result.endpointCount >= 2 ? 1.0 : 0.55;
    const double densityScore =
        result.validPixelRatio > 0.0 ? 1.0 : 0.0;
    result.confidence = clampRatio(
        0.50 * componentScore
        + 0.30 * endpointScore
        + 0.20 * densityScore);
    result.valid = result.connectedComponentCount == 1
        && result.confidence >= options.minConfidence;
    result.message = result.valid
        ? "skeleton feature extracted"
        : (result.connectedComponentCount > 1
               ? "skeleton is broken"
               : "skeleton confidence is low");
    return result;
}

WeldSkeletonFeatureResult CWeldSkeletonFeatureTools::extractLaserSkeleton(
    const cv::Mat &grayImage,
    const LaserSkeletonExtractionOptions &options)
{
    // 1. 输入校验：该公共流程只接收单通道 8 bit 灰度图，输出点单位为 pixel。
    WeldSkeletonFeatureResult result;
    if (!isValidGrayImage(grayImage))
    {
        result.message = grayImage.empty()
            ? "empty input"
            : "input must be CV_8UC1";
        return result;
    }

    if (!isValidLaserSkeletonOptions(options))
    {
        result.message = "invalid laser skeleton options";
        return result;
    }

    // 2. 阈值分割：先得到亮区域，保持旧工程 threshold 的入口语义。
    cv::Mat thresholdMask;
    cv::threshold(
        grayImage,
        thresholdMask,
        static_cast<double>(options.thresholdValue),
        255.0,
        cv::THRESH_BINARY);

    // 3. 连通域填充：替代旧工程中的 Connection 与 FillUp。
    const cv::Mat filled =
        fillComponents(thresholdMask, options.minComponentArea);
    if (foregroundCount(filled) == 0)
    {
        result.message = "empty foreground";
        return result;
    }

    // 4. 可选激光候选约束：该步骤属于业务主干筛选，不是 HALCON skeleton
    //    算子的默认语义；默认关闭，避免把旧工程可容忍的分叉和短支路提前删掉。
    const cv::Mat skeletonRegion = options.enableLaserRegionPruning
        ? pruneLaserLikeRegions(grayImage, filled, options)
        : filled;

    // 5. closing：用椭圆结构元近似旧工程 closing_circle，连接激光条纹小断裂。
    cv::Mat closed = skeletonRegion;
    if (options.closingRadiusPixel > 0)
    {
        const int kernelSize = 2 * options.closingRadiusPixel + 1;
        const cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size(kernelSize, kernelSize));
        cv::morphologyEx(skeletonRegion, closed, cv::MORPH_CLOSE, kernel);
    }
    result.preprocessedMask = closed.clone();

    // 6. HALCON-like 完整骨架：只做细化和拓扑统计，默认保留分叉、
    //    短支路和多连通段，避免下游算法拿不到旧 skeleton 中存在的结构。
    const cv::Mat fullSkeletonMask =
        thinBinaryMask(closed, options.maxThinningIterations);
    result.skeletonMask = fullSkeletonMask;
    result.skeletonPoints = collectSkeletonPoints(result.skeletonMask);
    result.connectedComponentCount = connectedComponentCount(result.skeletonMask);
    for (const cv::Point2d &point : result.skeletonPoints)
    {
        const int x = static_cast<int>(std::round(point.x));
        const int y = static_cast<int>(std::round(point.y));
        const int neighbors = neighborCount(result.skeletonMask, x, y);
        if (neighbors == 1)
        {
            ++result.endpointCount;
        }
        else if (neighbors >= 3)
        {
            ++result.branchPointCount;
        }
    }

    if (result.skeletonPoints.empty())
    {
        result.message = "empty skeleton";
        return result;
    }

    // 7. 派生中心线：LOG 支撑筛选只影响中心线来源，不回写 skeletonMask。
    const cv::Mat centerSourceMask = options.enableLogSupportedComponentFilter
        ? keepLogSupportedComponents(grayImage, result.skeletonMask, options)
        : result.skeletonMask;
    result.centerLinePoints = centerPointsByColumn(centerSourceMask);
    result.refinedCenterPoints =
        refineCenterPoints(grayImage, result.centerLinePoints, options);
    if (options.enableOrderedPathExtraction)
    {
        result.orderedPathPoints = refineCenterPoints(
            grayImage,
            orderedPathPointsByColumn(grayImage, centerSourceMask, options),
            options);
    }
    const std::size_t primaryPathPointCount = options.enableOrderedPathExtraction
        ? std::max(
            result.refinedCenterPoints.size(),
            result.orderedPathPoints.size())
        : result.refinedCenterPoints.size();

    // 8. 备用路径：主骨架点不足或断裂明显时，用 HoughLinesP 提取两段主线，
    //    只替换通用派生中心线，不覆盖完整骨架 mask，也不覆盖旧 Feng
    //    T 几何分支使用的 orderedPathPoints，避免两条 Hough 线按列平均。
    const bool needsHoughFallback =
        static_cast<int>(primaryPathPointCount) < options.minCenterPointCount
        || (options.enableHoughFallback && result.connectedComponentCount > 1);
    bool usedHoughFallback = false;
    if (needsHoughFallback)
    {
        if (options.enableHoughFallback)
        {
            std::vector<cv::Point2d> houghPoints =
                extractHoughCenterPoints(closed, options);
            houghPoints = refineCenterPoints(grayImage, houghPoints, options);
            if (static_cast<int>(houghPoints.size())
                >= options.minCenterPointCount)
            {
                result.centerLinePoints = houghPoints;
                result.refinedCenterPoints = houghPoints;
                usedHoughFallback = true;
            }
        }
    }

    // 9. 输出质量：用点数、连通性和前景占比形成轻量置信度，调用方可继续筛选。
    result.validPixelRatio = clampRatio(
        static_cast<double>(result.skeletonPoints.size())
        / static_cast<double>(std::max(1, foregroundCount(closed))));
    const double centerScore = std::min(
        1.0,
        static_cast<double>(primaryPathPointCount)
            / static_cast<double>(std::max(1, options.minCenterPointCount * 4)));
    const double topologyScore = result.connectedComponentCount <= 2 ? 1.0 : 0.75;
    result.confidence = clampRatio(
        0.40
        + 0.35 * centerScore
        + 0.15 * (result.validPixelRatio > 0.0 ? 1.0 : 0.0)
        + 0.10 * topologyScore);
    result.valid =
        static_cast<int>(primaryPathPointCount) >= options.minCenterPointCount
        && result.confidence >= options.minConfidence;
    result.message = result.valid
        ? (usedHoughFallback
               ? "halcon-like laser skeleton extracted by hough fallback"
               : "halcon-like laser skeleton extracted")
        : (static_cast<int>(primaryPathPointCount) < options.minCenterPointCount
               ? "not enough center points"
               : "halcon-like skeleton confidence is low");
    return result;
}

cv::Mat CWeldSkeletonFeatureTools::thinBinaryMask(
    const cv::Mat &binaryMask8,
    int maxIterations)
{
    if (!isValidGrayImage(binaryMask8) || maxIterations <= 0)
    {
        return cv::Mat();
    }

    cv::Mat binary01;
    cv::threshold(binaryMask8, binary01, 0, 1, cv::THRESH_BINARY);
    for (int iteration = 0; iteration < maxIterations; ++iteration)
    {
        bool changed = false;
        zhangSuenIteration(&binary01, 0, &changed);
        zhangSuenIteration(&binary01, 1, &changed);
        if (!changed)
        {
            break;
        }
    }

    cv::Mat skeletonMask;
    binary01.convertTo(skeletonMask, CV_8UC1, 255.0);
    return skeletonMask;
}

std::vector<cv::Point2d> CWeldSkeletonFeatureTools::collectSkeletonPoints(
    const cv::Mat &skeletonMask8)
{
    std::vector<cv::Point2d> points;
    if (!isValidGrayImage(skeletonMask8))
    {
        return points;
    }

    for (int y = 0; y < skeletonMask8.rows; ++y)
    {
        for (int x = 0; x < skeletonMask8.cols; ++x)
        {
            if (skeletonMask8.at<unsigned char>(y, x) > 0)
            {
                points.push_back(cv::Point2d(
                    static_cast<double>(x),
                    static_cast<double>(y)));
            }
        }
    }
    return points;
}

} // namespace i2r_rlc::vision

