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
 * @file        GapFeatureDetector.cpp
 * @brief       焊缝间隙特征检测实现
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
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "feature/GapFeatureDetector.h"

namespace i2r_rlc::vision {

namespace {

double clampRatio(double value)
{
    return std::max(0.0, std::min(1.0, value));
}

bool isValidOptions(const WeldFeatureDetectionOptions &options)
{
    return options.searchHalfWindowY >= 0
        && options.minEdgeContrast >= 0
        && options.minEdgeContrast <= 255
        && options.minGapWidthPixel >= 0
        && options.maxGapWidthPixel >= options.minGapWidthPixel
        && options.minConfidence >= 0.0
        && options.minConfidence <= 1.0
        && options.edgeSearchHalfWidthPixel >= 0;
}

bool isValidGrayImage(const cv::Mat &gray8)
{
    return !gray8.empty() && gray8.type() == CV_8UC1;
}

int clampToIntPixel(double value, int minValue, int maxValue)
{
    const int rounded = static_cast<int>(std::floor(value + 0.5));
    return std::max(minValue, std::min(maxValue, rounded));
}

std::vector<int> collectCandidateRows(
    const cv::Mat &gray8,
    double centerYPixel,
    int searchHalfWindowY)
{
    std::vector<int> rows;
    const int centerY = clampToIntPixel(centerYPixel, 0, gray8.rows - 1);
    const int beginY = std::max(1, centerY - searchHalfWindowY);
    const int endY = std::min(gray8.rows - 2, centerY + searchHalfWindowY);
    for (int yPixel = beginY; yPixel <= endY; ++yPixel)
    {
        rows.push_back(yPixel);
    }
    return rows;
}

double widthConfidence(
    double widthPixel,
    const WeldFeatureDetectionOptions &options)
{
    if (widthPixel < static_cast<double>(options.minGapWidthPixel)
        || widthPixel > static_cast<double>(options.maxGapWidthPixel))
    {
        return 0.0;
    }

    const double allowedWidth =
        static_cast<double>(options.maxGapWidthPixel - options.minGapWidthPixel);
    if (allowedWidth <= 0.0)
    {
        return 1.0;
    }

    const double centerWidth =
        0.5 * static_cast<double>(
            options.minGapWidthPixel + options.maxGapWidthPixel);
    const double normalizedDistance =
        std::fabs(widthPixel - centerWidth) / (0.5 * allowedWidth);
    return clampRatio(1.0 - 0.35 * normalizedDistance);
}

double centerPriorConfidence(
    double leftX,
    double rightX,
    double centerX,
    const WeldFeatureDetectionOptions &options)
{
    if (!options.useCenterlinePrior)
    {
        return 1.0;
    }

    const double halfWidth = std::max(1.0, 0.5 * (rightX - leftX));
    const double midpoint = 0.5 * (leftX + rightX);
    const double normalizedOffset = std::fabs(midpoint - centerX) / halfWidth;
    return clampRatio(1.0 - 0.5 * normalizedOffset);
}

double rowPriorConfidence(double yPixel, double centerYPixel, int halfWindow)
{
    if (halfWindow <= 0)
    {
        return std::fabs(yPixel - centerYPixel) <= 0.5 ? 1.0 : 0.0;
    }

    const double normalizedDistance =
        std::fabs(yPixel - centerYPixel) / static_cast<double>(halfWindow);
    return clampRatio(1.0 - 0.30 * normalizedDistance);
}

double calculateSignedEdgeContrast(
    const cv::Mat &gray8,
    int xPixel,
    int yPixel,
    int searchHalfWindowY)
{
    if (!isValidGrayImage(gray8)
        || xPixel <= 0
        || xPixel >= gray8.cols - 1
        || yPixel < 0
        || yPixel >= gray8.rows)
    {
        return 0.0;
    }

    const int verticalHalfWindow = std::min(1, std::max(0, searchHalfWindowY));
    const int beginY = std::max(0, yPixel - verticalHalfWindow);
    const int endY = std::min(gray8.rows - 1, yPixel + verticalHalfWindow);

    double contrastSum = 0.0;
    int sampleCount = 0;
    for (int sampleY = beginY; sampleY <= endY; ++sampleY)
    {
        const unsigned char leftValue =
            gray8.at<unsigned char>(sampleY, xPixel - 1);
        const unsigned char rightValue =
            gray8.at<unsigned char>(sampleY, xPixel + 1);
        contrastSum += static_cast<double>(rightValue)
            - static_cast<double>(leftValue);
        ++sampleCount;
    }

    if (sampleCount == 0)
    {
        return 0.0;
    }

    return contrastSum / static_cast<double>(sampleCount);
}

WeldFeaturePoint makeEdgePoint(
    double xPixel,
    double yPixel,
    double confidence,
    const std::string &label)
{
    WeldFeaturePoint point;
    point.xPixel = xPixel;
    point.yPixel = yPixel;
    point.confidence = clampRatio(confidence);
    point.valid = point.confidence > 0.0;
    point.label = label;
    return point;
}

GapEdgeCandidate makeEdgeCandidate(
    double xPixel,
    double yPixel,
    double contrastGrayValue,
    double centerYPixel,
    const WeldFeatureDetectionOptions &options,
    const std::string &label)
{
    GapEdgeCandidate candidate;
    candidate.xPixel = xPixel;
    candidate.yPixel = yPixel;
    candidate.contrastGrayValue = std::max(0.0, contrastGrayValue);
    const double contrastScore =
        clampRatio(candidate.contrastGrayValue / 255.0);
    const double rowScore = rowPriorConfidence(
        yPixel,
        centerYPixel,
        options.searchHalfWindowY);
    candidate.confidence = clampRatio(
        0.75 * contrastScore + 0.25 * rowScore);
    candidate.valid = candidate.confidence > 0.0;
    candidate.label = label;
    return candidate;
}

struct EdgePairSelection {
    WeldEdgePair edgePair;
    int selectedRowPixel = -1;
    double edgeContrastGrayValue = 0.0;
    std::string failureReason;
};

EdgePairSelection selectBestEdgePair(
    const std::vector<GapEdgeCandidate> &leftCandidates,
    const std::vector<GapEdgeCandidate> &rightCandidates,
    double centerXPixel,
    double centerYPixel,
    const WeldFeatureDetectionOptions &options)
{
    EdgePairSelection selection;
    if (leftCandidates.empty() && rightCandidates.empty())
    {
        selection.failureReason = "edge candidate not found";
        selection.edgePair.message = selection.failureReason;
        return selection;
    }
    if (leftCandidates.empty())
    {
        selection.failureReason = "left edge candidate not found";
        selection.edgePair.message = selection.failureReason;
        return selection;
    }
    if (rightCandidates.empty())
    {
        selection.failureReason = "right edge candidate not found";
        selection.edgePair.message = selection.failureReason;
        return selection;
    }

    double bestScore = -std::numeric_limits<double>::infinity();
    for (const GapEdgeCandidate &leftCandidate : leftCandidates)
    {
        for (const GapEdgeCandidate &rightCandidate : rightCandidates)
        {
            if (rightCandidate.xPixel <= leftCandidate.xPixel)
            {
                continue;
            }

            // V2-A2.1 仍是行截面 gap 检测，不跨行拼接左右边缘。
            // 跨行边缘拟合留给下游线段/RANSAC 语义扩展。
            if (std::fabs(rightCandidate.yPixel - leftCandidate.yPixel) > 0.5)
            {
                continue;
            }

            const double widthPixel =
                rightCandidate.xPixel - leftCandidate.xPixel;
            const double widthScore = widthConfidence(widthPixel, options);
            if (widthScore <= 0.0)
            {
                continue;
            }

            const double contrastScore = clampRatio(
                0.5 * (leftCandidate.contrastGrayValue
                       + rightCandidate.contrastGrayValue) / 255.0);
            const double centerScore = centerPriorConfidence(
                leftCandidate.xPixel,
                rightCandidate.xPixel,
                centerXPixel,
                options);
            const double rowScore = rowPriorConfidence(
                0.5 * (leftCandidate.yPixel + rightCandidate.yPixel),
                centerYPixel,
                options.searchHalfWindowY);
            const double pairScore =
                contrastScore * 0.40
                + widthScore * 0.25
                + centerScore * 0.25
                + rowScore * 0.10;

            if (pairScore <= bestScore)
            {
                continue;
            }

            bestScore = pairScore;
            selection.edgePair.left = makeEdgePoint(
                leftCandidate.xPixel,
                leftCandidate.yPixel,
                leftCandidate.confidence,
                "left_edge");
            selection.edgePair.right = makeEdgePoint(
                rightCandidate.xPixel,
                rightCandidate.yPixel,
                rightCandidate.confidence,
                "right_edge");
            selection.edgePair.widthPixel = widthPixel;
            selection.edgePair.confidence = clampRatio(pairScore);
            selection.edgePair.valid =
                selection.edgePair.confidence >= options.minConfidence;
            selection.edgePair.message = selection.edgePair.valid
                ? "edge pair detected"
                : "edge pair confidence is low";
            selection.selectedRowPixel = clampToIntPixel(
                0.5 * (leftCandidate.yPixel + rightCandidate.yPixel),
                0,
                std::numeric_limits<int>::max());
            selection.edgeContrastGrayValue =
                0.5 * (leftCandidate.contrastGrayValue
                       + rightCandidate.contrastGrayValue);
            selection.failureReason = selection.edgePair.valid
                ? ""
                : "edge pair confidence is low";
        }
    }

    if (bestScore < 0.0)
    {
        selection.failureReason = "gap width is out of range";
        selection.edgePair.message = selection.failureReason;
    }

    return selection;
}

const LaserCenterlinePoint *selectCenterlinePoint(
    const LaserCenterlineResult &centerline)
{
    const LaserCenterlinePoint *bestPoint = nullptr;
    for (const LaserCenterlinePoint &point : centerline.points)
    {
        if (!point.valid)
        {
            continue;
        }

        if (bestPoint == nullptr
            || point.confidence > bestPoint->confidence)
        {
            bestPoint = &point;
        }
    }
    return bestPoint;
}

} // namespace

WeldEdgePair CGapFeatureDetector::detectEdgesOnRow(
    const cv::Mat &gray8,
    double centerXPixel,
    double centerYPixel,
    const WeldFeatureDetectionOptions &options)
{
    WeldEdgePair edgePair;
    if (!isValidGrayImage(gray8))
    {
        edgePair.message = gray8.empty()
            ? "empty input"
            : "input must be CV_8UC1";
        return edgePair;
    }

    if (!isValidOptions(options))
    {
        edgePair.message = "invalid options";
        return edgePair;
    }

    std::vector<GapEdgeCandidate> leftCandidates;
    std::vector<GapEdgeCandidate> rightCandidates;
    collectEdgeCandidates(
        gray8,
        centerXPixel,
        centerYPixel,
        options,
        &leftCandidates,
        &rightCandidates);

    const EdgePairSelection selection = selectBestEdgePair(
        leftCandidates,
        rightCandidates,
        centerXPixel,
        centerYPixel,
        options);
    return selection.edgePair;
}

void CGapFeatureDetector::collectEdgeCandidates(
    const cv::Mat &gray8,
    double centerXPixel,
    double centerYPixel,
    const WeldFeatureDetectionOptions &options,
    std::vector<GapEdgeCandidate> *leftCandidates,
    std::vector<GapEdgeCandidate> *rightCandidates)
{
    if (leftCandidates != nullptr)
    {
        leftCandidates->clear();
    }
    if (rightCandidates != nullptr)
    {
        rightCandidates->clear();
    }

    if (!isValidGrayImage(gray8)
        || !isValidOptions(options)
        || leftCandidates == nullptr
        || rightCandidates == nullptr)
    {
        return;
    }

    const int centerX = clampToIntPixel(centerXPixel, 1, gray8.cols - 2);
    const std::vector<int> rows = collectCandidateRows(
        gray8,
        centerYPixel,
        options.searchHalfWindowY);
    if (rows.empty())
    {
        return;
    }

    int beginX = 1;
    int endX = gray8.cols - 2;
    if (options.edgeSearchHalfWidthPixel > 0)
    {
        beginX = std::max(1, centerX - options.edgeSearchHalfWidthPixel);
        endX = std::min(
            gray8.cols - 2,
            centerX + options.edgeSearchHalfWidthPixel);
    }

    for (int yPixel : rows)
    {
        for (int xPixel = beginX; xPixel <= endX; ++xPixel)
        {
            const double signedContrast = calculateSignedEdgeContrast(
                gray8,
                xPixel,
                yPixel,
                options.searchHalfWindowY);
            const double absContrast = std::fabs(signedContrast);
            if (absContrast < static_cast<double>(options.minEdgeContrast))
            {
                continue;
            }

            if (xPixel < centerX && signedContrast <= 0.0)
            {
                leftCandidates->push_back(makeEdgeCandidate(
                    static_cast<double>(xPixel),
                    static_cast<double>(yPixel),
                    absContrast,
                    centerYPixel,
                    options,
                    "left_edge_candidate"));
            }
            else if (xPixel > centerX && signedContrast >= 0.0)
            {
                rightCandidates->push_back(makeEdgeCandidate(
                    static_cast<double>(xPixel),
                    static_cast<double>(yPixel),
                    absContrast,
                    centerYPixel,
                    options,
                    "right_edge_candidate"));
            }
        }
    }
}

GapFeatureResult CGapFeatureDetector::detectGapFromCenterline(
    const cv::Mat &gray8,
    const LaserCenterlineResult &centerline,
    const WeldFeatureDetectionOptions &options)
{
    GapFeatureResult result;
    if (!isValidGrayImage(gray8))
    {
        result.message = gray8.empty()
            ? "empty input"
            : "input must be CV_8UC1";
        result.failureReason = result.message;
        return result;
    }

    if (!centerline.valid || centerline.points.empty())
    {
        result.message = "empty centerline";
        result.failureReason = result.message;
        return result;
    }

    if (!isValidOptions(options))
    {
        result.message = "invalid options";
        result.failureReason = result.message;
        return result;
    }

    const LaserCenterlinePoint *centerlinePoint =
        selectCenterlinePoint(centerline);
    if (centerlinePoint == nullptr)
    {
        result.message = "no valid centerline point";
        result.failureReason = result.message;
        return result;
    }

    std::vector<GapEdgeCandidate> leftCandidates;
    std::vector<GapEdgeCandidate> rightCandidates;
    collectEdgeCandidates(
        gray8,
        centerlinePoint->xPixel,
        centerlinePoint->yPixel,
        options,
        &leftCandidates,
        &rightCandidates);
    result.leftCandidateCount = static_cast<int>(leftCandidates.size());
    result.rightCandidateCount = static_cast<int>(rightCandidates.size());

    const EdgePairSelection selection = selectBestEdgePair(
        leftCandidates,
        rightCandidates,
        centerlinePoint->xPixel,
        centerlinePoint->yPixel,
        options);
    result.edgePair = selection.edgePair;
    result.selectedRowPixel = selection.selectedRowPixel;
    result.edgeContrastGrayValue = selection.edgeContrastGrayValue;
    result.failureReason = selection.failureReason;

    if (!result.edgePair.valid)
    {
        result.message = result.edgePair.message;
        if (result.failureReason.empty())
        {
            result.failureReason = result.message;
        }
        return result;
    }

    result.weldPoint = estimateWeldPointFromEdges(result.edgePair, options);
    result.gapWidthPixel = result.edgePair.widthPixel;
    result.confidence = clampRatio(
        0.65 * result.edgePair.confidence
        + 0.35 * result.weldPoint.confidence);
    result.valid = result.weldPoint.valid
        && result.confidence >= options.minConfidence;
    result.message = result.valid
        ? "gap feature detected"
        : "gap feature confidence is low";
    result.failureReason = result.valid ? "" : result.message;
    return result;
}

WeldFeaturePoint CGapFeatureDetector::estimateWeldPointFromEdges(
    const WeldEdgePair &edgePair,
    const WeldFeatureDetectionOptions &options)
{
    WeldFeaturePoint point;
    point.label = "weld_point";
    if (!edgePair.valid || edgePair.right.xPixel <= edgePair.left.xPixel)
    {
        return point;
    }

    const double widthScore = widthConfidence(edgePair.widthPixel, options);
    point.xPixel = 0.5 * (edgePair.left.xPixel + edgePair.right.xPixel);
    point.yPixel = 0.5 * (edgePair.left.yPixel + edgePair.right.yPixel);
    point.confidence = clampRatio(edgePair.confidence * widthScore);
    point.valid = point.confidence >= options.minConfidence;
    return point;
}

double CGapFeatureDetector::calculateEdgeContrast(
    const cv::Mat &gray8,
    int xPixel,
    int yPixel,
    int searchHalfWindowY)
{
    if (!isValidGrayImage(gray8)
        || xPixel <= 0
        || xPixel >= gray8.cols - 1
        || yPixel < 0
        || yPixel >= gray8.rows)
    {
        return 0.0;
    }

    const int beginY = std::max(0, yPixel - std::max(0, searchHalfWindowY));
    const int endY = std::min(
        gray8.rows - 1,
        yPixel + std::max(0, searchHalfWindowY));

    double contrastSum = 0.0;
    int sampleCount = 0;
    for (int sampleY = beginY; sampleY <= endY; ++sampleY)
    {
        const unsigned char leftValue =
            gray8.at<unsigned char>(sampleY, xPixel - 1);
        const unsigned char rightValue =
            gray8.at<unsigned char>(sampleY, xPixel + 1);
        contrastSum += std::fabs(
            static_cast<double>(rightValue)
            - static_cast<double>(leftValue));
        ++sampleCount;
    }

    if (sampleCount == 0)
    {
        return 0.0;
    }

    return contrastSum / static_cast<double>(sampleCount);
}

} // namespace i2r_rlc::vision

