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
 * @file        LaserLineResponseTools.cpp
 * @brief       激光线响应计算工具
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

#include "stripe/LaserLineResponseTools.h"

namespace i2r_rlc::vision {

namespace {

bool isSupportedGrayImage(const cv::Mat &grayImage)
{
    return !grayImage.empty() && grayImage.type() == CV_8UC1;
}

void setStatus(VisionPreprocessStatus *status, VisionPreprocessStatus value)
{
    if (status != nullptr)
    {
        *status = value;
    }
}

bool hasValidLaserWidth(
    const std::vector<int> &laserWidthByRow,
    int rowCount,
    int yPixel)
{
    return yPixel >= 0
        && yPixel < rowCount
        && static_cast<int>(laserWidthByRow.size()) >= rowCount
        && laserWidthByRow[yPixel] > 0;
}

int clampRow(int rowPixel, int rowCount)
{
    return std::max(0, std::min(rowPixel, rowCount));
}

} // namespace

double CLaserLineResponseTools::calculateLogResponseAt(
    const cv::Mat &grayImage,
    const std::vector<int> &laserWidthByRow,
    int xPixel,
    int yPixel,
    VisionPreprocessStatus *status)
{
    if (!isSupportedGrayImage(grayImage))
    {
        setStatus(
            status,
            grayImage.empty()
                ? VisionPreprocessStatus::EmptyInput
                : VisionPreprocessStatus::InvalidGrayImage);
        return 0.0;
    }
    if (xPixel < 0 || xPixel >= grayImage.cols
        || !hasValidLaserWidth(laserWidthByRow, grayImage.rows, yPixel))
    {
        setStatus(status, VisionPreprocessStatus::InvalidParameter);
        return 0.0;
    }

    const int laserWidth = laserWidthByRow[yPixel];
    const int filterLength = 3 * laserWidth;
    const int centerOffset = static_cast<int>(
        std::floor(static_cast<double>(filterLength) / 2.0));
    double response = 0.0;

    // 旧 LOGCore 的本质是列方向亮条纹响应：以当前行为中心取 3 倍激光
    // 宽度的窗口，中间 laserWidth 行代表激光主体，两侧代表背景抑制。
    // 因此权重固定为“中间 +1、两侧 -0.5”，最后除以宽度做归一化。
    for (int index = 0; index < filterLength; ++index)
    {
        const int offsetRow = index - centerOffset;
        const int sampleRow = yPixel + offsetRow;
        if (sampleRow < 0 || sampleRow >= grayImage.rows)
        {
            continue;
        }

        const bool isSideBand =
            index < laserWidth || index > 2 * laserWidth - 1;
        const double weight = isSideBand ? -0.5 : 1.0;
        response += weight * static_cast<double>(
            grayImage.at<unsigned char>(sampleRow, xPixel));
    }

    setStatus(status, VisionPreprocessStatus::Succeeded);
    return response / static_cast<double>(laserWidth);
}

LaserLogPeakResult CLaserLineResponseTools::detectLogPeaks(
    const cv::Mat &grayImage,
    const std::vector<int> &laserWidthByRow,
    const LaserLogPeakOptions &options,
    VisionPreprocessStatus *status)
{
    LaserLogPeakResult result;
    if (!isSupportedGrayImage(grayImage))
    {
        result.message = grayImage.empty()
            ? "empty input image"
            : "input image must be CV_8UC1";
        setStatus(
            status,
            grayImage.empty()
                ? VisionPreprocessStatus::EmptyInput
                : VisionPreprocessStatus::InvalidGrayImage);
        return result;
    }
    if (options.columnStepPixel <= 0
        || static_cast<int>(laserWidthByRow.size()) < grayImage.rows)
    {
        result.message = "invalid log peak options";
        setStatus(status, VisionPreprocessStatus::InvalidParameter);
        return result;
    }

    const int rowMin = clampRow(options.rowMinPixel, grayImage.rows);
    const int rowMax = clampRow(options.rowMaxPixel, grayImage.rows);
    if (rowMin >= rowMax)
    {
        result.message = "invalid row range";
        setStatus(status, VisionPreprocessStatus::InvalidRoi);
        return result;
    }

    result.maxRowByColumn.assign(grayImage.cols, -1);
    result.maxResponseByColumn.assign(grayImage.cols, 0.0);

    // 旧 LOG_new 不逐列输出全部候选，而是按 eff 间隔采样列；未采样列
    // 显式保留 -1，方便调用方沿用旧数组索引和列坐标关系。
    for (int x = 0; x < grayImage.cols - options.columnStepPixel;
         x += options.columnStepPixel)
    {
        int maxRow = -1;
        double maxResponse = 0.0;
        for (int y = rowMin; y < rowMax; ++y)
        {
            const int laserWidth = laserWidthByRow[y];
            if (laserWidth <= 0
                || y <= laserWidth
                || y >= grayImage.rows - laserWidth
                || grayImage.at<unsigned char>(y, x)
                    != static_cast<unsigned char>(
                        options.requiredPeakGrayValue))
            {
                continue;
            }

            VisionPreprocessStatus localStatus =
                VisionPreprocessStatus::Succeeded;
            const double response = calculateLogResponseAt(
                grayImage,
                laserWidthByRow,
                x,
                y,
                &localStatus);
            if (localStatus == VisionPreprocessStatus::Succeeded
                && response > maxResponse)
            {
                // 旧代码只有当最大响应超过阈值时才记录行号，但最大响应值
                // 仍持续更新；这样 replay 报告可以看到“未过阈值”的响应强度。
                if (response > options.logThreshold)
                {
                    maxRow = y + 1;
                }
                maxResponse = response;
            }
        }

        result.maxRowByColumn[x] = maxRow;
        result.maxResponseByColumn[x] = maxResponse;
        for (int offset = 1; offset < options.columnStepPixel
             && x + offset < grayImage.cols; ++offset)
        {
            result.maxRowByColumn[x + offset] = -1;
        }
    }

    result.valid = true;
    result.message = "log peaks detected";
    setStatus(status, VisionPreprocessStatus::Succeeded);
    return result;
}

LaserCenterOfMassResult CLaserLineResponseTools::refineCenterOfMass(
    const cv::Mat &grayImage,
    const std::vector<int> &peakRowsByColumn,
    const std::vector<int> &laserWidthByRow,
    int laserWidthMaxPixel,
    VisionPreprocessStatus *status)
{
    LaserCenterOfMassResult result;
    if (!isSupportedGrayImage(grayImage))
    {
        result.message = grayImage.empty()
            ? "empty input image"
            : "input image must be CV_8UC1";
        setStatus(
            status,
            grayImage.empty()
                ? VisionPreprocessStatus::EmptyInput
                : VisionPreprocessStatus::InvalidGrayImage);
        return result;
    }
    if (laserWidthMaxPixel <= 0
        || static_cast<int>(peakRowsByColumn.size()) < grayImage.cols
        || static_cast<int>(laserWidthByRow.size()) < grayImage.rows)
    {
        result.message = "invalid center of mass options";
        setStatus(status, VisionPreprocessStatus::InvalidParameter);
        return result;
    }

    result.refinedRowByColumn.resize(grayImage.cols, -1.0);
    result.originalRowByColumn.resize(grayImage.cols, -1.0);
    for (int x = 0; x < grayImage.cols; ++x)
    {
        int peakRow = peakRowsByColumn[x];
        result.refinedRowByColumn[x] = static_cast<double>(peakRow);
        result.originalRowByColumn[x] = static_cast<double>(peakRow);
        if (peakRow < 0
            || peakRow <= laserWidthMaxPixel
            || peakRow >= grayImage.rows - laserWidthMaxPixel)
        {
            continue;
        }

        int upSearchCount = 0;
        int downSearchCount = 0;
        // 旧 CoM_new 先统计峰值行上下连续亮像素数量，用该数量判断
        // 峰值是否偏向上侧或下侧，再决定灰度重心窗口的实际中心。
        for (int offset = 0; offset < laserWidthMaxPixel; ++offset)
        {
            if (peakRow + offset < grayImage.rows
                && grayImage.at<unsigned char>(peakRow + offset, x) > 230)
            {
                ++downSearchCount;
            }
            if (peakRow - offset >= 0
                && grayImage.at<unsigned char>(peakRow - offset, x) > 230)
            {
                ++upSearchCount;
            }
        }

        int centerOfMassWidth = upSearchCount + downSearchCount;
        const int baseLaserWidth = laserWidthByRow[peakRow];
        if (centerOfMassWidth > baseLaserWidth)
        {
            centerOfMassWidth = baseLaserWidth;
        }
        else
        {
            centerOfMassWidth = std::max(1, centerOfMassWidth);
            const int searchOffset =
                std::abs(upSearchCount - downSearchCount) / 2;
            peakRow += upSearchCount > downSearchCount
                ? -searchOffset
                : searchOffset;
        }

        const int halfWidth = static_cast<int>(
            std::floor(static_cast<double>(centerOfMassWidth) / 2.0));
        if (peakRow <= halfWidth
            || peakRow >= grayImage.rows - halfWidth)
        {
            continue;
        }

        double weightedSum = 0.0;
        double graySum = 0.0;
        // 旧实现使用行偏移作为权重，计算列方向灰度重心偏移；这里保留
        // 同样的正负权重含义，并在 graySum 为 0 时固定为无偏移。
        for (int index = 0; index < centerOfMassWidth; ++index)
        {
            const int weight = halfWidth - index;
            const int sampleRow = peakRow + weight;
            if (sampleRow < 0 || sampleRow >= grayImage.rows)
            {
                continue;
            }

            const double grayValue = static_cast<double>(
                grayImage.at<unsigned char>(sampleRow, x));
            graySum += grayValue;
            if (index != halfWidth)
            {
                weightedSum += static_cast<double>(weight) * grayValue;
            }
        }

        const double delta = graySum != 0.0 ? weightedSum / graySum : 0.0;
        result.refinedRowByColumn[x] = static_cast<double>(peakRow) + delta;
        result.originalRowByColumn[x] = result.refinedRowByColumn[x];
    }

    result.valid = true;
    result.message = "center of mass refined";
    setStatus(status, VisionPreprocessStatus::Succeeded);
    return result;
}

LaserDraggedPointResult CLaserLineResponseTools::dragPointToLaserLine(
    const cv::Mat &grayImage,
    const std::vector<int> &laserWidthByRow,
    const cv::Point2i &startPoint,
    int searchHalfHeightPixel,
    int minGrayValue,
    VisionPreprocessStatus *status)
{
    LaserDraggedPointResult result;
    result.point = cv::Point2d(startPoint.x, startPoint.y);
    result.maxResponse = -10000.0;
    if (!isSupportedGrayImage(grayImage))
    {
        result.message = grayImage.empty()
            ? "empty input image"
            : "input image must be CV_8UC1";
        setStatus(
            status,
            grayImage.empty()
                ? VisionPreprocessStatus::EmptyInput
                : VisionPreprocessStatus::InvalidGrayImage);
        return result;
    }
    if (searchHalfHeightPixel <= 0
        || startPoint.x < 0
        || startPoint.x >= grayImage.cols
        || startPoint.y < 0
        || startPoint.y >= grayImage.rows
        || static_cast<int>(laserWidthByRow.size()) < grayImage.rows)
    {
        result.message = "invalid drag point options";
        setStatus(status, VisionPreprocessStatus::InvalidParameter);
        return result;
    }

    int bestOffset = 0;
    if (startPoint.y > searchHalfHeightPixel
        && startPoint.y < grayImage.rows - searchHalfHeightPixel)
    {
        // DragOfflinePt 只在同一列上下搜索，不改变 x。每个候选点必须先
        // 通过亮度阈值，再用 LOGCore 响应比较，最终选择响应最大的行。
        for (int offset = 1; offset <= searchHalfHeightPixel; ++offset)
        {
            const int downRow = startPoint.y + offset;
            if (grayImage.at<unsigned char>(downRow, startPoint.x)
                >= minGrayValue)
            {
                VisionPreprocessStatus localStatus =
                    VisionPreprocessStatus::Succeeded;
                const double response = calculateLogResponseAt(
                    grayImage,
                    laserWidthByRow,
                    startPoint.x,
                    downRow,
                    &localStatus);
                if (localStatus == VisionPreprocessStatus::Succeeded
                    && response > result.maxResponse)
                {
                    result.maxResponse = response;
                    bestOffset = offset;
                }
            }

            const int upRow = startPoint.y - offset;
            if (grayImage.at<unsigned char>(upRow, startPoint.x)
                >= minGrayValue)
            {
                VisionPreprocessStatus localStatus =
                    VisionPreprocessStatus::Succeeded;
                const double response = calculateLogResponseAt(
                    grayImage,
                    laserWidthByRow,
                    startPoint.x,
                    upRow,
                    &localStatus);
                if (localStatus == VisionPreprocessStatus::Succeeded
                    && response > result.maxResponse)
                {
                    result.maxResponse = response;
                    bestOffset = -offset;
                }
            }
        }
    }

    result.point.y = static_cast<double>(startPoint.y + bestOffset);
    result.valid = true;
    result.message = "point dragged to laser line";
    setStatus(status, VisionPreprocessStatus::Succeeded);
    return result;
}

LaserNeighborMask CLaserLineResponseTools::buildEfficientMask(
    int offsetXPixel,
    int offsetYPixel)
{
    LaserNeighborMask mask;
    const auto appendOffset = [&mask](int x, int y) {
        mask.offsets.push_back(cv::Point(x, y));
    };

    // EfficientMask 是旧 BSnakeGrow 的方向表：不同增长方向只检查该方向
    // 前方的 3 个或 5 个邻域点，中心方向则返回完整 3x3 邻域。
    if (offsetXPixel == 0 && offsetYPixel == 0)
    {
        for (int x = -1; x <= 1; ++x)
        {
            for (int y = -1; y <= 1; ++y)
            {
                appendOffset(x, y);
            }
        }
    }
    else if (offsetXPixel == -1 && offsetYPixel == 0)
    {
        appendOffset(-1, -1);
        appendOffset(-1, 0);
        appendOffset(-1, 1);
    }
    else if (offsetXPixel == 1 && offsetYPixel == 0)
    {
        appendOffset(1, -1);
        appendOffset(1, 0);
        appendOffset(1, 1);
    }
    else if (offsetXPixel == 0 && offsetYPixel == -1)
    {
        appendOffset(-1, -1);
        appendOffset(0, -1);
        appendOffset(1, -1);
    }
    else if (offsetXPixel == 0 && offsetYPixel == 1)
    {
        appendOffset(-1, 1);
        appendOffset(0, 1);
        appendOffset(1, 1);
    }
    else if (offsetXPixel == -1 && offsetYPixel == -1)
    {
        appendOffset(-1, -1);
        appendOffset(0, -1);
        appendOffset(1, -1);
        appendOffset(-1, 0);
        appendOffset(-1, 1);
    }
    else if (offsetXPixel == -1 && offsetYPixel == 1)
    {
        appendOffset(-1, -1);
        appendOffset(-1, 0);
        appendOffset(-1, 1);
        appendOffset(0, 1);
        appendOffset(1, 1);
    }
    else if (offsetXPixel == 1 && offsetYPixel == 1)
    {
        appendOffset(1, -1);
        appendOffset(1, 0);
        appendOffset(1, 1);
        appendOffset(-1, 1);
        appendOffset(0, 1);
    }
    else if (offsetXPixel == 1 && offsetYPixel == -1)
    {
        appendOffset(1, -1);
        appendOffset(1, 0);
        appendOffset(1, 1);
        appendOffset(-1, -1);
        appendOffset(0, -1);
    }

    mask.valid = !mask.offsets.empty();
    return mask;
}

} // namespace i2r_rlc::vision
