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
 * @file        ActiveContourStripeExtractor.cpp
 * @brief       主动轮廓激光条纹提取实现
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
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>

#include <opencv2/imgproc.hpp>

#include "feature/ContourControlPointTools.h"
#include "stripe/ActiveContourStripeExtractor.h"
#include "stripe/LaserLineResponseTools.h"
#include "stripe/WeldSkeletonFeatureTools.h"

namespace i2r_rlc::vision {

namespace {

constexpr double kEpsilon = 1.0e-6;
constexpr int kSourceInitialLogColumnStepPixel = 5;
constexpr int kSourceInitialLogWidthPixel = 8;
constexpr int kSourceInitialComWidthPixel = 12;
constexpr int kSourceInitialComMaxWidthPixel = 16;

/**
 * @brief 骨架连通段及其筛选评分。
 * @details
 * 该结构只在主动轮廓初始化内部使用，用来替代旧工程中基于 HALCON
 * region/XLD 的骨架段处理。坐标单位均为 pixel，`score` 为无量纲
 * 内部排序分数，不作为 public confidence 输出。
 */
struct SkeletonComponent {
    /// @brief 连通段中的骨架像素集合，坐标单位为 pixel。
    std::vector<cv::Point> pixels;

    /// @brief 连通段覆盖的最小 x 坐标，单位为 pixel。
    int minX = 0;

    /// @brief 连通段覆盖的最大 x 坐标，单位为 pixel。
    int maxX = 0;

    /// @brief 连通段筛选分数，无量纲，仅用于内部排序。
    double score = 0.0;
};

/**
 * @brief 将比例或置信度限制到 0~1 范围。
 * @param value 输入比例或置信度，无量纲 ratio。
 * @return 限制到 `[0, 1]` 后的值。
 * @note 用于统一 confidence、gapWidth ratio 等归一化结果的边界。
 */
double clampRatio(double value)
{
    return std::max(0.0, std::min(1.0, value));
}

/**
 * @brief 检查主动轮廓算法输入是否为非空 CV_8UC1 灰度图。
 * @param gray8 输入图像。
 * @return true 表示图像非空且类型为 `CV_8UC1`。
 * @note 主动轮廓提取不在本层执行 UI 图像、彩色图或相机帧格式转换。
 */
bool isValidGrayImage(const cv::Mat &gray8)
{
    return !gray8.empty() && gray8.type() == CV_8UC1;
}

/**
 * @brief 判断整数像素坐标是否位于图像范围内。
 * @param gray8 用于提供宽高边界的输入图像。
 * @param xPixel x 坐标，单位为 pixel。
 * @param yPixel y 坐标，单位为 pixel。
 * @return true 表示坐标位于 `[0, cols)` 与 `[0, rows)` 范围内。
 */
bool isInsideImage(const cv::Mat &gray8, int xPixel, int yPixel)
{
    return xPixel >= 0
        && xPixel < gray8.cols
        && yPixel >= 0
        && yPixel < gray8.rows;
}

/**
 * @brief 安全读取灰度值，越界时返回 0 作为暗背景。
 * @param gray8 输入灰度图，必须为 `CV_8UC1`。
 * @param xPixel x 坐标，单位为 pixel。
 * @param yPixel y 坐标，单位为 pixel。
 * @return 像素灰度值，范围为 0~255；越界时返回 0。
 * @note gap 扫描和邻域检测将越界区域视为暗背景，减少调用侧重复边界判断。
 */
int grayAt(const cv::Mat &gray8, int xPixel, int yPixel)
{
    if (!isInsideImage(gray8, xPixel, yPixel))
    {
        return 0;
    }

    return gray8.at<unsigned char>(yPixel, xPixel);
}

/**
 * @brief 按 x/y 排序并删除完全重复的曲线点。
 * @param points 输入曲线点，坐标单位为 pixel。
 * @return 按 x/y 升序排序且去除完全重复坐标后的点列。
 * @note 该排序保证 ROI 选点、gap 扫描和线段拟合使用稳定的曲线顺序。
 */
std::vector<cv::Point2d> sortedUniquePoints(
    const std::vector<cv::Point2d> &points)
{
    std::vector<cv::Point2d> sortedPoints = points;
    std::sort(
        sortedPoints.begin(),
        sortedPoints.end(),
        [](const cv::Point2d &left, const cv::Point2d &right) {
            if (std::fabs(left.x - right.x) <= 1.0e-6)
            {
                return left.y < right.y;
            }
            return left.x < right.x;
        });
    sortedPoints.erase(
        std::unique(
            sortedPoints.begin(),
            sortedPoints.end(),
            [](const cv::Point2d &left, const cv::Point2d &right) {
                return std::fabs(left.x - right.x) <= 1.0e-6
                    && std::fabs(left.y - right.y) <= 1.0e-6;
            }),
        sortedPoints.end());
    return sortedPoints;
}

/**
 * @brief 填充二值区域中的内部孔洞。
 * @param binaryMask 输入二值 mask，类型应为 `CV_8UC1`，非零像素表示前景。
 * @return 填孔后的二值 mask；输入为空时返回空 `cv::Mat`。
 * @note 对应旧初始化流程中的区域填充语义，用于让下游 closing 和骨架化获得连续区域。
 * @warning 该函数假定输入为单通道二值图，非二值灰度图会按非零前景参与位运算。
 */
cv::Mat fillHoles(const cv::Mat &binaryMask)
{
    // 1. 空输入保护：初始化主路径失败时由上层 fallback 接管。
    if (binaryMask.empty())
    {
        return cv::Mat();
    }

    // 2. 从扩展边框外部 flood fill 背景，再取反得到内部孔洞区域。
    cv::Mat padded;
    cv::copyMakeBorder(
        binaryMask,
        padded,
        1,
        1,
        1,
        1,
        cv::BORDER_CONSTANT,
        cv::Scalar(0));

    cv::Mat floodMask = padded.clone();
    cv::floodFill(floodMask, cv::Point(0, 0), cv::Scalar(255));
    cv::Mat inverseFloodMask;
    cv::bitwise_not(floodMask, inverseFloodMask);

    cv::Mat holes = inverseFloodMask(
        cv::Rect(1, 1, binaryMask.cols, binaryMask.rows));

    cv::Mat filled;
    // 3. 合并孔洞：保留原前景，并把被背景包围的暗洞填成前景。
    cv::bitwise_or(binaryMask, holes, filled);
    return filled;
}

/**
 * @brief 收集骨架 mask 中的 8 邻域连通段。
 * @param skeletonMask 输入骨架二值 mask，类型应为 `CV_8UC1`。
 * @return 骨架连通段集合；空 mask 或无前景时返回空集合。
 * @note 每个连通段记录所有 pixel 点及 x 范围，供分叉剪枝和重叠段筛选使用。
 * @warning 输入 mask 非二值时，OpenCV 会把非零像素作为前景参与连通域标记。
 */
std::vector<SkeletonComponent> collectSkeletonComponents(
    const cv::Mat &skeletonMask)
{
    std::vector<SkeletonComponent> components;
    // 1. 无骨架时直接返回空集合，避免 connectedComponents 产生无意义背景标签。
    if (skeletonMask.empty() || cv::countNonZero(skeletonMask) == 0)
    {
        return components;
    }

    // 2. 连通域标记：label 0 是背景，其他 label 映射到 SkeletonComponent。
    cv::Mat labels;
    const int componentCount =
        cv::connectedComponents(skeletonMask, labels, 8, CV_32S);
    components.resize(static_cast<std::size_t>(std::max(0, componentCount - 1)));
    for (SkeletonComponent &component : components)
    {
        component.minX = skeletonMask.cols;
        component.maxX = -1;
    }

    // 3. 汇总像素和 x 范围：x 范围下游用于判断浮空段/重叠段。
    for (int yPixel = 0; yPixel < skeletonMask.rows; ++yPixel)
    {
        for (int xPixel = 0; xPixel < skeletonMask.cols; ++xPixel)
        {
            const int label = labels.at<int>(yPixel, xPixel);
            if (label <= 0)
            {
                continue;
            }

            SkeletonComponent &component =
                components[static_cast<std::size_t>(label - 1)];
            component.pixels.push_back(cv::Point(xPixel, yPixel));
            component.minX = std::min(component.minX, xPixel);
            component.maxX = std::max(component.maxX, xPixel);
        }
    }

    // 4. 清理空段：防御异常 label 或无前景场景，保证返回段都有像素。
    components.erase(
        std::remove_if(
            components.begin(),
            components.end(),
            [](const SkeletonComponent &component) {
                return component.pixels.empty();
            }),
        components.end());
    return components;
}

/**
 * @brief 根据骨架连通段集合生成二值 mask。
 * @param size 输出 mask 尺寸，单位为 pixel。
 * @param components 要绘制的骨架连通段集合。
 * @return `CV_8UC1` 二值 mask，连通段像素位置为 255。
 * @note 该函数用于把筛选后的 component 集合转换回 OpenCV mask，继续参与下游处理。
 * @warning 调用方需保证 component 像素坐标位于 `size` 范围内。
 */
cv::Mat maskFromComponents(
    const cv::Size &size,
    const std::vector<SkeletonComponent> &components)
{
    cv::Mat mask = cv::Mat::zeros(size, CV_8UC1);
    // 将筛选后的离散骨架点写回 mask，保持像素坐标单位不变。
    for (const SkeletonComponent &component : components)
    {
        for (const cv::Point &point : component.pixels)
        {
            mask.at<unsigned char>(point.y, point.x) = 255;
        }
    }
    return mask;
}

/**
 * @brief 统计骨架点 8 邻域内的前景邻居数量。
 * @param skeletonMask 输入骨架二值 mask，类型应为 `CV_8UC1`。
 * @param xPixel 待检查骨架点 x 坐标，单位为 pixel。
 * @param yPixel 待检查骨架点 y 坐标，单位为 pixel。
 * @return 8 邻域内非零骨架像素数量。
 * @note 邻居数大于等于 3 的点被视为分叉点，用于剪掉多余分支。
 */
int skeletonNeighborCount(const cv::Mat &skeletonMask, int xPixel, int yPixel)
{
    int count = 0;
    // 只统计 8 邻域，不包含中心点；越界邻居按背景处理。
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
            if (x < 0 || y < 0
                || x >= skeletonMask.cols || y >= skeletonMask.rows)
            {
                continue;
            }
            if (skeletonMask.at<unsigned char>(y, x) > 0)
            {
                ++count;
            }
        }
    }
    return count;
}

/**
 * @brief 剪除骨架分叉处的弱分支。
 * @param gray8 输入灰度图，必须为 `CV_8UC1`。
 * @param skeletonMask 输入骨架二值 mask，类型应为 `CV_8UC1`。
 * @return 剪枝后的骨架 mask，类型为 `CV_8UC1`。
 * @note 对应旧初始化中对骨架/XLD 分支的整理：遇到分叉时保留评分最高的两个分支，
 * 减少焊缝区域交叉亮斑或闭合区域产生的多余骨架。
 * @warning 若输入骨架为空或没有连通段，返回全黑 mask。
 */
cv::Mat pruneJunctionBranches(const cv::Mat &gray8, const cv::Mat &skeletonMask)
{
    const std::vector<SkeletonComponent> components =
        collectSkeletonComponents(skeletonMask);
    cv::Mat prunedMask = cv::Mat::zeros(skeletonMask.size(), CV_8UC1);
    // 1. 逐个骨架连通段处理；无分叉的段原样保留。
    for (const SkeletonComponent &component : components)
    {
        cv::Mat componentMask = cv::Mat::zeros(skeletonMask.size(), CV_8UC1);
        std::vector<cv::Point> junctionPoints;
        for (const cv::Point &point : component.pixels)
        {
            componentMask.at<unsigned char>(point.y, point.x) = 255;
            if (skeletonNeighborCount(skeletonMask, point.x, point.y) >= 3)
            {
                junctionPoints.push_back(point);
            }
        }

        if (junctionPoints.empty())
        {
            prunedMask.setTo(255, componentMask);
            continue;
        }

        // 2. 临时移除分叉点，让分叉两侧被拆成独立 segment 便于评分。
        cv::Mat splitMask = componentMask.clone();
        for (const cv::Point &point : junctionPoints)
        {
            splitMask.at<unsigned char>(point.y, point.x) = 0;
        }

        std::vector<SkeletonComponent> segments =
            collectSkeletonComponents(splitMask);
        if (segments.size() <= 2U)
        {
            prunedMask.setTo(255, componentMask);
            continue;
        }

        // 3. 分支评分：strength 表示该段落在原图中饱和亮点占比，
        // 加入极小长度项用于同等亮度时偏向更长的主分支。
        for (SkeletonComponent &segment : segments)
        {
            int brightCount = 0;
            for (const cv::Point &point : segment.pixels)
            {
                if (grayAt(gray8, point.x, point.y) == 255)
                {
                    ++brightCount;
                }
            }
            const double strength = segment.pixels.empty()
                ? 0.0
                : static_cast<double>(brightCount)
                    / static_cast<double>(segment.pixels.size());
            segment.score = strength
                + 0.001 * static_cast<double>(segment.pixels.size());
        }

        // 4. 保留最强两个分支并把分叉点补回，避免主曲线在交汇处断裂。
        std::sort(
            segments.begin(),
            segments.end(),
            [](const SkeletonComponent &left,
               const SkeletonComponent &right) {
                return left.score > right.score;
            });
        for (std::size_t index = 0U; index < 2U && index < segments.size();
             ++index)
        {
            for (const cv::Point &point : segments[index].pixels)
            {
                prunedMask.at<unsigned char>(point.y, point.x) = 255;
            }
        }
        for (const cv::Point &point : junctionPoints)
        {
            prunedMask.at<unsigned char>(point.y, point.x) = 255;
        }
    }
    return prunedMask;
}

/**
 * @brief 统计 LOG 峰值落入每个填充连通域的次数。
 * @param filledMask 填孔后的二值区域 mask，类型应为 `CV_8UC1`。
 * @param logResult LOG 峰值检测结果，`maxRowByColumn` 坐标单位为 pixel。
 * @return 按连通域 label 存储的 LOG 命中次数；下标 0 对应背景。
 * @note 该频次用于衡量哪个填充区域更符合旧 LOG_new 初始化得到的激光中心候选。
 * @warning `logResult.valid=false` 时返回全 0 频次数组。
 */
std::vector<int> countLogFrequencyByFilledComponent(
    const cv::Mat &filledMask,
    const LaserLogPeakResult &logResult)
{
    // 1. 对填充区域重新标号，频次数组大小与 label 数量一致。
    cv::Mat labels;
    const int componentCount =
        cv::connectedComponents(filledMask, labels, 8, CV_32S);
    std::vector<int> frequency(
        static_cast<std::size_t>(std::max(0, componentCount)),
        0);
    if (!logResult.valid)
    {
        return frequency;
    }

    // 2. 按旧初始化的列步长采样 LOG 峰值，统计峰值落入的填充连通域。
    for (int xPixel = 0;
         xPixel < static_cast<int>(logResult.maxRowByColumn.size());
         xPixel += kSourceInitialLogColumnStepPixel)
    {
        const int yPixel = logResult.maxRowByColumn[xPixel];
        if (yPixel < 0 || yPixel >= labels.rows || xPixel >= labels.cols)
        {
            continue;
        }

        const int label = labels.at<int>(yPixel, xPixel);
        if (label > 0 && label < static_cast<int>(frequency.size()))
        {
            ++frequency[static_cast<std::size_t>(label)];
        }
    }
    return frequency;
}

/**
 * @brief 将填充连通域 LOG 命中频次写入骨架段评分。
 * @param filledMask 填孔后的二值区域 mask，类型应为 `CV_8UC1`。
 * @param frequencyByLabel 每个填充连通域的 LOG 命中次数，下标 0 为背景。
 * @param components 待写入评分的骨架段集合。
 * @note 每个骨架段先找到其像素覆盖最多的填充连通域，再使用该连通域 LOG 频次作为 score。
 * @warning `components=nullptr`、mask 为空或频次数组为空时直接返回。
 */
void applyFilledComponentFrequencyScore(
    const cv::Mat &filledMask,
    const std::vector<int> &frequencyByLabel,
    std::vector<SkeletonComponent> *components)
{
    if (components == nullptr || filledMask.empty() || frequencyByLabel.empty())
    {
        return;
    }

    // 1. 重新标号填充区域，让每个骨架点可以映射回所在填充 component。
    cv::Mat labels;
    cv::connectedComponents(filledMask, labels, 8, CV_32S);
    for (SkeletonComponent &component : *components)
    {
        // 2. 统计当前骨架段覆盖到的填充 label，选择重叠最多的 label 作为归属。
        std::vector<int> touchedLabelCount(frequencyByLabel.size(), 0);
        for (const cv::Point &point : component.pixels)
        {
            const int label = labels.at<int>(point.y, point.x);
            if (label > 0 && label < static_cast<int>(touchedLabelCount.size()))
            {
                ++touchedLabelCount[static_cast<std::size_t>(label)];
            }
        }

        int bestLabel = 0;
        for (std::size_t label = 1U; label < touchedLabelCount.size(); ++label)
        {
            if (touchedLabelCount[label]
                > touchedLabelCount[static_cast<std::size_t>(bestLabel)])
            {
                bestLabel = static_cast<int>(label);
            }
        }

        // 3. score 表示该骨架段所在区域被 LOG 峰值命中的次数；
        // 没有有效归属时退回长度评分，避免所有分数为 0。
        component.score = bestLabel > 0
            ? static_cast<double>(
                frequencyByLabel[static_cast<std::size_t>(bestLabel)])
            : static_cast<double>(component.pixels.size());
    }
}

/**
 * @brief 判断两个骨架段的 x 范围是否重叠。
 * @param left 左侧候选骨架段，x 范围单位为 pixel。
 * @param right 右侧候选骨架段，x 范围单位为 pixel。
 * @return true 表示两个段在 x 轴投影存在重叠或包含关系。
 * @note 该判断用于把同一列范围竞争的浮空段归为一组，只保留最佳段。
 */
bool rangesOverlap(const SkeletonComponent &left, const SkeletonComponent &right)
{
    return (right.minX >= left.minX && right.minX <= left.maxX)
        || (right.maxX >= left.minX && right.maxX <= left.maxX)
        || (right.minX <= left.minX && right.maxX >= left.maxX);
}

/**
 * @brief 在 x 范围重叠的骨架段组内选择最高分段。
 * @param components 输入骨架段集合。
 * @param useLengthScore true 表示用段长度评分，false 表示用 `score` 字段评分。
 * @return 每个重叠组中选出的最佳骨架段集合。
 * @note 该函数用于剔除浮空段：先按 x 范围聚合互相重叠的段，再保留 LOG 频次或长度更优者。
 * @warning 若多个段得分相同，会选择像素数量更多的段。
 */
std::vector<SkeletonComponent> selectOverlappingComponentsByScore(
    const std::vector<SkeletonComponent> &components,
    bool useLengthScore)
{
    std::vector<SkeletonComponent> selectedComponents;
    std::vector<unsigned char> consumed(components.size(), 0);
    // 1. 通过 x 范围重叠关系构造连通组，表示彼此竞争同一横向区域的骨架段。
    for (std::size_t index = 0U; index < components.size(); ++index)
    {
        if (consumed[index] != 0)
        {
            continue;
        }

        std::vector<std::size_t> group;
        group.push_back(index);
        consumed[index] = 1;
        bool changed = true;
        while (changed)
        {
            changed = false;
            for (std::size_t candidate = 0U; candidate < components.size();
                 ++candidate)
            {
                if (consumed[candidate] != 0)
                {
                    continue;
                }

                bool overlap = false;
                for (std::size_t member : group)
                {
                    if (rangesOverlap(components[member], components[candidate]))
                    {
                        overlap = true;
                        break;
                    }
                }
                if (overlap)
                {
                    group.push_back(candidate);
                    consumed[candidate] = 1;
                    changed = true;
                }
            }
        }

        // 2. 组内选择：可按 LOG 频次 score，也可按像素长度。
        std::size_t bestIndex = group.front();
        for (std::size_t member : group)
        {
            const double memberScore = useLengthScore
                ? static_cast<double>(components[member].pixels.size())
                : components[member].score;
            const double bestScore = useLengthScore
                ? static_cast<double>(components[bestIndex].pixels.size())
                : components[bestIndex].score;
            if (memberScore > bestScore
                || (std::fabs(memberScore - bestScore) <= kEpsilon
                    && components[member].pixels.size()
                        > components[bestIndex].pixels.size()))
            {
                bestIndex = member;
            }
        }
        selectedComponents.push_back(components[bestIndex]);
    }
    return selectedComponents;
}

/**
 * @brief 由骨架 mask 计算每一列的中心行。
 * @param skeletonMask 输入骨架二值 mask，类型应为 `CV_8UC1`。
 * @return 每列中心行数组，单位为 pixel；该列无骨架点时为 -1。
 * @note 多个骨架点落在同一列时使用 y 平均值，提供给下游 CoM 亚像素细化。
 */
std::vector<int> makeColumnCenterRows(const cv::Mat &skeletonMask)
{
    // 1. 按列累计骨架 y 坐标，rowCount 表示该列有效骨架点数量。
    std::vector<double> rowSum(static_cast<std::size_t>(skeletonMask.cols), 0.0);
    std::vector<int> rowCount(static_cast<std::size_t>(skeletonMask.cols), 0);
    for (int yPixel = 0; yPixel < skeletonMask.rows; ++yPixel)
    {
        for (int xPixel = 0; xPixel < skeletonMask.cols; ++xPixel)
        {
            if (skeletonMask.at<unsigned char>(yPixel, xPixel) == 0)
            {
                continue;
            }
            rowSum[static_cast<std::size_t>(xPixel)] +=
                static_cast<double>(yPixel);
            ++rowCount[static_cast<std::size_t>(xPixel)];
        }
    }

    // 2. 输出整数中心行；无骨架列保持 -1，表示下游质心修正应跳过。
    std::vector<int> centerRows(static_cast<std::size_t>(skeletonMask.cols), -1);
    for (int xPixel = 0; xPixel < skeletonMask.cols; ++xPixel)
    {
        const int count = rowCount[static_cast<std::size_t>(xPixel)];
        if (count > 0)
        {
            centerRows[static_cast<std::size_t>(xPixel)] =
                static_cast<int>(std::lround(
                    rowSum[static_cast<std::size_t>(xPixel)]
                    / static_cast<double>(count)));
        }
    }
    return centerRows;
}

/**
 * @brief 根据每列中心行生成初始控制点。
 * @param centerRows 每列中心行，单位为 pixel；负数表示该列无中心。
 * @param sampleStepPixel 控制点列采样步长，单位为 pixel。
 * @return 初始控制点集合，坐标单位为 pixel，并按 x 升序去重。
 * @note 若采样列无中心，会在半个采样步长内寻找最近有效列，减少断裂造成的控制点缺失。
 * @warning `sampleStepPixel<=0` 时内部退化为 1 pixel 步长。
 */
std::vector<cv::Point2d> makeInitialControlPointsFromCenterRows(
    const std::vector<double> &centerRows,
    int sampleStepPixel)
{
    std::vector<cv::Point2d> controlPoints;
    const int step = std::max(1, sampleStepPixel);
    // 1. 按采样步长取列中心；无效列在附近搜索最近有效列作为替代。
    for (int xPixel = 0; xPixel < static_cast<int>(centerRows.size());
         xPixel += step)
    {
        int sampleXPixel = xPixel;
        double yPixel = centerRows[static_cast<std::size_t>(xPixel)];
        if (yPixel < 0.0)
        {
            const int searchRadius = std::max(1, step / 2);
            for (int offset = 1; offset <= searchRadius; ++offset)
            {
                const int leftX = xPixel - offset;
                const int rightX = xPixel + offset;
                if (leftX >= 0
                    && centerRows[static_cast<std::size_t>(leftX)] >= 0.0)
                {
                    sampleXPixel = leftX;
                    yPixel = centerRows[static_cast<std::size_t>(leftX)];
                    break;
                }
                if (rightX < static_cast<int>(centerRows.size())
                    && centerRows[static_cast<std::size_t>(rightX)] >= 0.0)
                {
                    sampleXPixel = rightX;
                    yPixel = centerRows[static_cast<std::size_t>(rightX)];
                    break;
                }
            }
        }
        if (yPixel >= 0.0)
        {
            controlPoints.push_back(cv::Point2d(
                static_cast<double>(sampleXPixel),
                yPixel));
        }
    }
    return sortedUniquePoints(controlPoints);
}

/**
 * @brief 构建主动轮廓初始化主路径的控制点。
 * @param gray8 输入灰度图，必须为 `CV_8UC1`。
 * @param options 初始阈值、closing 半径和控制点采样步长参数。
 * @return 初始控制点集合，坐标单位为 pixel；任一关键步骤失败时返回空集合。
 * @note 该函数流程级复刻旧 `InitBSnake_new`：阈值区域、填孔、closing、骨架化、
 * 分叉剪枝、LOG 频次筛选、浮空段筛选、列中心和 CoM 亚像素修正。
 * @warning 返回空集合会触发 `initializeCurve` 的 skeleton/mask fallback 初始化。
 */
std::vector<cv::Point2d> buildInitialControlPoints(
    const cv::Mat &gray8,
    const ActiveContourStripeOptions &options)
{
    // 1. 先走公共激光骨架流程：与 ImageProFeng 的 SkeletonExtraction 迁移
    //    共用阈值、填充、closing、细化、筛选和 CoM 中心线提取。
    LaserSkeletonExtractionOptions skeletonOptions;
    skeletonOptions.thresholdValue = options.initialThresholdGrayValue;
    skeletonOptions.minComponentArea = 3;
    skeletonOptions.closingRadiusPixel = std::max(
        1,
        options.initialClosingRadiusPixel);
    skeletonOptions.logColumnStepPixel = kSourceInitialLogColumnStepPixel;
    skeletonOptions.laserWidthPixel = kSourceInitialComWidthPixel;
    skeletonOptions.laserWidthMaxPixel = kSourceInitialComMaxWidthPixel;
    skeletonOptions.enableHoughFallback = false;
    const WeldSkeletonFeatureResult sharedSkeleton =
        CWeldSkeletonFeatureTools::extractLaserSkeleton(gray8, skeletonOptions);
    const std::vector<cv::Point2d> &sharedCenterPoints =
        !sharedSkeleton.refinedCenterPoints.empty()
            ? sharedSkeleton.refinedCenterPoints
            : sharedSkeleton.centerLinePoints;
    if (sharedSkeleton.valid && sharedCenterPoints.size() >= 2U)
    {
        std::vector<double> centerRows(
            static_cast<std::size_t>(gray8.cols),
            -1.0);
        std::vector<double> rowSum(
            static_cast<std::size_t>(gray8.cols),
            0.0);
        std::vector<int> rowCount(
            static_cast<std::size_t>(gray8.cols),
            0);
        for (const cv::Point2d &point : sharedCenterPoints)
        {
            const int xPixel = static_cast<int>(std::round(point.x));
            if (xPixel < 0 || xPixel >= gray8.cols)
            {
                continue;
            }
            rowSum[static_cast<std::size_t>(xPixel)] += point.y;
            ++rowCount[static_cast<std::size_t>(xPixel)];
        }
        for (int xPixel = 0; xPixel < gray8.cols; ++xPixel)
        {
            const int count = rowCount[static_cast<std::size_t>(xPixel)];
            if (count > 0)
            {
                centerRows[static_cast<std::size_t>(xPixel)] =
                    rowSum[static_cast<std::size_t>(xPixel)]
                    / static_cast<double>(count);
            }
        }

        const std::vector<cv::Point2d> sharedControlPoints =
            makeInitialControlPointsFromCenterRows(
                centerRows,
                options.initialControlPointStepPixel);
        const double sharedSpanPixel = sharedControlPoints.size() >= 2U
            ? sharedControlPoints.back().x - sharedControlPoints.front().x
            : 0.0;
        if (sharedControlPoints.size() >= 2U
            && sharedSpanPixel >= 0.60 * static_cast<double>(gray8.cols))
        {
            return sharedControlPoints;
        }
    }

    // 2. 兼容 fallback：公共流程不满足控制点数量时，保留原主动轮廓
    //    初始化细节，避免旧测试和低对比图像初始化被意外削弱。
    // 2.1 区域预处理：亮像素阈值、填孔和 closing 对应旧 HALCON region 整理步骤。
    cv::Mat brightPixels;
    cv::threshold(
        gray8,
        brightPixels,
        static_cast<double>(options.initialThresholdGrayValue),
        255.0,
        cv::THRESH_BINARY);

    const cv::Mat particlesFillUp = fillHoles(brightPixels);
    cv::Mat regionClosing;
    const int kernelRadius = std::max(1, options.initialClosingRadiusPixel);
    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(2 * kernelRadius + 1, 2 * kernelRadius + 1));
    cv::morphologyEx(
        particlesFillUp,
        regionClosing,
        cv::MORPH_CLOSE,
        kernel);

    const cv::Mat skeleton =
        CWeldSkeletonFeatureTools::thinBinaryMask(regionClosing, 100);
    if (skeleton.empty() || cv::countNonZero(skeleton) == 0)
    {
        return std::vector<cv::Point2d>();
    }

    // 2.2 骨架分支整理：剪掉分叉中的弱分支，减少闭合区域或反光产生的多余路径。
    const cv::Mat branchPruned = pruneJunctionBranches(gray8, skeleton);
    std::vector<int> logWidthByRow(
        static_cast<std::size_t>(gray8.rows),
        kSourceInitialLogWidthPixel);
    LaserLogPeakOptions logOptions;
    logOptions.rowMinPixel = 0;
    logOptions.rowMaxPixel = gray8.rows;
    logOptions.columnStepPixel = kSourceInitialLogColumnStepPixel;
    logOptions.logThreshold = -3000.0;
    logOptions.requiredPeakGrayValue = 255;

    // 2.3 LOG 频次评分：用旧 LOG_new 等价峰值落点频率选择更可信的填充区域骨架段。
    VisionPreprocessStatus status = VisionPreprocessStatus::Succeeded;
    const LaserLogPeakResult logResult = CLaserLineResponseTools::detectLogPeaks(
        gray8,
        logWidthByRow,
        logOptions,
        &status);
    const std::vector<int> frequencyByLabel =
        countLogFrequencyByFilledComponent(particlesFillUp, logResult);

    std::vector<SkeletonComponent> branchComponents =
        collectSkeletonComponents(branchPruned);
    applyFilledComponentFrequencyScore(
        particlesFillUp,
        frequencyByLabel,
        &branchComponents);
    const std::vector<SkeletonComponent> frequencySelectedComponents =
        selectOverlappingComponentsByScore(branchComponents, false);
    cv::Mat frequencySelectedMask = maskFromComponents(
        gray8.size(),
        frequencySelectedComponents);
    if (cv::countNonZero(frequencySelectedMask) == 0)
    {
        frequencySelectedMask = branchPruned;
    }

    // 2.4 浮空段筛选：对横向范围重叠的骨架段按长度再筛一次，
    // 避免短小浮空段参与初始曲线。
    const std::vector<SkeletonComponent> floatingComponents =
        collectSkeletonComponents(frequencySelectedMask);
    const std::vector<SkeletonComponent> lengthSelectedComponents =
        selectOverlappingComponentsByScore(floatingComponents, true);
    cv::Mat finalSkeletonMask =
        maskFromComponents(gray8.size(), lengthSelectedComponents);
    if (cv::countNonZero(finalSkeletonMask) == 0)
    {
        finalSkeletonMask = frequencySelectedMask;
    }

    // 2.5 CoM 亚像素修正：先用骨架列中心作为整数行候选，再用质心细化到 sub-pixel。
    const std::vector<int> skeletonRows =
        makeColumnCenterRows(finalSkeletonMask);
    std::vector<int> centerOfMassWidthByRow(
        static_cast<std::size_t>(gray8.rows),
        kSourceInitialComWidthPixel);
    const LaserCenterOfMassResult centerOfMassResult =
        CLaserLineResponseTools::refineCenterOfMass(
            gray8,
            skeletonRows,
            centerOfMassWidthByRow,
            kSourceInitialComMaxWidthPixel,
            &status);
    if (!centerOfMassResult.valid)
    {
        return std::vector<cv::Point2d>();
    }

    // 2.6 控制点采样：按配置步长抽取 refinedRowByColumn，输出 pixel 坐标控制点。
    return makeInitialControlPointsFromCenterRows(
        centerOfMassResult.refinedRowByColumn,
        options.initialControlPointStepPixel);
}

/**
 * @brief 从主动轮廓检测参数提取 BSnake 控制点工具参数。
 * @param options 主动轮廓检测参数。
 * @return 控制点修正、拖拽、增长和插值工具参数。
 * @note 该转换只读取 Snake/BSnake 条纹提取参数，不读取焊缝语义参数。
 */
ContourControlPointOptions makeContourControlPointOptions(
    const ActiveContourStripeOptions &options)
{
    ContourControlPointOptions contourOptions;
    contourOptions.growMinGrayValue = options.controlPointGrowMinGrayValue;
    contourOptions.growSampleStepPixel =
        options.controlPointGrowSampleStepPixel;
    contourOptions.offlineMinGrayValue =
        options.controlPointOfflineMinGrayValue;
    contourOptions.curveOfflineMinGrayValue =
        options.controlPointCurveOfflineMinGrayValue;
    contourOptions.dragSearchHalfHeightPixel =
        options.controlPointDragSearchHalfHeightPixel;
    contourOptions.dragMinGrayValue = options.controlPointDragMinGrayValue;
    contourOptions.maxControlPointGapPixel =
        options.controlPointMaxGapPixel;
    contourOptions.curveScanMaxPointCount =
        options.controlPointCurveScanMaxPointCount;
    contourOptions.interpolationSamplesPerSegment =
        options.interpolationSamplesPerSegment;
    return contourOptions;
}

} // namespace

/**
 * @brief 估计每一行激光条纹宽度。
 * @param gray8 输入灰度图，必须为 `CV_8UC1`。
 * @param options 激光宽度阈值、最大宽度和安全宽度倍率。
 * @return 逐行激光宽度估计；失败时 `valid=false`。
 * @note 先按列统计亮像素宽度，再以亮条纹中心行和列宽做线性拟合，
 * 输出每一行的宽度与安全宽度，单位均为 pixel。
 * @warning 空图、非 `CV_8UC1` 图像、宽度上限非法或没有亮前景时返回无效结果。
 */
LaserWidthProfile CActiveContourStripeExtractor::estimateLaserWidth(
    const cv::Mat &gray8,
    const ActiveContourStripeOptions &options)
{
    LaserWidthProfile profile;
    // 1. 输入检查：主动轮廓链路只接收 CV_8UC1，宽度上限和安全倍率必须为正。
    if (!isValidGrayImage(gray8))
    {
        profile.message = gray8.empty()
            ? "empty input"
            : "input must be CV_8UC1";
        return profile;
    }
    if (options.laserWidthMaxPixel <= 0
        || options.laserWidthSafeScale <= 0.0)
    {
        profile.message = "invalid laser width options";
        return profile;
    }

    std::vector<int> widthByColumn(gray8.cols, 0);
    std::vector<int> centerRowByColumn(gray8.cols, 0);
    // 2. 列方向统计：每列亮像素数量表示该列激光宽度，首尾亮点中点表示中心行。
    for (int xPixel = 0; xPixel < gray8.cols; ++xPixel)
    {
        int firstRow = -1;
        int lastRow = -1;
        int count = 0;
        for (int yPixel = 0; yPixel < gray8.rows; ++yPixel)
        {
            if (gray8.at<unsigned char>(yPixel, xPixel)
                <= options.laserWidthThresholdGrayValue)
            {
                continue;
            }
            if (firstRow < 0)
            {
                firstRow = yPixel;
            }
            lastRow = yPixel;
            ++count;
        }

        widthByColumn[xPixel] = count;
        centerRowByColumn[xPixel] = firstRow >= 0
            ? (firstRow + lastRow) / 2
            : 0;
        if (count > 0)
        {
            ++profile.validColumnCount;
        }
    }
    if (profile.validColumnCount <= 0)
    {
        profile.message = "laser width foreground not found";
        return profile;
    }

    double xy = 0.0;
    double xx = 0.0;
    double xsum = 0.0;
    double ysum = 0.0;
    // 3. 宽度拟合：用中心行 row 与列宽 width 做一阶拟合，
    // 得到按图像 y 坐标变化的激光宽度模型。
    for (int xPixel = 0; xPixel < gray8.cols; ++xPixel)
    {
        if (widthByColumn[xPixel] == 0)
        {
            continue;
        }
        const double row = static_cast<double>(centerRowByColumn[xPixel]);
        const double width = static_cast<double>(widthByColumn[xPixel]);
        xy += row * width;
        xx += row * row;
        xsum += row;
        ysum += width;
    }

    const double count = static_cast<double>(profile.validColumnCount);
    const double denominator = xx - xsum * xsum / count;
    const double slope = std::fabs(denominator) > kEpsilon
        ? (xy - xsum * ysum / count) / denominator
        : 0.0;
    const double intercept = ysum / count - slope * xsum / count;

    profile.widthByRowPixel.reserve(static_cast<std::size_t>(gray8.rows));
    profile.safeWidthByRowPixel.reserve(static_cast<std::size_t>(gray8.rows));
    // 4. 逐行输出：宽度限制在 1~laserWidthMaxPixel，
    // safeWidthByRowPixel 是用于拖拽/搜索的放大安全宽度，单位为 pixel。
    for (int yPixel = 0; yPixel < gray8.rows; ++yPixel)
    {
        int width = static_cast<int>(std::lround(
            slope * static_cast<double>(yPixel) + intercept));
        width = std::max(1, std::min(options.laserWidthMaxPixel, width));
        profile.widthByRowPixel.push_back(width);
        profile.safeWidthByRowPixel.push_back(static_cast<int>(std::ceil(
            static_cast<double>(width) * options.laserWidthSafeScale)));
    }

    // 5. 质量语义：validColumnCount 表示参与拟合的有效列数量。
    profile.valid = true;
    profile.message = "laser width estimated";
    return profile;
}

/**
 * @brief 从灰度图初始化主动轮廓曲线。
 * @param gray8 输入灰度图，必须为 `CV_8UC1`。
 * @param options 初始阈值、形态学闭运算半径、控制点步长和插值参数。
 * @return 初始主动轮廓曲线；控制点不足或插值失败时 `valid=false`。
 * @note 主路径按旧初始化顺序执行阈值、连通域等价填充、closing、骨架化、
 * 分支剔除、浮空段筛选、每列中心和 CoM 亚像素修正；失败时才退回
 * skeleton/mask 的列中心初始化。
 * @warning 初始控制点步长或插值采样非法、图像中激光候选不足时无法初始化。
 */
ActiveContourCurve CActiveContourStripeExtractor::initializeCurve(
    const cv::Mat &gray8,
    const ActiveContourStripeOptions &options)
{
    ActiveContourCurve curve;
    // 1. 输入检查：初始化阶段不负责彩色图转灰度，也不负责相机采集。
    if (!isValidGrayImage(gray8))
    {
        curve.message = gray8.empty() ? "empty input" : "input must be CV_8UC1";
        return curve;
    }
    if (options.initialControlPointStepPixel <= 0
        || options.interpolationSamplesPerSegment <= 0)
    {
        curve.message = "invalid curve options";
        return curve;
    }

    // 2. 主路径：流程级复刻旧 InitBSnake_new，HALCON region/XLD/NURBS
    // 分别由 OpenCV mask、骨架 component 和 tinyspline 采样点列承接。
    std::vector<cv::Point2d> controlPoints =
        buildInitialControlPoints(gray8, options);
    const bool usedCommonSkeletonFlow = controlPoints.size() >= 2U;

    cv::Mat mask;
    // 3. fallback 粗分割：主路径不足时，用阈值和可选 closing 连接断裂亮区。
    cv::threshold(
        gray8,
        mask,
        static_cast<double>(options.initialThresholdGrayValue),
        255.0,
        cv::THRESH_BINARY);
    if (options.initialClosingRadiusPixel > 0)
    {
        const int kernelSize =
            2 * options.initialClosingRadiusPixel + 1;
        cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size(kernelSize, kernelSize));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    }

    if (controlPoints.size() < 2U)
    {
        // 4. fallback 初始化：公共骨架流程不足时，使用 skeleton 的列平均行；
        // skeleton 为空或该列无骨架点时退回二值 mask 的列平均行。
        const cv::Mat skeleton =
            CWeldSkeletonFeatureTools::thinBinaryMask(mask, 100);
        controlPoints.clear();
        for (int xPixel = 0; xPixel < gray8.cols;
             xPixel += options.initialControlPointStepPixel)
        {
            double rowSum = 0.0;
            int rowCount = 0;
            for (int yPixel = 0; yPixel < gray8.rows; ++yPixel)
            {
                if (!skeleton.empty()
                    && skeleton.at<unsigned char>(yPixel, xPixel) > 0)
                {
                    rowSum += static_cast<double>(yPixel);
                    ++rowCount;
                }
            }
            if (rowCount == 0)
            {
                for (int yPixel = 0; yPixel < gray8.rows; ++yPixel)
                {
                    if (mask.at<unsigned char>(yPixel, xPixel) > 0)
                    {
                        rowSum += static_cast<double>(yPixel);
                        ++rowCount;
                    }
                }
            }
            if (rowCount > 0)
            {
                controlPoints.push_back(cv::Point2d(
                    static_cast<double>(xPixel),
                    rowSum / static_cast<double>(rowCount)));
            }
        }
    }

    controlPoints = sortedUniquePoints(controlPoints);
    // 5. 曲线插值：控制点按 x 升序去重后交给控制点工具输出 NURBS/B-spline 替代曲线。
    if (controlPoints.size() < 2U)
    {
        curve.message = "not enough initial control points";
        return curve;
    }

    VisionPreprocessStatus status = VisionPreprocessStatus::Succeeded;
    const ControlPointCurveResult interpolation =
        CContourControlPointTools::interpolateControlCurve(
            controlPoints,
            options.interpolationSamplesPerSegment,
            &status);
    if (!interpolation.valid)
    {
        curve.message = interpolation.message;
        return curve;
    }

    curve.controlPoints = interpolation.controlPoints;
    curve.curvePoints = interpolation.curvePoints;
    curve.valid = true;
    curve.message = usedCommonSkeletonFlow
        ? "active contour curve initialized by shared skeleton flow"
        : "active contour curve initialized by fallback";
    return curve;
}

/**
 * @brief 对已有主动轮廓曲线执行 BSnake 风格贪婪迭代。
 * @param gray8 输入灰度图，必须为 `CV_8UC1`。
 * @param laserWidthProfile 已估计的逐行激光宽度，单位为 pixel。
 * @param inputCurve 上一帧或初始化得到的主动轮廓曲线。
 * @param options 控制点修正、贪婪搜索和插值参数。
 * @return 迭代后的主动轮廓曲线；失败时 `valid=false`。
 * @note 迭代前会先执行 `BSnake_HandleConPt` 等价修正，再使用旧
 * `EfficientMask/LOGCore` 风格的局部候选响应搜索。
 * @warning 宽度数组长度不足、控制点不足或 tinyspline 插值失败时返回无效结果。
 */
ActiveContourCurve CActiveContourStripeExtractor::iterateCurve(
    const cv::Mat &gray8,
    const LaserWidthProfile &laserWidthProfile,
    const ActiveContourCurve &inputCurve,
    const ActiveContourStripeOptions &options)
{
    ActiveContourCurve curve;
    // 1. 输入检查：迭代依赖有效宽度、有效曲线和至少两个控制点。
    if (!isValidGrayImage(gray8))
    {
        curve.message = gray8.empty() ? "empty input" : "input must be CV_8UC1";
        return curve;
    }
    if (!laserWidthProfile.valid
        || static_cast<int>(laserWidthProfile.widthByRowPixel.size())
            < gray8.rows
        || !inputCurve.valid
        || inputCurve.controlPoints.size() < 2U)
    {
        curve.message = "invalid active contour input";
        return curve;
    }

    VisionPreprocessStatus controlPointStatus =
        VisionPreprocessStatus::Succeeded;
    // 2. 控制点预处理：先接入 BSnake_HandleConPt 等价流程，
    // 修正离线点、补暗点、补大间距点并扩展左右端部。
    const ControlPointCurveResult preparedCurve =
        CContourControlPointTools::adjustOfflineControlPoints(
            gray8,
            laserWidthProfile.widthByRowPixel,
            inputCurve.controlPoints,
            makeContourControlPointOptions(options),
            &controlPointStatus);
    if (!preparedCurve.valid)
    {
        curve.message = "active contour control point adjustment failed: "
            + preparedCurve.message;
        return curve;
    }

    std::vector<cv::Point2d> adjustedPoints;
    adjustedPoints.reserve(preparedCurve.controlPoints.size());
    // 3. 贪婪迭代：每个控制点根据上次移动方向构造 EfficientMask，
    // 在局部候选中选择 LOG 响应最高且亮度满足阈值的位置。
    for (const cv::Point2d &point : preparedCurve.controlPoints)
    {
        int currentX = static_cast<int>(std::lround(point.x));
        int currentY = static_cast<int>(std::lround(point.y));
        if (!isInsideImage(gray8, currentX, currentY))
        {
            continue;
        }

        double bestResponse = -std::numeric_limits<double>::infinity();
        int directionX = 0;
        int directionY = 0;
        for (int iteration = 0; iteration < options.snakeIterationCount;
             ++iteration)
        {
            const LaserNeighborMask mask =
                CLaserLineResponseTools::buildEfficientMask(
                    directionX,
                    directionY);
            if (!mask.valid || mask.offsets.empty())
            {
                break;
            }

            bool moved = false;
            int bestX = currentX;
            int bestY = currentY;
            int bestDirectionX = 0;
            int bestDirectionY = 0;
            for (const cv::Point &offset : mask.offsets)
            {
                const int candidateX = currentX + offset.x;
                const int candidateY = currentY + offset.y;
                if (!isInsideImage(gray8, candidateX, candidateY)
                    || grayAt(gray8, candidateX, candidateY)
                        < options.snakeMinGrayValue)
                {
                    continue;
                }

                VisionPreprocessStatus status =
                    VisionPreprocessStatus::Succeeded;
                const double response =
                    CLaserLineResponseTools::calculateLogResponseAt(
                        gray8,
                        laserWidthProfile.widthByRowPixel,
                        candidateX,
                        candidateY,
                        &status);
                if (status == VisionPreprocessStatus::Succeeded
                    && response > bestResponse)
                {
                    bestResponse = response;
                    bestX = candidateX;
                    bestY = candidateY;
                    bestDirectionX = offset.x;
                    bestDirectionY = offset.y;
                    moved = offset.x != 0 || offset.y != 0;
                }
            }

            if (!moved)
            {
                break;
            }

            currentX = bestX;
            currentY = bestY;
            directionX = bestDirectionX;
            directionY = bestDirectionY;
        }
        adjustedPoints.push_back(cv::Point2d(
            static_cast<double>(currentX),
            static_cast<double>(currentY)));
    }

    adjustedPoints = sortedUniquePoints(adjustedPoints);
    // 4. 曲线重建：迭代后的控制点重新插值为 curvePoints，
    // 供下游 T/对接/搭接分支在 ROI 内拟合线段。
    VisionPreprocessStatus status = VisionPreprocessStatus::Succeeded;
    const ControlPointCurveResult interpolation =
        CContourControlPointTools::interpolateControlCurve(
            adjustedPoints,
            options.interpolationSamplesPerSegment,
            &status);
    if (!interpolation.valid)
    {
        curve.message = interpolation.message;
        return curve;
    }

    curve.controlPoints = interpolation.controlPoints;
    curve.curvePoints = interpolation.curvePoints;
    curve.valid = true;
    curve.message = "active contour curve iterated";
    return curve;
}

/**
 * @brief 独立提取主动轮廓条纹曲线。
 * @param gray8 输入灰度图，必须为 `CV_8UC1`。
 * @param options 主动轮廓条纹参数。
 * @param state 调用方持有的跨帧条纹状态；不得为 nullptr。
 * @return 条纹曲线提取结果，不包含任何焊缝类型语义。
 */
ActiveContourStripeResult CActiveContourStripeExtractor::extract(
    const cv::Mat &gray8,
    const ActiveContourStripeOptions &options,
    ActiveContourStripeState *state)
{
    const auto startedAt = std::chrono::steady_clock::now();
    auto finish = [startedAt](ActiveContourStripeResult result) {
        const auto finishedAt = std::chrono::steady_clock::now();
        result.processingTimeMs =
            std::chrono::duration<double, std::milli>(
                finishedAt - startedAt).count();
        return result;
    };

    ActiveContourStripeResult result;
    if (state == nullptr)
    {
        result.message = "stripe state is required";
        return finish(result);
    }
    if (!isValidGrayImage(gray8))
    {
        result.message = gray8.empty() ? "empty input" : "input must be CV_8UC1";
        return finish(result);
    }

    ++state->frameCount;
    if (!state->initialized)
    {
        state->laserWidthProfile = estimateLaserWidth(gray8, options);
        state->activeCurve = initializeCurve(gray8, options);
        state->initialized =
            state->laserWidthProfile.valid && state->activeCurve.valid;
        if (!state->initialized)
        {
            result.laserWidthProfile = state->laserWidthProfile;
            result.activeCurve = state->activeCurve;
            result.message = state->laserWidthProfile.valid
                ? state->activeCurve.message
                : state->laserWidthProfile.message;
            return finish(result);
        }
    }

    const ActiveContourCurve iteratedCurve = iterateCurve(
        gray8,
        state->laserWidthProfile,
        state->activeCurve,
        options);
    if (!iteratedCurve.valid)
    {
        state->initialized = false;
        result.laserWidthProfile = state->laserWidthProfile;
        result.activeCurve = iteratedCurve;
        result.message = iteratedCurve.message;
        return finish(result);
    }

    state->activeCurve = iteratedCurve;
    result.valid = true;
    result.laserWidthProfile = state->laserWidthProfile;
    result.activeCurve = iteratedCurve;
    result.qualityScore = clampRatio(
        static_cast<double>(iteratedCurve.curvePoints.size())
        / static_cast<double>(std::max(1, gray8.cols)));
    result.message = "active contour stripe extracted";
    return finish(result);
}

} // namespace i2r_rlc::vision

