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
 * @file        WeldSkeletonFeatureTools.h
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
#ifndef I2R_RLC_VISION_STRIPE_WELD_SKELETON_FEATURE_TOOLS_H
#define I2R_RLC_VISION_STRIPE_WELD_SKELETON_FEATURE_TOOLS_H

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace i2r_rlc::vision {

/**
 * @brief 焊接骨架提取参数。
 *
 * 本结构只用于 CV_8UC1 mask 或灰度图。ratio/confidence 字段按 0~1
 * 约定，面积和点数单位均为 pixel。
 */
struct WeldSkeletonExtractionOptions {
    /// @brief 输入是否已经是二值 mask；false 时按 thresholdValue 阈值化。
    bool inputAlreadyBinary = true;

    /// @brief 灰度阈值，范围 0~255，仅在 inputAlreadyBinary=false 时使用。
    int thresholdValue = 128;

    /// @brief 小连通域过滤阈值，单位 pixel；小于等于 1 时不启用。
    int minComponentArea = 3;

    /// @brief Zhang-Suen 细化最大迭代次数。
    int maxIterations = 100;

    /// @brief 低于该置信度时输出 invalid，范围 0~1。
    double minConfidence = 0.10;
};

/**
 * @brief 激光中心线骨架提取参数。
 *
 * 本结构用于旧工程来源的激光中心线主流程，也作为主动轮廓初始化可复用
 * 的公共 OpenCV 流程参数。默认流程优先保留接近 HALCON `skeleton` 的完整
 * 单像素骨架，再从该骨架派生中心线点。面积和距离单位为 pixel。
 */
struct LaserSkeletonExtractionOptions {
    /// @brief 灰度阈值，范围 0~255。
    int thresholdValue = 100;

    /// @brief 小连通域过滤阈值，单位 pixel；默认 0 表示保留短支路以贴近 HALCON skeleton。
    int minComponentArea = 0;

    /// @brief closing 圆形结构元素半径近似值，单位 pixel。
    int closingRadiusPixel = 4;

    /// @brief true 表示骨架化前启用激光线候选 region 约束；默认关闭以保留完整骨架。
    bool enableLaserRegionPruning = false;

    /// @brief region 厚度超过该值时视为宽反光候选，单位 pixel。
    int maxRegionThicknessPixel = 10;

    /// @brief 窄长 region 的最小长宽比，小于该值且较厚时优先按 LOG 峰值窄带保留。
    double minRegionAspectRatio = 3.0;

    /// @brief 大块 region 面积阈值，单位 pixel；超过后不直接整块填充。
    int maxRegionAreaPixel = 1500;

    /// @brief 从 LOG 峰值重建窄带 region 时的半宽，单位 pixel。
    int regionPruningPeakHalfWidthPixel = 3;

    /// @brief Zhang-Suen 细化最大迭代次数。
    int maxThinningIterations = 100;

    /// @brief LOG 列采样步长，单位 pixel。
    int logColumnStepPixel = 5;

    /// @brief 估计激光宽度，单位 pixel。
    int laserWidthPixel = 8;

    /// @brief CoM 搜索最大半宽，单位 pixel。
    int laserWidthMaxPixel = 12;

    /// @brief 浮空段筛选的最小 LOG 命中列数。
    int minLogSupportColumns = 3;

    /// @brief true 表示用 LOG 支撑筛选派生中心线，不会覆盖完整 skeletonMask。
    bool enableLogSupportedComponentFilter = false;

    /**
     * @brief true 表示额外生成 T/角接几何分支专用的有序主骨架路径。
     *
     * 该路径会在同列多骨架点中按连续性、灰度和 LOG 支撑选择真实骨架点，
     * 不做平均。它只应在旧 ImageProFeng T 几何分支等需要 `split_line`
     * 输入的场景开启；主动轮廓和普通中心线提取默认不启用，避免额外开销和
     * 不必要的分支选择影响。
     */
    bool enableOrderedPathExtraction = false;

    /// @brief 分支强度筛选的最小平均灰度。
    int minBranchMeanGrayValue = 80;

    /// @brief 最少中心线点数。
    int minCenterPointCount = 5;

    /// @brief 低于该置信度时输出 invalid，范围 0~1。
    double minConfidence = 0.10;

    /// @brief true 表示主骨架流程失败时启用 HoughLinesP 备用线段提取。
    bool enableHoughFallback = true;

    /// @brief HoughLinesP 累计阈值。
    int houghThreshold = 20;

    /// @brief HoughLinesP 最短线段长度，单位 pixel。
    double houghMinLineLengthPixel = 20.0;

    /// @brief HoughLinesP 最大断裂连接间隙，单位 pixel。
    double houghMaxLineGapPixel = 8.0;
};

/**
 * @brief 焊接骨架提取结果。
 */
struct WeldSkeletonFeatureResult {
    /// @brief true 表示得到完整骨架，并派生出可用于下游 feature 的中心线。
    bool valid = false;

    /// @brief 完整单像素骨架 mask，类型为 CV_8UC1，前景值为 255，默认保留分叉。
    cv::Mat skeletonMask;

    /// @brief 骨架化前的二值候选 region，类型为 CV_8UC1，用于 replay 调试。
    cv::Mat preprocessedMask;

    /// @brief 完整骨架点列，单位 pixel；与 skeletonMask 语义一致，不保证按列连续。
    std::vector<cv::Point2d> skeletonPoints;

    /// @brief 从完整骨架按列派生的中心线点，单位 pixel，用于拟合和控制点初始化。
    std::vector<cv::Point2d> centerLinePoints;

    /// @brief 经过 CoM 亚像素修正后的中心线点，单位 pixel；优先供下游算法使用。
    std::vector<cv::Point2d> refinedCenterPoints;

    /**
     * @brief 旧工程来源 T/角接几何分支使用的有序主骨架路径，单位 pixel。
     *
     * 该点列先从完整 skeleton 中选择连续主分支，再逐列取一个真实骨架点并
     * 做 CoM 修正；它不会对同列多骨架点求平均，因此适合 `split_line` 和
     * RANSAC。通用 `centerLinePoints` 仍保留按列中心语义，仅用于单值中心线
     * 派生或可视化。
     */
    std::vector<cv::Point2d> orderedPathPoints;

    /// @brief 骨架端点数量。
    int endpointCount = 0;

    /// @brief 骨架分叉点数量。
    int branchPointCount = 0;

    /// @brief 骨架连通分量数量。
    int connectedComponentCount = 0;

    /// @brief 骨架像素数占输入前景像素数比例，范围 0~1。
    double validPixelRatio = 0.0;

    /// @brief 骨架质量置信度，范围 0~1。
    double confidence = 0.0;

    /// @brief 状态说明，不使用平台相关字符串类型。
    std::string message;
};

/**
 * @brief 焊接骨架提取和拓扑统计工具。
 *
 * 本类迁移旧 `ImageProFeng::SkeletonExtraction` 中 skeleton、端点、
 * 分叉点和轮廓点列的低阶语义。实现使用 OpenCV-only 二值形态和
 * Zhang-Suen 细化，不依赖 Qt、HALCON、PCL 或真实设备。
 */
class CWeldSkeletonFeatureTools final {
public:
    /**
     * @brief 从 CV_8UC1 mask 或灰度图提取单像素骨架和拓扑统计。
     * @param grayOrMask8 输入 `CV_8UC1` 图像；按 options 可解释为二值 mask 或灰度图。
     * @param options 骨架提取参数，面积和迭代次数单位见字段说明。
     * @return 骨架、中心点和拓扑统计结果。
     */
    static WeldSkeletonFeatureResult extract(
        const cv::Mat &grayOrMask8,
        const WeldSkeletonExtractionOptions &options);

    /**
     * @brief 按旧工程来源主流程提取激光骨架中心线。
     *
     * 该入口在 `extract` 的最小骨架语义之上补齐旧流程中的阈值分割、
     * 连通域等价填充、closing、骨架化、每列中心点和 `CoM_new` 亚像素修正。
     * 输出 `skeletonMask` / `skeletonPoints` 为完整骨架，`refinedCenterPoints`
     * 为下游拟合使用的中心线。当前使用 OpenCV 形态学和 Zhang-Suen 细化替代
     * HALCON skeleton，因此是流程级等价，不承诺 HALCON 像素级一致。
     *
     * @param grayImage 输入 `CV_8UC1` 灰度图。
     * @param options 激光骨架提取参数，距离单位均为 pixel。
     * @return 激光骨架和派生中心线结果。
     */
    static WeldSkeletonFeatureResult extractLaserSkeleton(
        const cv::Mat &grayImage,
        const LaserSkeletonExtractionOptions &options);

    /**
     * @brief 对二值 mask 执行 Zhang-Suen 细化。
     * @param binaryMask8 输入 `CV_8UC1` 二值 mask，非零为前景。
     * @param maxIterations 最大细化迭代次数。
     * @return 单像素骨架 mask，类型为 `CV_8UC1`。
     */
    static cv::Mat thinBinaryMask(
        const cv::Mat &binaryMask8,
        int maxIterations);

    /**
     * @brief 收集骨架 mask 中的前景点。
     * @param skeletonMask8 输入 `CV_8UC1` 骨架 mask，非零为前景。
     * @return 前景点集合，坐标单位 pixel。
     */
    static std::vector<cv::Point2d> collectSkeletonPoints(
        const cv::Mat &skeletonMask8);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_STRIPE_WELD_SKELETON_FEATURE_TOOLS_H

