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
 * @file        ActiveContourStripeExtractor.h
 * @brief       主动轮廓激光条纹提取接口
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
#ifndef I2R_RLC_VISION_STRIPE_ACTIVE_CONTOUR_STRIPE_EXTRACTOR_H
#define I2R_RLC_VISION_STRIPE_ACTIVE_CONTOUR_STRIPE_EXTRACTOR_H

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace i2r_rlc::vision {

/**
 * @brief 每一行的激光宽度估计结果。
 *
 * 宽度单位为 pixel。`widthByRowPixel` 对应旧宽度数组前半段，
 * `safeWidthByRowPixel` 对应旧安全宽度数组后半段。
 */
struct LaserWidthProfile {
    /// @brief 每行激光宽度，单位 pixel。
    std::vector<int> widthByRowPixel;

    /// @brief 每行安全激光宽度，单位 pixel。
    std::vector<int> safeWidthByRowPixel;

    /// @brief 用于拟合宽度的有效列数量，单位为 column。
    int validColumnCount = 0;

    /// @brief true 表示宽度估计可用于主动轮廓迭代。
    bool valid = false;

    /// @brief 状态说明，不使用平台相关字符串类型；仅用于诊断。
    std::string message;
};

/**
 * @brief 主动轮廓曲线。
 *
 * `controlPoints` 是用于迭代和插值的控制点，`curvePoints` 是
 * NURBS/B-spline 替代曲线采样点。坐标单位均为 pixel。
 */
struct ActiveContourCurve {
    /// @brief 控制点，单位 pixel，按 x 坐标升序排列。
    std::vector<cv::Point2d> controlPoints;

    /// @brief 曲线采样点，单位 pixel。
    std::vector<cv::Point2d> curvePoints;

    /// @brief true 表示曲线可用于下游条纹或焊缝特征处理。
    bool valid = false;

    /// @brief 状态说明，不使用平台相关字符串类型；仅用于诊断。
    std::string message;
};

/**
 * @brief 主动轮廓条纹提取参数。
 *
 * 参数来自旧主动轮廓检测链路的经验值，并用显式字段替代旧全局变量。
 * 所有距离单位均为 pixel，灰度阈值范围为 0~255。
 */
struct ActiveContourStripeOptions {
    /// @brief 激光宽度估计使用的亮像素阈值，范围为 0~255。
    int laserWidthThresholdGrayValue = 200;

    /// @brief 激光宽度最大值，单位 pixel。
    int laserWidthMaxPixel = 16;

    /// @brief 安全宽度相对宽度的倍率，无量纲 ratio。
    double laserWidthSafeScale = 1.5;

    /// @brief 初始曲线二值化阈值，范围为 0~255。
    int initialThresholdGrayValue = 120;

    /// @brief 初始曲线形态学 closing 半径近似值，单位 pixel。
    int initialClosingRadiusPixel = 4;

    /// @brief 从列中心点生成初始控制点时的列采样步长，单位为 pixel。
    int initialControlPointStepPixel = 8;

    /// @brief 曲线插值每段采样点数量，单位为 point。
    int interpolationSamplesPerSegment = 8;

    /// @brief 主动轮廓每个控制点的最大贪婪迭代次数，单位为 iteration。
    int snakeIterationCount = 50;

    /// @brief 控制点候选必须达到的亮度阈值，范围为 0~255。
    int snakeMinGrayValue = 180;

    /// @brief 控制点左右增长时的亮点阈值，范围为 0~255。
    int controlPointGrowMinGrayValue = 180;

    /// @brief 控制点增长时每隔多少个跟踪像素输出一个控制点，单位为 pixel。
    int controlPointGrowSampleStepPixel = 5;

    /// @brief BSnake 控制点离线判定阈值，范围为 0~255。
    int controlPointOfflineMinGrayValue = 220;

    /// @brief 临时曲线采样点离线判定阈值，范围为 0~255。
    int controlPointCurveOfflineMinGrayValue = 200;

    /// @brief 控制点拖拽搜索半高，单位 pixel。
    int controlPointDragSearchHalfHeightPixel = 20;

    /// @brief 控制点拖拽候选亮度阈值，范围为 0~255。
    int controlPointDragMinGrayValue = 100;

    /// @brief 相邻控制点列距超过该值时插入中间控制点，单位 pixel。
    int controlPointMaxGapPixel = 30;

    /// @brief 临时曲线采样点少于该数量时扫描离线点，单位为 point。
    int controlPointCurveScanMaxPointCount = 1000;

};

/**
 * @brief 主动轮廓条纹提取跨帧状态。
 */
struct ActiveContourStripeState {
    /// @brief true 表示已完成初始曲线和宽度估计，可直接进入迭代。
    bool initialized = false;

    /// @brief 已处理帧数，单位为 frame。
    int frameCount = 0;

    /// @brief 当前激光宽度估计。
    LaserWidthProfile laserWidthProfile;

    /// @brief 当前主动轮廓曲线。
    ActiveContourCurve activeCurve;
};

/**
 * @brief 主动轮廓条纹提取结果。
 */
struct ActiveContourStripeResult {
    /// @brief true 表示得到可用于下游算法的条纹曲线。
    bool valid = false;

    /// @brief 本帧激光宽度估计。
    LaserWidthProfile laserWidthProfile;

    /// @brief 本帧主动轮廓曲线。
    ActiveContourCurve activeCurve;

    /// @brief 条纹曲线质量，范围为 0~1。
    double qualityScore = 0.0;

    /// @brief 处理耗时，单位 ms。
    double processingTimeMs = 0.0;

    /// @brief 状态说明。
    std::string message;
};

/**
 * @brief OpenCV-only 主动轮廓条纹提取器。
 */
class CActiveContourStripeExtractor final {
public:
    /**
     * @brief 估计每行激光条纹宽度。
     * @param gray8 输入 `CV_8UC1` 灰度图。
     * @param options 主动轮廓参数，宽度单位为 pixel。
     * @return 每行宽度和安全宽度估计。
     */
    static LaserWidthProfile estimateLaserWidth(
        const cv::Mat &gray8,
        const ActiveContourStripeOptions &options);

    /**
     * @brief 从二值骨架/列中心初始化主动轮廓曲线。
     * @param gray8 输入 `CV_8UC1` 灰度图。
     * @param options 主动轮廓初始化参数。
     * @return 初始控制点和采样曲线，坐标单位 pixel。
     */
    static ActiveContourCurve initializeCurve(
        const cv::Mat &gray8,
        const ActiveContourStripeOptions &options);

    /**
     * @brief 对已有主动轮廓曲线执行一帧迭代。
     * @param gray8 输入 `CV_8UC1` 灰度图。
     * @param laserWidthProfile 当前激光宽度估计。
     * @param inputCurve 输入控制点和曲线，坐标单位 pixel。
     * @param options 主动轮廓迭代参数。
     * @return 迭代后的曲线。
     */
    static ActiveContourCurve iterateCurve(
        const cv::Mat &gray8,
        const LaserWidthProfile &laserWidthProfile,
        const ActiveContourCurve &inputCurve,
        const ActiveContourStripeOptions &options);

    /**
     * @brief 一次性执行主动轮廓条纹提取。
     * @param gray8 输入 `CV_8UC1` 灰度图。
     * @param options 主动轮廓参数。
     * @param state 可选跨帧状态；函数会更新宽度和曲线缓存，不持有该指针。
     * @return 主动轮廓条纹提取结果。
     */
    static ActiveContourStripeResult extract(
        const cv::Mat &gray8,
        const ActiveContourStripeOptions &options,
        ActiveContourStripeState *state);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_STRIPE_ACTIVE_CONTOUR_STRIPE_EXTRACTOR_H

