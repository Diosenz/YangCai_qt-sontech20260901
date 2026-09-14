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
 * @file        ContourControlPointTools.h
 * @brief       主动轮廓控制点处理工具
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
#ifndef I2R_RLC_VISION_FEATURE_CONTOUR_CONTROL_POINT_TOOLS_H
#define I2R_RLC_VISION_FEATURE_CONTOUR_CONTROL_POINT_TOOLS_H

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "common/VisionImageTypes.h"

namespace i2r_rlc::vision {

/**
 * @brief 控制点左右增长方向。
 *
 * 对应旧 `BSnakeGrow` 中从当前控制点向左或向右沿激光线寻找新控制点
 * 的方向参数。坐标仍使用图像 pixel 单位。
 */
enum class ControlPointGrowDirection {
    /// @brief 向图像左侧逐列增长控制点。
    Left,
    /// @brief 向图像右侧逐列增长控制点。
    Right
};

/**
 * @brief 控制点增长和曲线插值参数。
 */
struct ContourControlPointOptions {
    /// @brief 亮点阈值，范围为 0~255。
    int growMinGrayValue = 180;

    /// @brief 每隔多少个跟踪像素输出一个控制点，单位为 pixel。
    int growSampleStepPixel = 5;

    /// @brief 判断离线点的亮度阈值，范围为 0~255。
    int offlineMinGrayValue = 220;

    /// @brief 扫描临时曲线时判定曲线点离线的亮度阈值，范围为 0~255。
    int curveOfflineMinGrayValue = 200;

    /// @brief 拖拽离线点时的同列搜索半高，单位 pixel。
    int dragSearchHalfHeightPixel = 20;

    /// @brief 拖拽离线点时的候选亮度阈值，范围为 0~255。
    int dragMinGrayValue = 100;

    /// @brief 相邻控制点列距超过该值时插入中间控制点，单位 pixel。
    int maxControlPointGapPixel = 30;

    /// @brief 临时曲线采样点少于该数量时才扫描曲线离线点，单位为 point。
    int curveScanMaxPointCount = 1000;

    /// @brief tinyspline NURBS/B-spline 曲线每段采样数量，单位为 point，必须大于 0。
    int interpolationSamplesPerSegment = 8;
};

/**
 * @brief 控制点增长结果。
 */
struct ControlPointGrowResult {
    /// @brief 增长得到的控制点，单位 pixel。
    std::vector<cv::Point2d> points;

    /// @brief true 表示至少得到一个增长控制点，可继续参与曲线插值。
    bool valid = false;

    /// @brief 状态说明，不使用平台相关字符串类型；仅用于诊断。
    std::string message;
};

/**
 * @brief 控制点调整和曲线插值结果。
 */
struct ControlPointCurveResult {
    /// @brief 修正后的控制点，单位 pixel，按 x 坐标升序排列。
    std::vector<cv::Point2d> controlPoints;

    /// @brief 根据控制点插值得到的曲线采样点，单位 pixel。
    std::vector<cv::Point2d> curvePoints;

    /// @brief true 表示输出曲线可用于下游焊缝特征分析。
    bool valid = false;

    /// @brief 状态说明，不使用平台相关字符串类型；仅用于诊断。
    std::string message;
};

/**
 * @brief  BSnake 控制点处理的 OpenCV/C++ 替代工具。
 *曲线插值使用 tinyspline 生成 NURBS/B-spline 采样点。
 * 本类只包含无状态静态函数，不持有设备、线程或 UI 对象。
 */
class CContourControlPointTools final {
public:
    /**
     * @brief 左右三邻域策略增长控制点。
     *
     * 函数从 `startPoint` 出发，每次向左或向右推进一列，并按“当前行、
     * 上一行、下一行”的顺序寻找满足亮度阈值的激光线像素。每累计
     * `growSampleStepPixel` 个跟踪像素输出一个控制点。
     *
     * @param grayImage 输入灰度图，必须为 `CV_8UC1`。
     * @param startPoint 增长起点，单位 pixel。
     * @param direction 增长方向，向左或向右。
     * @param options 增长阈值和采样步长。
     * @param status 可选状态输出。
     * @return 增长得到的控制点；未增长出点时 `valid=false`。
     * @warning 起点越界、图像为空或采样步长非法时返回无效结果。
     */
    static ControlPointGrowResult growLaserControlPoints(
        const cv::Mat &grayImage,
        const cv::Point2i &startPoint,
        ControlPointGrowDirection direction,
        const ContourControlPointOptions &options,
        VisionPreprocessStatus *status);

    /**
     * @brief 修正离线控制点并输出插值曲线。
     *
     * 函数会先检查控制点
     * 是否落在亮激光线上；若亮度不足，则调用 `DragOfflinePt` 等价逻辑
     * 沿同列拖拽到附近激光线，拖拽失败的点会被删除。随后函数会生成
     * 临时曲线，扫描曲线离线点并补充控制点，对大间距控制点插入中点，
     * 最后向左右端继续增长并用 tinyspline 输出 NURBS/B-spline 曲线采样点。
     *
     * @param grayImage 输入灰度图，必须为 `CV_8UC1`。
     * @param laserWidthByRow 每一行估计的激光宽度，单位 pixel。
     * @param controlPoints 初始控制点，单位 pixel。
     * @param options 离线点阈值、拖拽范围、增长步长和曲线采样参数。
     * @param status 可选状态输出。
     * @return 修正后的控制点和曲线采样点；输入不足时 `valid=false`。
     * @note `laserWidthByRow` 的长度必须覆盖图像行数，宽度单位为
     * pixel；该数组由上层激光宽度估计得到。
     * @warning 离线点拖拽失败会删除对应控制点，控制点不足时不会输出曲线。
     */
    static ControlPointCurveResult adjustOfflineControlPoints(
        const cv::Mat &grayImage,
        const std::vector<int> &laserWidthByRow,
        const std::vector<cv::Point2d> &controlPoints,
        const ContourControlPointOptions &options,
        VisionPreprocessStatus *status);

    /**
     * @brief 对控制点执行 tinyspline NURBS/B-spline 曲线采样。
     *
     * 两个控制点时输出线性采样；三个及以上控制点时使用 tinyspline
     * 自然三次 B-spline 插值。tinyspline 类型只存在于 `.cpp`，不会
     * 泄漏到 vision 公共 API。
     *
     * @param controlPoints 输入控制点，单位 pixel。
     * @param samplesPerSegment 每段采样点数量，必须大于 0。
     * @param status 可选状态输出。
     * @return 排序去重后的控制点和曲线采样点。
     * @note 输出坐标单位为 pixel；该函数不暴露 tinyspline 或 HALCON 类型。
     * @warning 控制点不足、采样数量非法或 tinyspline 插值异常时返回无效结果。
     */
    static ControlPointCurveResult interpolateControlCurve(
        const std::vector<cv::Point2d> &controlPoints,
        int samplesPerSegment,
        VisionPreprocessStatus *status);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_FEATURE_CONTOUR_CONTROL_POINT_TOOLS_H

