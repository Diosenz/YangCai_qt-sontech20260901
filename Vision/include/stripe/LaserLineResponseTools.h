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
 * @file        LaserLineResponseTools.h
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
#ifndef I2R_RLC_VISION_STRIPE_LASER_LINE_RESPONSE_TOOLS_H
#define I2R_RLC_VISION_STRIPE_LASER_LINE_RESPONSE_TOOLS_H

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "common/VisionImageTypes.h"

namespace i2r_rlc::vision {

/**
 * @brief LOG 列峰检测参数。
 *
 * 所有坐标单位均为 pixel。本结构对应旧 `LOG_new` 的 ROI 行范围、
 * 列采样步长和 LOG 阈值。输入图像必须由调用层预先转换为
 * `CV_8UC1 cv::Mat`，本工具不负责文件读取、彩色转灰度、Qt 图像
 * 适配或相机 SDK buffer 适配。
 */
struct LaserLogPeakOptions {
    /// @brief ROI 起始行，单位 pixel，包含该行。
    int rowMinPixel = 0;

    /// @brief ROI 结束行，单位 pixel，不包含该行。
    int rowMaxPixel = 0;

    /// @brief 列采样步长，对应旧参数 `eff`。
    int columnStepPixel = 1;

    /// @brief 旧 LOG 阈值，响应大于该值时记录当前列的候选行。
    double logThreshold = -3000.0;

    /// @brief 参与 LOG 列峰候选的亮像素灰度，旧实现默认只处理 255 亮点。
    int requiredPeakGrayValue = 255;
};

/**
 * @brief LOG 列峰检测结果。
 *
 * `maxRowByColumn` 使用旧 `LOG_new` 的行号语义：检测到峰值时记录
 * `row + 1`，未采样或未检测到时为 -1。`maxResponseByColumn`
 * 保存对应列的最大 LOG 响应，便于离线 replay 判断阈值是否合适。
 */
struct LaserLogPeakResult {
    /// @brief 每列最大响应行，单位 pixel；无效列为 -1。
    std::vector<int> maxRowByColumn;

    /// @brief 每列最大 LOG 响应值。
    std::vector<double> maxResponseByColumn;

    /// @brief true 表示输入有效并完成列峰扫描。
    bool valid = false;

    /// @brief 状态说明，不使用平台相关字符串类型。
    std::string message;
};

/**
 * @brief 灰度重心细化结果。
 *
 * 旧函数同时输出 `pMaxRowXLD` 和 `pMaxRowXLDO` 两组行坐标。本结构用
 * `refinedRowByColumn` 和 `originalRowByColumn` 显式表达，避免旧裸指针
 * 输出的生命周期不清问题。
 */
struct LaserCenterOfMassResult {
    /// @brief 灰度重心修正后的行坐标，单位 pixel。
    std::vector<double> refinedRowByColumn;

    /// @brief 与旧 `pMaxRowXLDO` 对应的原始输出行坐标，单位 pixel。
    std::vector<double> originalRowByColumn;

    /// @brief true 表示输入有效并完成重心细化。
    bool valid = false;

    /// @brief 状态说明，不使用平台相关字符串类型。
    std::string message;
};

/**
 * @brief 同列拖拽离线点到激光线的结果。
 *
 * 对应旧 `DragOfflinePt`，用于把亮度不足的控制点沿同一列上下搜索到
 * LOG 响应更强的激光线位置。该结果只表达图像坐标，不做空间重建。
 */
struct LaserDraggedPointResult {
    /// @brief 拖拽后的点坐标，单位 pixel。
    cv::Point2d point;

    /// @brief 拖拽过程中观测到的最大 LOG 响应。
    double maxResponse = 0.0;

    /// @brief true 表示输入有效并完成搜索。
    bool valid = false;

    /// @brief 状态说明，不使用平台相关字符串类型。
    std::string message;
};

/**
 * @brief 旧 EfficientMask 的邻域偏移结果。
 *
 * 旧函数根据当前增长方向返回 3x3 邻域中的有效偏移。本结构用
 * `std::vector<cv::Point>` 替代裸数组，坐标单位为 pixel。
 */
struct LaserNeighborMask {
    /// @brief 邻域偏移，x/y 单位为 pixel。
    std::vector<cv::Point> offsets;

    /// @brief true 表示输入方向被旧函数支持。
    bool valid = false;
};

/**
 * @brief ImageProShare 中 LOG/CoM/离线点拖拽的 OpenCV 等价工具。
 *
 * 本类完整承接旧 `LOG_new`、`CoM_new`、`LOGCore`、`EfficientMask`
 * 和 `DragOfflinePt` 的可调用算法边界。输入统一为 `CV_8UC1 cv::Mat`
 * 和数值结构，不暴露裸指针、HALCON、Qt 或设备 SDK 类型。
 */
class CLaserLineResponseTools final {
public:
    /**
     * @brief 计算单个像素位置的旧三段权重 LOG 响应。
     *
     * 对应旧 `ImageProShare::LOGCore`。函数在同一列内按当前行的
     * `laserWidthByRow[yPixel]` 取 `3 * laserWidth` 个采样点，中间
     * 一段权重为 `+1`，上下两侧权重为 `-0.5`，最终除以激光宽度。
     *
     * @param grayImage 输入灰度图，必须为 `CV_8UC1`。
     * @param laserWidthByRow 每一行估计的激光宽度，单位 pixel，长度至少为图像行数。
     * @param xPixel 采样列坐标，单位 pixel。
     * @param yPixel 采样行坐标，单位 pixel。
     * @param status 可选状态输出；成功时为 `Succeeded`。
     * @return LOG 响应值；输入非法时返回 0。
     */
    static double calculateLogResponseAt(
        const cv::Mat &grayImage,
        const std::vector<int> &laserWidthByRow,
        int xPixel,
        int yPixel,
        VisionPreprocessStatus *status);

    /**
     * @brief 按旧 `LOG_new` 逻辑扫描列峰。
     *
     * 函数在指定 ROI 行范围内，以 `columnStepPixel` 为列步长扫描亮点，
     * 对满足 `requiredPeakGrayValue` 的像素计算 LOG 响应，并为每个采样列
     * 记录最大响应行。未采样列和未检出列均输出 -1，保持旧返回数组语义。
     *
     * @param grayImage 输入灰度图，必须为 `CV_8UC1`。
     * @param laserWidthByRow 每一行估计的激光宽度，单位 pixel。
     * @param options ROI、列步长和阈值参数。
     * @param status 可选状态输出。
     * @return 每列峰值行和响应值；输入非法时 `valid=false`。
     */
    static LaserLogPeakResult detectLogPeaks(
        const cv::Mat &grayImage,
        const std::vector<int> &laserWidthByRow,
        const LaserLogPeakOptions &options,
        VisionPreprocessStatus *status);

    /**
     * @brief 按旧 `CoM_new` 逻辑对 LOG 峰值行做灰度重心细化。
     *
     * 函数先在峰值行上下统计连续亮点数量，用该宽度修正局部中心行，
     * 再在局部窗口内按灰度加权求行方向重心偏移。该逻辑用于把整数峰
     * 行细化为亚像素风格的中心行，但不做 Hessian/Steger 或路径优化。
     *
     * @param grayImage 输入灰度图，必须为 `CV_8UC1`。
     * @param peakRowsByColumn `detectLogPeaks` 输出的每列峰值行，单位 pixel。
     * @param laserWidthByRow 每一行估计的激光宽度，单位 pixel。
     * @param laserWidthMaxPixel 上下搜索最大宽度，单位 pixel。
     * @param status 可选状态输出。
     * @return 每列重心行；输入非法时 `valid=false`。
     */
    static LaserCenterOfMassResult refineCenterOfMass(
        const cv::Mat &grayImage,
        const std::vector<int> &peakRowsByColumn,
        const std::vector<int> &laserWidthByRow,
        int laserWidthMaxPixel,
        VisionPreprocessStatus *status);

    /**
     * @brief 按旧 `DragOfflinePt` 逻辑在同一列上下搜索最大 LOG 响应点。
     *
     * 旧 `BSnake_HandleConPt` 会在控制点亮度不足时调用该逻辑，将控制点
     * 沿同一列拖拽到附近激光线。新实现保留“同列上下搜索 + 亮度阈值 +
     * 最大 LOG 响应选择”的行为，但不写调试图、不依赖旧全局路径。
     *
     * @param grayImage 输入灰度图，必须为 `CV_8UC1`。
     * @param laserWidthByRow 每一行估计的激光宽度，单位 pixel。
     * @param startPoint 待修正控制点，单位 pixel。
     * @param searchHalfHeightPixel 上下搜索半高，单位 pixel。
     * @param minGrayValue 候选点最低灰度值。
     * @param status 可选状态输出。
     * @return 拖拽后的点；输入非法时 `valid=false`。
     */
    static LaserDraggedPointResult dragPointToLaserLine(
        const cv::Mat &grayImage,
        const std::vector<int> &laserWidthByRow,
        const cv::Point2i &startPoint,
        int searchHalfHeightPixel,
        int minGrayValue,
        VisionPreprocessStatus *status);

    /**
     * @brief 返回旧 `EfficientMask` 支持的 3x3 方向邻域偏移。
     *
     * 该函数用于 `BSnakeGrow` 风格的像素跟踪：根据当前增长方向返回
     * 下一步应检查的邻域位置。未知方向返回 `valid=false`。
     *
     * @param offsetXPixel 当前增长方向 x 分量，取值通常为 -1、0、1。
     * @param offsetYPixel 当前增长方向 y 分量，取值通常为 -1、0、1。
     * @return 邻域偏移列表，单位 pixel。
     */
    static LaserNeighborMask buildEfficientMask(
        int offsetXPixel,
        int offsetYPixel);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_STRIPE_LASER_LINE_RESPONSE_TOOLS_H

