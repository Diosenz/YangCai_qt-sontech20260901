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
 * @file        DirectionalMorphologyFilter.h
 * @brief       方向形态学增强接口
 * @details     提供灰度图像校验、增强、形态学和质量评价能力，供条纹、特征和管线模块复用。
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
#ifndef I2R_RLC_VISION_PREPROCESS_DIRECTIONAL_MORPHOLOGY_FILTER_H
#define I2R_RLC_VISION_PREPROCESS_DIRECTIONAL_MORPHOLOGY_FILTER_H

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace i2r_rlc::vision {

/**
 * @brief 方向形态学增强参数。
 *
 * 该结构用于复刻旧主链路中由仿真激光线方向构造左右结构元素、
 * 再对图像分区做灰度腐蚀和差分增强的预处理流程。长度单位为 pixel。
 */
struct DirectionalMorphologyOptions {
    /// @brief 左侧线形结构元素长度，单位 pixel。
    int leftLengthPixel = 31;

    /// @brief 右侧线形结构元素长度，单位 pixel。
    int rightLengthPixel = 31;

    /// @brief 均值滤波核尺寸，单位 pixel，自动修正为奇数。
    int blurSizePixel = 7;

    /// @brief 计算左右方向时的最少仿真线点数。
    int minLaserPointCount = 5;
};

/**
 * @brief 方向形态学增强结果。
 */
struct DirectionalMorphologyResult {
    /// @brief true 表示增强图像可用于下游骨架提取。
    bool valid = false;

    /// @brief 增强后的 CV_8UC1 图像。
    cv::Mat enhancedImage;

    /// @brief 左侧结构元素，CV_8UC1，前景值为 255。
    cv::Mat leftKernel;

    /// @brief 右侧结构元素，CV_8UC1，前景值为 255。
    cv::Mat rightKernel;

    /// @brief 仿真线分割点，单位 pixel。
    cv::Point2d splitPoint;

    /// @brief 仿真线分割点在输入点列中的索引。
    int splitPointIndex = -1;

    /// @brief 状态说明。
    std::string message;
};

/**
 * @brief 方向形态学预处理的 OpenCV 实现。
 *
 * 本类替代旧 `ConstructLineSE`、`ConstructMorSEFromSimLaser` 和
 * `ImageNonuniformEro`。输入输出均为 `cv::Mat` 与数值结构。
 */
class CDirectionalMorphologyFilter final {
public:
    /**
     * @brief 根据角度和长度构造线形结构元素。
     *
     * @param angleDegree 方向角，单位 degree。
     * @param lengthPixel 线段长度，单位 pixel。
     * @return CV_8UC1 结构元素；参数非法时返回空图。
     */
    static cv::Mat createLineKernel(double angleDegree, int lengthPixel);

    /**
     * @brief 根据仿真激光线生成左右结构元素和分割点。
     *
     * 分割点沿用旧工程“点到首尾连线距离最大”的语义，通常对应 T 型
     * 仿真线的焊缝附近弯折点。
     *
     * @param laserPoints 仿真激光线点列，坐标单位 pixel。
     * @param options 方向形态学参数。
     * @return 结构元素、分割点和状态信息。
     */
    static DirectionalMorphologyResult buildFromSimulatedLaser(
        const std::vector<cv::Point2d> &laserPoints,
        const DirectionalMorphologyOptions &options);

    /**
     * @brief 执行左右分区的方向腐蚀差分增强。
     *
     * 输入必须为 `CV_8UC1`。输出为 `CV_8UC1`，用于下游阈值、骨架和匹配。
     *
     * @param grayImage 输入 `CV_8UC1` 灰度图。
     * @param laserPoints 仿真激光线点列，坐标单位 pixel。
     * @param options 方向形态学参数。
     * @return 增强图像、结构元素、分割点和状态信息。
     */
    static DirectionalMorphologyResult enhance(
        const cv::Mat &grayImage,
        const std::vector<cv::Point2d> &laserPoints,
        const DirectionalMorphologyOptions &options);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_PREPROCESS_DIRECTIONAL_MORPHOLOGY_FILTER_H

