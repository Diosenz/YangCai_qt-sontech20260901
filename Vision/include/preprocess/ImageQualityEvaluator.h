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
 * @file        ImageQualityEvaluator.h
 * @brief       图像质量评价接口
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
#ifndef I2R_RLC_VISION_PREPROCESS_IMAGE_QUALITY_EVALUATOR_H
#define I2R_RLC_VISION_PREPROCESS_IMAGE_QUALITY_EVALUATOR_H

#include <map>
#include <string>

#include <opencv2/core.hpp>

namespace i2r_rlc::vision {

/**
 * @brief 图像质量指标。
 */
enum class ImageQualityMetric {
    /// @brief 灰度标准差/离散度。
    Deviation = 0,
    /// @brief Laplace 清晰度。
    Laplace = 1,
    /// @brief 梯度能量。
    Energy = 2,
    /// @brief Brenner 清晰度。
    Brenner = 3,
    /// @brief Tenegrad 梯度清晰度。
    Tenegrad = 4
};

/**
 * @brief 图像质量评价参数。
 */
struct ImageQualityOptions {
    /// @brief 是否限制 ROI。
    bool useRoi = false;

    /// @brief 可选 ROI，单位 pixel。
    cv::Rect roi;
};

/**
 * @brief 单项图像质量评价结果。
 */
struct ImageQualityResult {
    /// @brief true 表示评价成功。
    bool valid = false;

    /// @brief 指标类型。
    ImageQualityMetric metric = ImageQualityMetric::Deviation;

    /// @brief 指标值；不同指标量纲与旧实现一致。
    double value = 0.0;

    /// @brief 状态说明。
    std::string message;
};

/**
 * @brief 图像质量评价工具。
 *
 * 对应旧 `ImageEvaluationDef` 中的 Deviation、Laplace、Energy、Brenner、
 * Tenegrad。输入必须是 `CV_8UC1`，输出只做数值评价，不参与 UI 显示。
 */
class CImageQualityEvaluator final {
public:
    /**
     * @brief 计算单项质量指标。
     * @param gray8 输入 `CV_8UC1` 灰度图。
     * @param metric 要计算的质量指标。
     * @param options ROI 等评价参数。
     * @return 单项质量评价结果；输入非法时 `valid=false`。
     */
    static ImageQualityResult evaluate(
        const cv::Mat &gray8,
        ImageQualityMetric metric,
        const ImageQualityOptions &options);

    /**
     * @brief 一次性计算全部已迁移质量指标。
     * @param gray8 输入 `CV_8UC1` 灰度图。
     * @param options ROI 等评价参数。
     * @return 指标到评价结果的映射。
     */
    static std::map<ImageQualityMetric, ImageQualityResult> evaluateAll(
        const cv::Mat &gray8,
        const ImageQualityOptions &options);

    /**
     * @brief 返回指标英文 key。
     * @param metric 指标枚举。
     * @return 稳定英文 key，用于日志、CSV 和测试断言。
     */
    static std::string metricToKey(ImageQualityMetric metric);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_PREPROCESS_IMAGE_QUALITY_EVALUATOR_H

