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
 * @file        ImagePreprocessor.h
 * @brief       图像预处理接口
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
#ifndef I2R_RLC_VISION_PREPROCESS_IMAGE_PREPROCESSOR_H
#define I2R_RLC_VISION_PREPROCESS_IMAGE_PREPROCESSOR_H

#include <opencv2/core.hpp>

#include "common/VisionImageTypes.h"

namespace i2r_rlc::vision {

/**
 * @brief 基于 `cv::Mat` 的基础图像预处理工具。
 *
 * 本类只处理调用方已经准备好的 OpenCV 图像，不读取文件，
 * 也不管理相机 SDK buffer 生命周期。
 */
class CImagePreprocessor final {
public:
    /**
     * @brief 判断输入图像是否是当前阶段支持的 OpenCV 图像。
     *
     * 支持 CV_8UC1、CV_8UC3、CV_8UC4、CV_16UC1 和 CV_32FC1。
     *
     * @param image 输入图像。
     * @return true 表示格式受支持且图像非空。
     */
    static bool isSupportedInput(const cv::Mat &image);

    /**
     * @brief 判断 ROI 是否完全位于图像范围内。
     *
     * @param image 输入图像，必须为非空 `cv::Mat`。
     * @param roi 待检查 ROI，单位 pixel。
     * @return true 表示 ROI 有正面积并且完全位于图像内。
     */
    static bool isValidRoi(const cv::Mat &image, const VisionImageRoi &roi);

    /**
     * @brief 按 ROI 裁剪图像并返回独立副本。
     *
     * 若输入无效或 ROI 越界，返回空 `cv::Mat`，并通过 status 返回原因。
     *
     * @param image 输入图像。
     * @param roi 待裁剪 ROI，坐标和尺寸单位为 pixel。
     * @param status 可选输出状态；函数不持有该指针。
     * @return ROI 独立副本，失败时为空。
     */
    static cv::Mat cropRoi(const cv::Mat &image,
                           const VisionImageRoi &roi,
                           VisionPreprocessStatus *status);

    /**
     * @brief 将图像转换为 8-bit 单通道灰度图。
     *
     * 彩色输入按 OpenCV 的 BGR/BGRA 约定转换。16-bit 或 32-bit 单通道输入
     * 会按当前图像最小/最大值归一化到 0~255。
     *
     * @param image 输入 OpenCV 图像。
     * @param status 可选输出状态；函数不持有该指针。
     * @return `CV_8UC1` 灰度图独立副本，失败时为空。
     */
    static cv::Mat toGray8(const cv::Mat &image,
                           VisionPreprocessStatus *status);

    /**
     * @brief 将单通道图像归一化为 8-bit 灰度图。
     *
     * 若输入已是 CV_8UC1，则返回独立副本。
     *
     * @param image 输入单通道图像。
     * @param status 可选输出状态；函数不持有该指针。
     * @return `CV_8UC1` 灰度图独立副本，失败时为空。
     */
    static cv::Mat normalizeToGray8(const cv::Mat &image,
                                    VisionPreprocessStatus *status);

    /**
     * @brief 计算 8-bit 灰度图统计值。
     *
     * 输入必须是非空 CV_8UC1。统计值单位为 gray value，范围 0~255。
     *
     * @param grayImage 输入 `CV_8UC1` 灰度图。
     * @param status 可选输出状态；函数不持有该指针。
     * @return 灰度统计结果。
     */
    static VisionGrayStatistics calculateGrayStatistics(
        const cv::Mat &grayImage,
        VisionPreprocessStatus *status);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_PREPROCESS_IMAGE_PREPROCESSOR_H

