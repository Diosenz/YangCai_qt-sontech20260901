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
 * @file        ImageMorphologyUtils.h
 * @brief       图像形态学基础工具
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
#ifndef I2R_RLC_VISION_PREPROCESS_IMAGE_MORPHOLOGY_UTILS_H
#define I2R_RLC_VISION_PREPROCESS_IMAGE_MORPHOLOGY_UTILS_H

#include <opencv2/core.hpp>

#include "common/VisionImageTypes.h"

namespace i2r_rlc::vision {

/**
 * @brief 基于 CV_8UC1 图像的基础形态学工具。
 *
 * 本类只处理调用方传入的 `cv::Mat`，不读取文件、不显示图像，
 * 也不依赖 Qt、HALCON 或 PCL。
 */
class CImageMorphologyUtils final {
public:
    /**
     * @brief 对灰度图执行 opening 运算。
     *
     * @param grayImage 输入 CV_8UC1 图像。
     * @param kernelWidthPixel 结构元素宽度，单位 pixel，必须为正数。
     * @param kernelHeightPixel 结构元素高度，单位 pixel，必须为正数。
     * @param status 可选输出状态。
     * @return 成功时返回独立输出图像；失败时返回空图。
     */
    static cv::Mat morphologyOpen(const cv::Mat &grayImage,
                                  int kernelWidthPixel,
                                  int kernelHeightPixel,
                                  VisionPreprocessStatus *status);

    /**
     * @brief 对灰度图执行 closing 运算。
     * @param grayImage 输入 `CV_8UC1` 灰度图。
     * @param kernelWidthPixel 结构元素宽度，单位 pixel，必须为正数。
     * @param kernelHeightPixel 结构元素高度，单位 pixel，必须为正数。
     * @param status 可选输出状态；函数不持有该指针。
     * @return 成功时返回独立输出图像；失败时返回空图。
     */
    static cv::Mat morphologyClose(const cv::Mat &grayImage,
                                   int kernelWidthPixel,
                                   int kernelHeightPixel,
                                   VisionPreprocessStatus *status);

    /**
     * @brief 对灰度图执行 erosion 运算。
     * @param grayImage 输入 `CV_8UC1` 灰度图。
     * @param kernelWidthPixel 结构元素宽度，单位 pixel，必须为正数。
     * @param kernelHeightPixel 结构元素高度，单位 pixel，必须为正数。
     * @param status 可选输出状态；函数不持有该指针。
     * @return 成功时返回独立输出图像；失败时返回空图。
     */
    static cv::Mat erodeGray(const cv::Mat &grayImage,
                             int kernelWidthPixel,
                             int kernelHeightPixel,
                             VisionPreprocessStatus *status);

    /**
     * @brief 对灰度图执行 dilation 运算。
     * @param grayImage 输入 `CV_8UC1` 灰度图。
     * @param kernelWidthPixel 结构元素宽度，单位 pixel，必须为正数。
     * @param kernelHeightPixel 结构元素高度，单位 pixel，必须为正数。
     * @param status 可选输出状态；函数不持有该指针。
     * @return 成功时返回独立输出图像；失败时返回空图。
     */
    static cv::Mat dilateGray(const cv::Mat &grayImage,
                              int kernelWidthPixel,
                              int kernelHeightPixel,
                              VisionPreprocessStatus *status);

    /**
     * @brief 删除二值图中的小连通域。
     *
     * 非零像素视为前景。保留面积大于等于 `minAreaPixel` 的连通域，
     * 输出仍为 CV_8UC1，前景值为 255。
     *
     * @param binaryImage 输入 `CV_8UC1` 二值图。
     * @param minAreaPixel 最小保留面积，单位 pixel。
     * @param status 可选输出状态；函数不持有该指针。
     * @return 小连通域过滤后的二值图。
     */
    static cv::Mat removeSmallComponents(const cv::Mat &binaryImage,
                                         int minAreaPixel,
                                         VisionPreprocessStatus *status);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_PREPROCESS_IMAGE_MORPHOLOGY_UTILS_H

