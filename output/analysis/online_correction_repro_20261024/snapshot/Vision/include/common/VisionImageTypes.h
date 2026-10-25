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
 * @file        VisionImageTypes.h
 * @brief       视觉图像基础数据类型
 * @details     定义 vision 模块共享的图像 ROI、灰度统计和预处理状态类型。
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
#ifndef I2R_RLC_VISION_COMMON_VISION_IMAGE_TYPES_H
#define I2R_RLC_VISION_COMMON_VISION_IMAGE_TYPES_H

#include <opencv2/core.hpp>

namespace i2r_rlc::vision {

/**
 * @brief vision 算法使用的像素级 ROI。
 *
 * 本结构只表达 `cv::Mat` 图像内的矩形区域，单位为 pixel。
 * `vision` 不负责文件读取或相机 SDK buffer 转换。
 */
struct VisionImageRoi {
    /// @brief ROI 左上角 X 坐标，单位 pixel。
    int xPixel = 0;

    /// @brief ROI 左上角 Y 坐标，单位 pixel。
    int yPixel = 0;

    /// @brief ROI 宽度，单位 pixel。
    int widthPixel = 0;

    /// @brief ROI 高度，单位 pixel。
    int heightPixel = 0;
};

/**
 * @brief 灰度图统计值。
 *
 * 所有 gray value 均按 0~255 的 8-bit 灰度约定表达。
 */
struct VisionGrayStatistics {
    /// @brief 参与统计的像素数量。
    int pixelCount = 0;

    /// @brief 灰度均值，范围 0~255。
    double meanGrayValue = 0.0;

    /// @brief 灰度最小值，范围 0~255。
    double minGrayValue = 0.0;

    /// @brief 灰度最大值，范围 0~255。
    double maxGrayValue = 0.0;
};

/**
 * @brief 基础预处理状态码。
 */
enum class VisionPreprocessStatus {
    Succeeded,
    EmptyInput,
    UnsupportedImageType,
    InvalidRoi,
    InvalidGrayImage,
    InvalidKernel,
    InvalidParameter,
    InsufficientPoints,
    DegenerateGeometry
};

/**
 * @brief 判断 ROI 是否有正面积。
 */
bool hasPositiveArea(const VisionImageRoi &roi);

/**
 * @brief 将 vision ROI 转换为 OpenCV 矩形。
 */
cv::Rect toCvRect(const VisionImageRoi &roi);

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_COMMON_VISION_IMAGE_TYPES_H

