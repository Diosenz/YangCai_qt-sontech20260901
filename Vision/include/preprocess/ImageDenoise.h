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
 * @file        ImageDenoise.h
 * @brief       图像去噪与增强接口
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
#ifndef I2R_RLC_VISION_PREPROCESS_IMAGE_DENOISE_H
#define I2R_RLC_VISION_PREPROCESS_IMAGE_DENOISE_H

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "common/VisionImageTypes.h"
#include "preprocess/GaborFilterTypes.h"

namespace i2r_rlc::vision {

/**
 * @brief Gabor 降噪滤波显式参数。
 *
 * 参数直接传给 OpenCV `cv::getGaborKernel`，角度单位为 rad。
 * 本结构不持有仿真线、全局状态或旧工程调试路径。
 */
struct GaborDenoiseOptions {
    /// @brief kernel 宽度，单位 pixel，必须为正奇数。
    int kernelWidthPixel = 11;

    /// @brief kernel 高度，单位 pixel，必须为正奇数。
    int kernelHeightPixel = 11;

    /// @brief 高斯包络 sigma，必须大于 0。
    double sigma = 3.0;

    /// @brief Gabor 方向角，单位 rad。
    double thetaRad = 0.0;

    /// @brief 正弦波波长，单位 pixel，必须大于 0。
    double lambdaPixel = 9.0;

    /// @brief 空间纵横比，必须大于 0。
    double gamma = 0.5;

    /// @brief 相位偏移，单位 rad。
    double psiRad = 0.0;
};

/**
 * @brief GaborFilter 兼容路径的参数。
 *
 * 默认值对齐旧工程 `Process_P` 路径：kernel_size=21、pos_sigma=4、
 * pos_lm=90、pos_psi=90。`posLm` 是旧工程百分值参数，实际 lambda 为
 * `0.5 + posLm / 100.0`。
 */
struct GaborFilterOptions {
    /// @brief kernel 尺寸，单位 pixel，必须为正奇数。
    int kernelSizePixel = 21;

    /// @brief 无量纲 sigma 参数，进入 mkKernel 前会除以 kernelSize。
    double posSigma = 4.0;

    /// @brief  lambda 参数，实际 lambda = 0.5 + posLm / 100.0。
    double posLm = 90.0;

    /// @brief 相位参数，单位 degree。
    double posPsiDeg = 90.0;

    /// @brief 焊点左右参与均值估计的点数窗口。
    int neighborWindow = 10;

    /// @brief 判断退化分母的阈值，单位 pixel。
    double denominatorEpsilon = 1.0e-9;
};

/**
 * @brief 根据仿真点和焊点估计 GaborFilter 滤波方向的结果。
 */
struct GaborFilterAngleResult {
    /// @brief true 表示角度估计成功。
    bool valid = false;

    /// @brief 旧 `pos_th`，单位 degree。
    double filterAngleDeg = 0.0;

    /// @brief 与焊点 X 坐标最近的仿真点索引。
    int weldPointIndex = -1;

    /// @brief 状态说明，不使用平台相关字符串类型。
    std::string message;
};

/**
 * @brief 基于 CV_8UC1 图像的基础降噪工具。
 *
 * 本类只接收 `cv::Mat`，不读取文件、不写调试图、不依赖 Qt、HALCON 或 PCL。
 */
class CImageDenoise final {
public:
    /**
     * @brief 中值滤波降噪。
     *
     * @param grayImage 输入灰度图，必须为 CV_8UC1。
     * @param kernelSizePixel 中值核尺寸，单位 pixel，必须为正奇数。
     * @param status 可选输出状态，允许为 `nullptr`。
     * @return 滤波后的 CV_8UC1 图像；失败时返回空矩阵。
     */
    static cv::Mat medianDenoise(const cv::Mat &grayImage,
                                 int kernelSizePixel,
                                 VisionPreprocessStatus *status);

    /**
     * @brief 高斯滤波降噪。
     *
     * @param grayImage 输入灰度图，必须为 CV_8UC1。
     * @param kernelSizePixel 高斯核尺寸，单位 pixel，必须为正奇数。
     * @param sigma 高斯 sigma，必须大于 0。
     * @param status 可选输出状态，允许为 `nullptr`。
     * @return 滤波后的 CV_8UC1 图像；失败时返回空矩阵。
     */
    static cv::Mat gaussianDenoise(const cv::Mat &grayImage,
                                   int kernelSizePixel,
                                   double sigma,
                                   VisionPreprocessStatus *status);

    /**
     * @brief 双边滤波降噪。
     *
     * @param grayImage 输入灰度图，必须为 CV_8UC1。
     * @param diameterPixel 双边滤波邻域直径，单位 pixel，必须大于 0。
     * @param sigmaColor 颜色空间 sigma，必须大于 0。
     * @param sigmaSpace 坐标空间 sigma，必须大于 0。
     * @param status 可选输出状态，允许为 `nullptr`。
     * @return 滤波后的 CV_8UC1 图像；失败时返回空矩阵。
     */
    static cv::Mat bilateralDenoise(const cv::Mat &grayImage,
                                    int diameterPixel,
                                    double sigmaColor,
                                    double sigmaSpace,
                                    VisionPreprocessStatus *status);

    /**
     * @brief 使用显式 Gabor 参数执行方向性滤波。
     *
     * @param grayImage 输入灰度图，必须为 CV_8UC1。
     * @param options OpenCV Gabor 参数，角度单位为 rad。
     * @param status 可选输出状态，允许为 `nullptr`。
     * @return 滤波后的 CV_8UC1 图像；失败时返回空矩阵。
     */
    static cv::Mat gaborDenoise(const cv::Mat &grayImage,
                                const GaborDenoiseOptions &options,
                                VisionPreprocessStatus *status);

    /**
     * @brief 获取 GaborFilter 兼容路径的当前参数。
     *
     * @return 当前参数副本，单位见 `GaborFilterOptions` 字段说明。
     */
    static GaborFilterOptions gaborFilterOptions();

    /**
     * @brief 设置 GaborFilter 兼容路径的参数。
     *
     * @param options 下游 GaborFilter 兼容接口使用的参数。
     *
     * 参数在下游调用基于仿真点的 Gabor 接口时生效；
     */
    static void setGaborFilterOptions(const GaborFilterOptions &options);

    /**
     * @brief 从 SimPts 与 WeldPt 估计 Gabor 方向。
     *
     * @param simPoints 仿真激光图像点，单位 pixel。
     * @param weldPoint 焊缝点，单位 pixel。
     * @return 方向估计结果，`filterAngleDeg` 单位为 degree。
     */
    static GaborFilterAngleResult estimateGaborFilterAngle(
        const std::vector<MyLaserImagePoint> &simPoints,
        const MyLaserImagePoint &weldPoint);

    /**
     * @brief 构造 `mkKernel -> Process_P` 路径使用的 Gabor kernel。
     *
     * @param filterAngleDeg 滤波方向角，单位 degree。
     * @param options GaborFilter 参数。
     * @param status 可选输出状态，允许为 `nullptr`。
     * @return CV_32F kernel；参数非法时返回空矩阵。
     */
    static cv::Mat makeGaborFilterKernel(
        double filterAngleDeg,
        const GaborFilterOptions &options,
        VisionPreprocessStatus *status);

    /**
     * @brief 使用仿真点和焊缝点估计方向后执行 Gabor 滤波。
     *
     * @param grayImage 输入灰度图，必须为 CV_8UC1。
     * @param simPoints 仿真激光图像点，单位 pixel。
     * @param weldPoint 焊缝点，单位 pixel。
     * @param status 可选输出状态，允许为 `nullptr`。
     * @return 滤波后的 CV_8UC1 图像；失败时返回空矩阵。
     *
     */
    static cv::Mat gaborDenoise(
        const cv::Mat &grayImage,
        const std::vector<MyLaserImagePoint> &simPoints,
        const MyLaserImagePoint &weldPoint,
        VisionPreprocessStatus *status);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_PREPROCESS_IMAGE_DENOISE_H

