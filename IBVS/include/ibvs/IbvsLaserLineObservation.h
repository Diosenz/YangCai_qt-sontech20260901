/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsLaserLineObservation.h
 * @brief     IBVS 模块自有的线结构光观测提取入口。
 * @details   本文件定义从虚拟激光线先验提取真实中心线并生成 LVS 观测的值对象。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 17 日
 */

#ifndef I2R_RLC_IBVS_IBVS_LASER_LINE_OBSERVATION_H
#define I2R_RLC_IBVS_IBVS_LASER_LINE_OBSERVATION_H

#include <string>
#include <vector>

#ifdef I2R_RLC_ENABLE_OPENCV
#include <opencv2/core.hpp>
#endif

#include "ibvs/IbvsSensorFusion.h"
#include "ibvs/IbvsTypes.h"

namespace i2r_rlc::ibvs {

/**
 * @brief IBVS 模块内部的虚拟激光线先验。
 */
struct IbvsLaserPrior
{
    int frameIndex = -1;
    std::vector<PixelPoint> virtualLaserLine;
    bool hasVirtualWeldPoint = false;
    PixelPoint virtualWeldPoint;
    int roiExpandPx = 250;
    double roiTopScale = 0.6;
    double roiBottomScale = 0.1;
};

/**
 * @brief IBVS 激光中心线提取参数。
 */
struct IbvsLaserExtractionOptions
{
    int intensityThreshold = 64;
    int gaussianKernelSize = 5;
    double gaussianSigma = 6.0;
    bool enableGabor = true;
    int gaborKernelSize = 11;
    double gaborSigma = 3.0;
    double gaborLambda = 9.0;
    double gaborPsi = 0.0;
    double gaborGamma = 1.0;
};

/**
 * @brief IBVS 激光观测提取结果。
 */
struct IbvsLaserObservationResult
{
    IbvsError error;
    PixelRoi roi;
    std::vector<PixelPoint> centerline;
    IbvsLvsObservation observation;
    double centerlineQuality = 0.0;
    std::string message;
};

#ifdef I2R_RLC_ENABLE_OPENCV
/**
 * @brief IBVS 模块内部的线结构光中心线提取器。
 *
 * 该类不依赖 Localization 或 Vision，算法步骤与定位模块保持一致，输出
 * IBVS 自有的 `IbvsLvsObservation`。
 */
class IbvsLaserLineExtractor
{
public:
    /**
     * @brief 从灰度或彩色图像提取中心线并构造 LVS 观测。
     */
    static IbvsLaserObservationResult extractObservation(
        const cv::Mat& image,
        const IbvsLaserPrior& prior,
        const IbvsLaserExtractionOptions& options);
};
#endif

} // namespace i2r_rlc::ibvs

#endif // I2R_RLC_IBVS_IBVS_LASER_LINE_OBSERVATION_H
