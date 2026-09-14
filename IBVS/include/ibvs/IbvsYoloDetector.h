/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsYoloDetector.h
 * @brief     IBVS 焊枪 ROI 的 YOLO/ONNX 检测器。
 * @details   本文件定义工件图像中的焊枪 ROI 候选检测接口。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 8 日
 */

#ifndef I2R_RLC_IBVS_IBVS_YOLO_DETECTOR_H
#define I2R_RLC_IBVS_IBVS_YOLO_DETECTOR_H

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace i2r_rlc::ibvs {

/**
 * @brief YOLO 输出的焊枪 ROI 候选。
 */
struct IbvsYoloDetection
{
    cv::Rect2f box;
    float score = 0.0f;
};

/**
 * @brief YOLO/ONNX 推理参数。
 */
struct IbvsYoloDetectorConfig
{
    int imageSize = 640;
    float confidenceThreshold = 0.3f;
    float iouThreshold = 0.5f;
    int maxDetections = 1;
};

/**
 * @brief ONNX Runtime YOLO wrapper，输出焊枪 ROI 候选。
 */
class IbvsYoloDetector
{
public:
    /**
     * @brief 加载 ONNX 模型并创建焊枪 ROI 检测器。
     */
    IbvsYoloDetector(
        const std::string& onnxPath,
        const IbvsYoloDetectorConfig& config);
    ~IbvsYoloDetector();

    IbvsYoloDetector(const IbvsYoloDetector& other) = delete;
    IbvsYoloDetector& operator=(const IbvsYoloDetector& other) = delete;

    /**
     * @brief 在 BGR 图像中检测焊枪 ROI 候选。
     */
    std::vector<IbvsYoloDetection> detect(const cv::Mat& bgrImage) const;

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace i2r_rlc::ibvs

#endif // I2R_RLC_IBVS_IBVS_YOLO_DETECTOR_H
