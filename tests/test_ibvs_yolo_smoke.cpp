/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_ibvs_yolo_smoke.cpp
 * @brief       ibvs yolo smoke 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "ibvs/IbvsYoloDetector.h"

#ifndef I2R_RLC_IBVS_YOLO_MODEL_PATH
#define I2R_RLC_IBVS_YOLO_MODEL_PATH ""
#endif

namespace {

using namespace i2r_rlc::ibvs;

bool isValidDetection(
    const IbvsYoloDetection& detection,
    const cv::Size& imageSize)
{
    if (!std::isfinite(detection.score)
        || detection.score < 0.0f
        || detection.score > 1.0f)
    {
        return false;
    }

    if (detection.box.width <= 0.0f || detection.box.height <= 0.0f)
    {
        return false;
    }

    const cv::Rect2f imageBounds(
        0.0f,
        0.0f,
        static_cast<float>(imageSize.width),
        static_cast<float>(imageSize.height));
    return (detection.box & imageBounds).area() > 0.0f;
}

void printDetectionSummary(
    const std::filesystem::path& imagePath,
    const std::vector<IbvsYoloDetection>& detections)
{
    std::cout << "Image: " << imagePath.string() << '\n';
    std::cout << "Detection count: " << detections.size() << '\n';
    if (!detections.empty())
    {
        const IbvsYoloDetection& top = detections.front();
        std::cout << "Top score: " << top.score << '\n';
        std::cout << "Top ROI: x=" << top.box.x
                  << ", y=" << top.box.y
                  << ", w=" << top.box.width
                  << ", h=" << top.box.height << '\n';
    }
}

} // namespace

/**
 * @brief IBVS YOLO wrapper 的最小模型推理冒烟测试入口。
 *
 * 该测试先用黑图验证模型加载和推理链路不崩溃，再用本机真实离线
 * 图像验证至少一帧可产生合法 ROI。真实数据缺失时跳过，避免影响
 * 没有 `C:\Feature_detection` 的环境。
 */
int main()
{
    const std::filesystem::path modelPath(I2R_RLC_IBVS_YOLO_MODEL_PATH);
    if (modelPath.empty() || !std::filesystem::exists(modelPath))
    {
        std::cout << "SKIP: IBVS YOLO model file does not exist: "
                  << modelPath.string() << '\n';
        return 0;
    }

    try
    {
        IbvsYoloDetectorConfig config;
        config.imageSize = 640;
        config.confidenceThreshold = 0.3f;
        config.iouThreshold = 0.5f;
        config.maxDetections = 5;

        IbvsYoloDetector detector(
            modelPath.string(),
            config);
        const cv::Mat blackImage(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
        const auto blackDetections = detector.detect(blackImage);
        std::cout << "Black image detection count: "
                  << blackDetections.size() << '\n';

        const std::vector<std::filesystem::path> imagePaths{
            "C:/Feature_detection/FeatureDetection/FeatureDetection/Desired_image/Images_0100.jpg",
            "C:/Feature_detection/FeatureDetection/FeatureDetection/Images/Images_0100.jpg",
            "C:/Feature_detection/FeatureDetection/FeatureDetection/Orthogonal/Images/Images_00.jpg"
        };

        std::vector<std::filesystem::path> existingImages;
        for (const std::filesystem::path& imagePath : imagePaths)
        {
            if (std::filesystem::exists(imagePath))
            {
                existingImages.push_back(imagePath);
            }
        }

        if (existingImages.empty())
        {
            std::cout << "SKIP: no IBVS YOLO real test images found\n";
            return 0;
        }

        bool hasValidRealDetection = false;
        for (const std::filesystem::path& imagePath : existingImages)
        {
            const cv::Mat image = cv::imread(imagePath.string(), cv::IMREAD_COLOR);
            if (image.empty())
            {
                std::cerr << "FAIL: cannot read YOLO real test image: "
                          << imagePath.string() << '\n';
                return 1;
            }

            const auto detections = detector.detect(image);
            printDetectionSummary(imagePath, detections);
            for (const IbvsYoloDetection& detection : detections)
            {
                if (!isValidDetection(detection, image.size()))
                {
                    std::cerr << "FAIL: invalid YOLO detection for image: "
                              << imagePath.string() << '\n';
                    return 1;
                }
            }

            if (!detections.empty())
            {
                hasValidRealDetection = true;
            }
        }

        if (!hasValidRealDetection)
        {
            std::cerr << "FAIL: YOLO produced no detections on available real images\n";
            return 1;
        }

        std::cout << "PASS: IBVS YOLO smoke test\n";
        std::cout << "Model: " << modelPath.string() << '\n';
        std::cout << "Real images tested: " << existingImages.size() << '\n';
    }
    catch (const std::exception &exception)
    {
        std::cerr << "FAIL: IBVS YOLO smoke test exception: "
                  << exception.what() << '\n';
        return 1;
    }

    return 0;
}
