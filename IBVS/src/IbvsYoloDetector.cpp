/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsYoloDetector.cpp
 * @brief     IBVS YOLO/ONNX 焊枪 ROI 检测实现。
 * @details   本文件封装 ONNX Runtime 推理、输出解析和 ROI 非极大值抑制。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 8 日
 */

#include <algorithm>
#include <array>
#include <numeric>

#include <onnxruntime_cxx_api.h>
#include <opencv2/imgproc.hpp>

#include "ibvs/IbvsYoloDetector.h"

namespace i2r_rlc::ibvs {

class IbvsYoloDetector::Impl
{
public:
    Impl(const std::string& onnxPath, const IbvsYoloDetectorConfig& config);
    std::vector<IbvsYoloDetection> detect(const cv::Mat& bgrImage) const;

private:
    static float iou(const cv::Rect2f& first, const cv::Rect2f& second);

    std::unique_ptr<Ort::Env> m_env;
    std::unique_ptr<Ort::Session> m_session;
    std::string m_inputName;
    std::string m_outputName;
    IbvsYoloDetectorConfig m_config;
};

IbvsYoloDetector::Impl::Impl(
    const std::string& onnxPath,
    const IbvsYoloDetectorConfig& config)
    : m_config(config)
{
    m_env = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_ERROR, "ibvs_yolo");
    Ort::SessionOptions sessionOptions;
    sessionOptions.SetIntraOpNumThreads(1);
    sessionOptions.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);

#ifdef _WIN32
    const std::wstring widePath(onnxPath.begin(), onnxPath.end());
    m_session = std::make_unique<Ort::Session>(
        *m_env,
        widePath.c_str(),
        sessionOptions);
#else
    m_session = std::make_unique<Ort::Session>(
        *m_env,
        onnxPath.c_str(),
        sessionOptions);
#endif

    Ort::AllocatorWithDefaultOptions allocator;
    m_inputName = m_session->GetInputNameAllocated(0, allocator).get();
    m_outputName = m_session->GetOutputNameAllocated(0, allocator).get();
}

std::vector<IbvsYoloDetection> IbvsYoloDetector::Impl::detect(
    const cv::Mat& bgrImage) const
{
    if (bgrImage.empty())
    {
        return {};
    }

    const int originalWidth = bgrImage.cols;
    const int originalHeight = bgrImage.rows;
    cv::Mat image;
    cv::resize(bgrImage, image, cv::Size(m_config.imageSize, m_config.imageSize));
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
    image.convertTo(image, CV_32F, 1.0f / 255.0f);

    std::vector<float> input(
        static_cast<std::size_t>(3 * m_config.imageSize * m_config.imageSize));
    std::size_t inputIndex = 0;
    for (int channel = 0; channel < 3; ++channel)
    {
        for (int y = 0; y < m_config.imageSize; ++y)
        {
            for (int x = 0; x < m_config.imageSize; ++x)
            {
                input[inputIndex] = image.at<cv::Vec3f>(y, x)[channel];
                ++inputIndex;
            }
        }
    }

    const std::array<int64_t, 4> shape{
        1,
        3,
        m_config.imageSize,
        m_config.imageSize
    };
    Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(
        OrtArenaAllocator,
        OrtMemTypeDefault);
    Ort::Value tensor = Ort::Value::CreateTensor<float>(
        memoryInfo,
        input.data(),
        input.size(),
        shape.data(),
        shape.size());

    const char* inputNames[] = { m_inputName.c_str() };
    const char* outputNames[] = { m_outputName.c_str() };
    auto outputs = m_session->Run(
        Ort::RunOptions{ nullptr },
        inputNames,
        &tensor,
        1,
        outputNames,
        1);

    const float* output = outputs[0].GetTensorData<float>();
    const auto outputShape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    if (outputShape.size() < 3 || outputShape[2] < 6)
    {
        return {};
    }

    const int64_t detectionCount = outputShape[1];
    const int64_t stride = outputShape[2];
    std::vector<cv::Rect2f> boxes;
    std::vector<float> scores;
    for (int64_t i = 0; i < detectionCount; ++i)
    {
        const float* row = output + i * stride;
        const float objectness = row[4];
        if (objectness < m_config.confidenceThreshold)
        {
            continue;
        }

        const float confidence = objectness * row[5];
        if (confidence < m_config.confidenceThreshold)
        {
            continue;
        }

        const float cx = row[0];
        const float cy = row[1];
        const float width = row[2];
        const float height = row[3];
        boxes.push_back(cv::Rect2f(
            (cx - width * 0.5f) * originalWidth / m_config.imageSize,
            (cy - height * 0.5f) * originalHeight / m_config.imageSize,
            width * originalWidth / m_config.imageSize,
            height * originalHeight / m_config.imageSize));
        scores.push_back(confidence);
    }

    std::vector<int> indices(boxes.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(
        indices.begin(),
        indices.end(),
        [&scores](int first, int second)
        {
            return scores[static_cast<std::size_t>(first)]
                > scores[static_cast<std::size_t>(second)];
        });

    std::vector<IbvsYoloDetection> detections;
    while (!indices.empty()
        && static_cast<int>(detections.size()) < m_config.maxDetections)
    {
        const int current = indices.front();
        detections.push_back({
            boxes[static_cast<std::size_t>(current)],
            scores[static_cast<std::size_t>(current)]
        });

        std::vector<int> rest;
        for (std::size_t i = 1; i < indices.size(); ++i)
        {
            const int candidate = indices[i];
            if (iou(
                    boxes[static_cast<std::size_t>(current)],
                    boxes[static_cast<std::size_t>(candidate)])
                < m_config.iouThreshold)
            {
                rest.push_back(candidate);
            }
        }
        indices.swap(rest);
    }

    return detections;
}

float IbvsYoloDetector::Impl::iou(
    const cv::Rect2f& first,
    const cv::Rect2f& second)
{
    const float intersection = (first & second).area();
    return intersection / (first.area() + second.area() - intersection + 1e-6f);
}

IbvsYoloDetector::IbvsYoloDetector(
    const std::string& onnxPath,
    const IbvsYoloDetectorConfig& config)
    : m_impl(std::make_unique<Impl>(onnxPath, config))
{
}

IbvsYoloDetector::~IbvsYoloDetector() = default;

std::vector<IbvsYoloDetection> IbvsYoloDetector::detect(
    const cv::Mat& bgrImage) const
{
    return m_impl->detect(bgrImage);
}

} // namespace i2r_rlc::ibvs
