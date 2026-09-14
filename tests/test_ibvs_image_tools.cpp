/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_ibvs_image_tools.cpp
 * @brief       ibvs image tools 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <iostream>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "IbvsColorBlobTools.h"
#include "IbvsImageProjectionTools.h"

using namespace i2r_rlc::ibvs;

namespace {

int expect(bool condition, const char* message)
{
    if (condition)
    {
        return 0;
    }

    std::cerr << message << '\n';
    return 1;
}

int testColorBlobExtraction()
{
    cv::Mat image = cv::Mat::zeros(80, 80, CV_8UC3);
    cv::circle(image, cv::Point(24, 40), 6, cv::Scalar(0, 0, 255), cv::FILLED);
    cv::circle(image, cv::Point(52, 40), 6, cv::Scalar(0, 0, 255), cv::FILLED);

    IbvsColorBlobExtractionConfig config;
    config.applyGamma = false;
    config.applyBilateral = false;
    config.hsvRanges.push_back(
        IbvsColorRange{ cv::Scalar(0, 50, 50), cv::Scalar(15, 255, 255) });
    config.hsvRanges.push_back(
        IbvsColorRange{ cv::Scalar(160, 50, 50), cv::Scalar(180, 255, 255) });
    config.minBlobAreaPx = 20.0;
    config.maxBlobSizePx = 500.0;

    cv::Mat hsvImage;
    const cv::Mat mask =
        IbvsColorBlobTools::extractColorMask(image, config, &hsvImage);
    const std::vector<IbvsColorBlobCandidate> candidates =
        IbvsColorBlobTools::extractCandidates(mask, hsvImage, config);

    return expect(candidates.size() >= 2, "red blob extraction found too few candidates");
}

int testProjectionSampling()
{
    IbvsCameraIntrinsics intrinsics;
    intrinsics.fx = 100.0;
    intrinsics.fy = 100.0;
    intrinsics.cx = 50.0;
    intrinsics.cy = 40.0;

    const cv::Mat cameraMatrix =
        IbvsImageProjectionTools::makeCameraMatrix(intrinsics);
    const std::vector<std::vector<cv::Point3f>> edges = {
        { cv::Point3f(0.0f, 0.0f, 1000.0f),
          cv::Point3f(100.0f, 0.0f, 1000.0f) }
    };

    const std::vector<cv::Point2f> sampled =
        IbvsImageProjectionTools::projectAndSampleEdges(
            edges,
            cv::Vec3d(0.0, 0.0, 0.0),
            cv::Vec3d(0.0, 0.0, 0.0),
            cameraMatrix,
            1.0,
            2.0);

    if (const int failed = expect(!sampled.empty(), "projection sampling produced no points"))
    {
        return failed;
    }

    const cv::Point2f first = sampled.front();
    return expect(
        std::abs(first.x - 50.0f) < 0.1f && std::abs(first.y - 40.0f) < 0.1f,
        "projection sampling first point is not at the expected pixel");
}

} // namespace

int main()
{
    if (const int failed = testColorBlobExtraction())
    {
        return failed;
    }
    if (const int failed = testProjectionSampling())
    {
        return failed;
    }

    return 0;
}
