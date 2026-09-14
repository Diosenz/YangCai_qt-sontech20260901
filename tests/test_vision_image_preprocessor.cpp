/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_image_preprocessor.cpp
 * @brief       vision image preprocessor 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <cstdio>

#include "preprocess/ImagePreprocessor.h"

namespace {

bool expectTrue(bool condition)
{
    return condition;
}

bool expectNear(double actual, double expected, double tolerance)
{
    return std::fabs(actual - expected) <= tolerance;
}

} // namespace

int main()
{
    using i2r_rlc::vision::CImagePreprocessor;
    using i2r_rlc::vision::VisionGrayStatistics;
    using i2r_rlc::vision::VisionImageRoi;
    using i2r_rlc::vision::VisionPreprocessStatus;

    cv::Mat bgrImage(2, 2, CV_8UC3);
    bgrImage.at<cv::Vec3b>(0, 0) = cv::Vec3b(10, 20, 30);
    bgrImage.at<cv::Vec3b>(0, 1) = cv::Vec3b(40, 50, 60);
    bgrImage.at<cv::Vec3b>(1, 0) = cv::Vec3b(70, 80, 90);
    bgrImage.at<cv::Vec3b>(1, 1) = cv::Vec3b(100, 110, 120);

    VisionPreprocessStatus status = VisionPreprocessStatus::EmptyInput;
    const cv::Mat grayImage = CImagePreprocessor::toGray8(bgrImage, &status);
    if (!expectTrue(status == VisionPreprocessStatus::Succeeded)
        || !expectTrue(grayImage.type() == CV_8UC1)
        || !expectTrue(grayImage.rows == 2)
        || !expectTrue(grayImage.cols == 2))
    {
        return 1;
    }

    const VisionGrayStatistics statistics =
        CImagePreprocessor::calculateGrayStatistics(grayImage, &status);
    if (!expectTrue(status == VisionPreprocessStatus::Succeeded)
        || !expectTrue(statistics.pixelCount == 4)
        || !expectTrue(statistics.minGrayValue <= statistics.meanGrayValue)
        || !expectTrue(statistics.meanGrayValue <= statistics.maxGrayValue))
    {
        return 2;
    }

    const VisionImageRoi roi{1, 0, 1, 2};
    const cv::Mat croppedImage = CImagePreprocessor::cropRoi(grayImage, roi, &status);
    if (!expectTrue(status == VisionPreprocessStatus::Succeeded)
        || !expectTrue(croppedImage.rows == 2)
        || !expectTrue(croppedImage.cols == 1)
        || !expectTrue(croppedImage.type() == CV_8UC1)
        || !expectTrue(croppedImage.data != grayImage.data))
    {
        return 3;
    }

    const VisionImageRoi invalidRoi{1, 1, 5, 1};
    const cv::Mat invalidCrop = CImagePreprocessor::cropRoi(grayImage, invalidRoi, &status);
    if (!expectTrue(status == VisionPreprocessStatus::InvalidRoi)
        || !expectTrue(invalidCrop.empty()))
    {
        return 4;
    }

    cv::Mat depthImage(1, 3, CV_16UC1);
    depthImage.at<unsigned short>(0, 0) = 100;
    depthImage.at<unsigned short>(0, 1) = 200;
    depthImage.at<unsigned short>(0, 2) = 300;

    const cv::Mat normalizedImage =
        CImagePreprocessor::normalizeToGray8(depthImage, &status);
    if (!expectTrue(status == VisionPreprocessStatus::Succeeded)
        || !expectTrue(normalizedImage.type() == CV_8UC1)
        || !expectTrue(normalizedImage.at<unsigned char>(0, 0) == 0)
        || !expectNear(normalizedImage.at<unsigned char>(0, 2), 255.0, 1.0))
    {
        return 5;
    }

    const cv::Mat emptyGray = CImagePreprocessor::toGray8(cv::Mat(), &status);
    if (!expectTrue(status == VisionPreprocessStatus::EmptyInput)
        || !expectTrue(emptyGray.empty()))
    {
        return 6;
    }

    std::printf("PASS: vision image preprocessor tests\n");
    return 0;
}
