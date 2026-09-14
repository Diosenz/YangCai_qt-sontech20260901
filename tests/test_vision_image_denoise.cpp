/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_image_denoise.cpp
 * @brief       vision image denoise 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cstdio>

#include <opencv2/imgproc.hpp>

#include "preprocess/ImageDenoise.h"
#include "preprocess/ImageMorphologyUtils.h"

namespace {

bool expectTrue(bool condition)
{
    return condition;
}

bool hasOutputShape(const cv::Mat &image, int rows, int cols)
{
    return !image.empty()
        && image.type() == CV_8UC1
        && image.rows == rows
        && image.cols == cols;
}

} // namespace

int main()
{
    using i2r_rlc::vision::GaborDenoiseOptions;
    using i2r_rlc::vision::CImageDenoise;
    using i2r_rlc::vision::CImageMorphologyUtils;
    using i2r_rlc::vision::VisionPreprocessStatus;

    VisionPreprocessStatus status = VisionPreprocessStatus::Succeeded;
    const cv::Mat emptyMedian = CImageDenoise::medianDenoise(
        cv::Mat(),
        3,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::EmptyInput)
        || !expectTrue(emptyMedian.empty()))
    {
        return 1;
    }

    cv::Mat noisyImage = cv::Mat::zeros(7, 7, CV_8UC1);
    noisyImage.at<unsigned char>(3, 3) = 255;
    noisyImage.at<unsigned char>(0, 0) = 255;

    const cv::Mat medianImage = CImageDenoise::medianDenoise(
        noisyImage,
        3,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::Succeeded)
        || !expectTrue(hasOutputShape(medianImage, 7, 7)))
    {
        return 2;
    }

    const cv::Mat gaussianImage = CImageDenoise::gaussianDenoise(
        noisyImage,
        3,
        1.0,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::Succeeded)
        || !expectTrue(hasOutputShape(gaussianImage, 7, 7)))
    {
        return 3;
    }

    const cv::Mat bilateralImage = CImageDenoise::bilateralDenoise(
        noisyImage,
        3,
        30.0,
        30.0,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::Succeeded)
        || !expectTrue(hasOutputShape(bilateralImage, 7, 7)))
    {
        return 4;
    }

    cv::Mat openInput = cv::Mat::zeros(9, 9, CV_8UC1);
    cv::rectangle(openInput, cv::Rect(1, 1, 3, 3), cv::Scalar(255), -1);
    openInput.at<unsigned char>(7, 7) = 255;

    const cv::Mat openedImage = CImageMorphologyUtils::morphologyOpen(
        openInput,
        3,
        3,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::Succeeded)
        || !expectTrue(hasOutputShape(openedImage, 9, 9))
        || !expectTrue(openedImage.at<unsigned char>(7, 7) == 0)
        || !expectTrue(openedImage.at<unsigned char>(2, 2) == 255))
    {
        return 5;
    }

    cv::Mat closeInput(5, 5, CV_8UC1, cv::Scalar(255));
    closeInput.at<unsigned char>(2, 2) = 0;
    const cv::Mat closedImage = CImageMorphologyUtils::morphologyClose(
        closeInput,
        3,
        3,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::Succeeded)
        || !expectTrue(hasOutputShape(closedImage, 5, 5))
        || !expectTrue(closedImage.at<unsigned char>(2, 2) == 255))
    {
        return 6;
    }

    cv::Mat componentInput = cv::Mat::zeros(8, 8, CV_8UC1);
    componentInput.at<unsigned char>(0, 0) = 255;
    cv::rectangle(componentInput, cv::Rect(3, 3, 3, 3), cv::Scalar(255), -1);
    const cv::Mat componentImage =
        CImageMorphologyUtils::removeSmallComponents(
            componentInput,
            4,
            &status);
    if (!expectTrue(status == VisionPreprocessStatus::Succeeded)
        || !expectTrue(hasOutputShape(componentImage, 8, 8))
        || !expectTrue(componentImage.at<unsigned char>(0, 0) == 0)
        || !expectTrue(componentImage.at<unsigned char>(4, 4) == 255))
    {
        return 7;
    }

    const cv::Mat erodedImage = CImageMorphologyUtils::erodeGray(
        closeInput,
        3,
        3,
        &status);
    const cv::Mat dilatedImage = CImageMorphologyUtils::dilateGray(
        noisyImage,
        3,
        3,
        &status);
    if (!expectTrue(hasOutputShape(erodedImage, 5, 5))
        || !expectTrue(hasOutputShape(dilatedImage, 7, 7)))
    {
        return 8;
    }

    GaborDenoiseOptions gaborOptions;
    gaborOptions.kernelWidthPixel = 11;
    gaborOptions.kernelHeightPixel = 11;
    gaborOptions.sigma = 3.0;
    gaborOptions.thetaRad = 0.0;
    gaborOptions.lambdaPixel = 9.0;
    gaborOptions.gamma = 0.5;
    gaborOptions.psiRad = 0.0;

    const cv::Mat gaborImage = CImageDenoise::gaborDenoise(
        noisyImage,
        gaborOptions,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::Succeeded)
        || !expectTrue(hasOutputShape(gaborImage, 7, 7)))
    {
        return 9;
    }

    const cv::Mat invalidOpen = CImageMorphologyUtils::morphologyOpen(
        noisyImage,
        0,
        3,
        &status);
    if (!expectTrue(status == VisionPreprocessStatus::InvalidKernel)
        || !expectTrue(invalidOpen.empty()))
    {
        return 10;
    }

    std::printf("PASS: vision image denoise tests\n");
    return 0;
}
