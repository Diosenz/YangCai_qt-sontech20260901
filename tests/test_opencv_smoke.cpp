/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_opencv_smoke.cpp
 * @brief       opencv smoke 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

/**
 * @brief OpenCV 3.4.3 CMake 接入冒烟测试入口。
 *
 * 该测试只验证 OpenCV core/imgproc 头文件、库链接和基础矩阵操作可用，
 * 不读取真实图像，不实现焊缝检测算法，也不访问旧工程。
 */
int main()
{
    cv::Mat bgrImage(4, 4, CV_8UC3, cv::Scalar(10, 20, 30));
    cv::Mat grayImage;
    cv::cvtColor(bgrImage, grayImage, cv::COLOR_BGR2GRAY);

    const cv::Scalar meanValue = cv::mean(grayImage);
    if (grayImage.empty() || grayImage.rows != 4 || grayImage.cols != 4) {
        std::cerr << "FAIL: cv::Mat or cv::cvtColor smoke check failed\n";
        return 1;
    }

    std::cout << "PASS: OpenCV smoke test\n";
    std::cout << "OpenCV version: " << CV_VERSION << '\n';
    std::cout << "Mean gray value: " << meanValue[0] << '\n';
    return 0;
}
