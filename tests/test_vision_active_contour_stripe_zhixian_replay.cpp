/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_active_contour_stripe_zhixian_replay.cpp
 * @brief       vision active contour stripe zhixian replay 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <regex>

#include "stripe/ActiveContourStripeExtractor.h"

namespace {

namespace fs = std::filesystem;

struct ReplayFrame {
    int frameIndex = -1;
    fs::path imagePath;
};

std::vector<ReplayFrame> collectFrames(const fs::path &imageDir)
{
    std::vector<ReplayFrame> frames;
    const std::regex imagePattern(R"(image_(\d+)\.(png|jpg|jpeg|bmp))",
                                  std::regex::icase);
    if (!fs::exists(imageDir))
    {
        return frames;
    }

    for (const fs::directory_entry &entry : fs::directory_iterator(imageDir))
    {
        if (!entry.is_regular_file())
        {
            continue;
        }

        const std::string fileName = entry.path().filename().string();
        std::smatch match;
        if (!std::regex_match(fileName, match, imagePattern))
        {
            continue;
        }

        ReplayFrame frame;
        frame.frameIndex = std::stoi(match[1].str());
        frame.imagePath = entry.path();
        frames.push_back(frame);
    }

    std::sort(
        frames.begin(),
        frames.end(),
        [](const ReplayFrame &left, const ReplayFrame &right) {
            return left.frameIndex < right.frameIndex;
        });
    return frames;
}

cv::Mat readGray8(const fs::path &path)
{
    return cv::imread(path.string(), cv::IMREAD_GRAYSCALE);
}

void drawOverlay(
    const cv::Mat &gray8,
    const i2r_rlc::vision::ActiveContourStripeResult &result,
    const fs::path &outputPath)
{
    cv::Mat overlay;
    cv::cvtColor(gray8, overlay, cv::COLOR_GRAY2BGR);

    for (std::size_t index = 1U; index < result.activeCurve.curvePoints.size();
         ++index)
    {
        cv::line(
            overlay,
            result.activeCurve.curvePoints[index - 1U],
            result.activeCurve.curvePoints[index],
            cv::Scalar(0, 255, 0),
            1,
            cv::LINE_AA);
    }

    for (const cv::Point2d &point : result.activeCurve.controlPoints)
    {
        cv::circle(overlay, point, 2, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
    }

    if (!result.valid)
    {
        cv::putText(
            overlay,
            "invalid",
            cv::Point(12, 24),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(0, 0, 255),
            2,
            cv::LINE_AA);
    }

    cv::imwrite(outputPath.string(), overlay);
}

i2r_rlc::vision::ActiveContourStripeOptions makeOptions()
{
    i2r_rlc::vision::ActiveContourStripeOptions options;
    options.laserWidthThresholdGrayValue = 180;
    options.initialThresholdGrayValue = 100;
    options.initialClosingRadiusPixel = 3;
    // replay 测试关注全帧稳定可视化，使用较轻的采样与迭代参数；
    // 生产默认值仍由 ActiveContourStripeOptions 自身提供。
    options.initialControlPointStepPixel = 12;
    options.interpolationSamplesPerSegment = 3;
    options.snakeIterationCount = 16;
    options.snakeMinGrayValue = 150;
    options.controlPointDragSearchHalfHeightPixel = 20;
    options.controlPointDragMinGrayValue = 90;
    options.controlPointGrowMinGrayValue = 150;
    return options;
}

} // namespace

/**
 * @brief zhixian replay 主动轮廓条纹提取测试入口。
 *
 * 本测试只观察 Snake 提取到的激光条纹曲线，不做焊缝类型检测，也不绘制
 * 仿真激光线或仿真焊缝点。
 */
int main(int argc, char **argv)
{
#ifndef I2R_RLC_PROJECT_SOURCE_DIR
    std::cerr << "I2R_RLC_PROJECT_SOURCE_DIR is not defined\n";
    return 1;
#else
    int maxFrames = -1;
    if (argc > 1)
    {
        maxFrames = std::atoi(argv[1]);
    }

    const fs::path projectRoot = I2R_RLC_PROJECT_SOURCE_DIR;
    const fs::path reportDir =
        fs::current_path()
        / "Testing"
        / "Temporary"
        / "active_contour_stripe_zhixian";
    fs::create_directories(reportDir);
    const fs::path reportPath =
        reportDir / "active_contour_stripe_zhixian_report.csv";
    std::ofstream report(reportPath);
    if (!report.is_open())
    {
        std::cerr << "failed to open report: " << reportPath.string() << '\n';
        return 3;
    }
    report << "frame,valid,control_points,curve_points,quality,elapsed_ms,message\n";
    report.flush();

    const fs::path imageDir =
        projectRoot / "data" / "replay" / "zhixian" / "imagecatch";
    const std::vector<ReplayFrame> frames = collectFrames(imageDir);
    if (frames.empty())
    {
        std::cout << "SKIP: zhixian replay images not found: "
                  << imageDir.string() << '\n';
        return 0;
    }

    const fs::path overlayDir =
        reportDir / "active_contour_stripe_zhixian_overlays";
    fs::create_directories(overlayDir);
    std::cout << "found " << frames.size() << " zhixian frames";
    if (maxFrames > 0)
    {
        std::cout << ", maxFrames=" << maxFrames;
    }
    std::cout << '\n';

    i2r_rlc::vision::ActiveContourStripeOptions options = makeOptions();
    i2r_rlc::vision::ActiveContourStripeState state;
    int processedCount = 0;
    for (const ReplayFrame &frame : frames)
    {
        if (maxFrames > 0 && processedCount >= maxFrames)
        {
            break;
        }
        std::cout << "processing zhixian frame " << frame.frameIndex << '\n';
        const cv::Mat gray8 = readGray8(frame.imagePath);
        if (gray8.empty())
        {
            continue;
        }

        const auto result =
            i2r_rlc::vision::CActiveContourStripeExtractor::extract(
                gray8,
                options,
                &state);

        report << frame.frameIndex << ','
               << (result.valid ? 1 : 0) << ','
               << result.activeCurve.controlPoints.size() << ','
               << result.activeCurve.curvePoints.size() << ','
               << result.qualityScore << ','
               << result.processingTimeMs << ','
               << '"' << result.message << '"' << '\n';
        report.flush();

        drawOverlay(
            gray8,
            result,
            overlayDir / ("stripe_" + std::to_string(frame.frameIndex) + ".png"));
        ++processedCount;
        if (processedCount % 10 == 0)
        {
            std::cout << "processed " << processedCount << '/'
                      << frames.size() << " zhixian frames\n";
        }
    }

    if (processedCount == 0)
    {
        std::cerr << "no zhixian frames processed\n";
        return 4;
    }

    std::cout << "PASS: active contour stripe zhixian replay processed "
              << processedCount << " frames\n";
    return 0;
#endif
}
