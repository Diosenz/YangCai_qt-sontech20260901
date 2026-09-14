/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_image_pro_feng_replay.cpp
 * @brief       vision image pro feng replay 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "feature/SimulatedLaserWeldDetector.h"

namespace {

namespace fs = std::filesystem;

struct FrameData {
    int frameIndex = -1;
    fs::path imagePath;
};

std::string toLower(std::string text)
{
    for (char &character : text)
    {
        character = static_cast<char>(
            std::tolower(static_cast<unsigned char>(character)));
    }
    return text;
}

int parseImageFrameIndex(const fs::path &path)
{
    const std::string stem = path.stem().string();
    const std::string prefix = "image_";
    if (stem.rfind(prefix, 0U) != 0U)
    {
        return -1;
    }

    const std::string numberText = stem.substr(prefix.size());
    if (numberText.empty())
    {
        return -1;
    }

    for (char character : numberText)
    {
        if (!std::isdigit(static_cast<unsigned char>(character)))
        {
            return -1;
        }
    }
    return std::stoi(numberText);
}

fs::path findProjectRoot(void)
{
#ifdef I2R_RLC_PROJECT_SOURCE_DIR
    const fs::path sourceDir = I2R_RLC_PROJECT_SOURCE_DIR;
    if (fs::exists(sourceDir / "CMakeLists.txt")
        && fs::exists(sourceDir / "src")
        && fs::exists(sourceDir / "tests"))
    {
        return sourceDir;
    }
#endif

    fs::path current = fs::current_path();
    while (!current.empty())
    {
        if (fs::exists(current / "CMakeLists.txt")
            && fs::exists(current / "src")
            && fs::exists(current / "tests"))
        {
            return current;
        }

        const fs::path parent = current.parent_path();
        if (parent == current)
        {
            break;
        }
        current = parent;
    }
    return fs::current_path();
}

std::vector<FrameData> collectImages(const fs::path &imageDir)
{
    std::vector<FrameData> frames;
    if (!fs::exists(imageDir) || !fs::is_directory(imageDir))
    {
        return frames;
    }

    for (const fs::directory_entry &entry : fs::directory_iterator(imageDir))
    {
        if (!entry.is_regular_file()
            || toLower(entry.path().extension().string()) != ".png")
        {
            continue;
        }

        const int frameIndex = parseImageFrameIndex(entry.path());
        if (frameIndex < 0)
        {
            continue;
        }

        FrameData frame;
        frame.frameIndex = frameIndex;
        frame.imagePath = entry.path();
        frames.push_back(frame);
    }

    std::sort(
        frames.begin(),
        frames.end(),
        [](const FrameData &left, const FrameData &right) {
            return left.frameIndex < right.frameIndex;
        });
    return frames;
}

std::string readTextFile(const fs::path &path)
{
    std::ifstream input(path);
    if (!input.is_open())
    {
        return std::string();
    }

    std::ostringstream output;
    output << input.rdbuf();
    return output.str();
}

cv::Mat readGray8(const fs::path &imagePath)
{
    const cv::Mat image = cv::imread(imagePath.string(), cv::IMREAD_UNCHANGED);
    if (image.empty())
    {
        return cv::Mat();
    }

    if (image.type() == CV_8UC1)
    {
        return image;
    }

    cv::Mat gray8;
    if (image.channels() == 3)
    {
        cv::cvtColor(image, gray8, cv::COLOR_BGR2GRAY);
    }
    else if (image.channels() == 4)
    {
        cv::cvtColor(image, gray8, cv::COLOR_BGRA2GRAY);
    }
    else
    {
        image.convertTo(gray8, CV_8UC1);
    }
    return gray8;
}

double errorToObservationWeldPoint(
    const i2r_rlc::vision::SimulatedLaserMatchingResult &result,
    const i2r_rlc::digital_twin::DigitalTwinVisionLaserObservation &observation)
{
    if (!result.valid || !observation.hasWeldPointImage_px)
    {
        return 0.0;
    }

    const double dx = result.weldPoint.x - observation.weldPointImage_px.x_px;
    const double dy = result.weldPoint.y - observation.weldPointImage_px.y_px;
    return std::sqrt(dx * dx + dy * dy);
}

template <typename Callable>
i2r_rlc::vision::SimulatedLaserMatchingResult runTimed(
    Callable callable,
    double *elapsedMs)
{
    const auto start = std::chrono::steady_clock::now();
    i2r_rlc::vision::SimulatedLaserMatchingResult result = callable();
    const auto end = std::chrono::steady_clock::now();
    *elapsedMs = std::chrono::duration<double, std::milli>(end - start).count();
    return result;
}

void writeReportRow(
    std::ofstream &report,
    int frameIndex,
    const char *method,
    const i2r_rlc::vision::SimulatedLaserMatchingResult &result,
    const i2r_rlc::digital_twin::DigitalTwinVisionLaserObservation &observation,
    double elapsedMs)
{
    report << frameIndex << ','
        << method << ','
        << (result.valid ? 1 : 0) << ','
        << result.weldPoint.x << ','
        << result.weldPoint.y << ','
        << result.matchedLaserPoints.size() << ','
        << result.skeletonPoints.size() << ','
        << result.meanResidualPixel << ','
        << result.confidence << ','
        << (result.coarseValid ? 1 : 0) << ','
        << (result.usedHoughFallback ? 1 : 0) << ','
        << (result.usedTemplateFallback ? 1 : 0) << ','
        << (result.usedChamferFallback ? 1 : 0) << ','
        << result.templateMethod << ','
        << result.templateScore << ','
        << result.templateAngleDegree << ','
        << result.templateModelBuildMs << ','
        << result.templateAddMs << ','
        << result.templateMatchMs << ','
        << result.templateTotalMs << ','
        << elapsedMs << ','
        << errorToObservationWeldPoint(result, observation) << ','
        << '"' << result.message << '"' << '\n';
}

} // namespace

int main()
{
    using i2r_rlc::vision::SimulatedLaserMatchingOptions;
    using i2r_rlc::vision::CSimulatedLaserRegistrationMatcher;
    using i2r_rlc::vision::CSimulatedLaserReplayParser;
    using i2r_rlc::vision::CSimulatedLaserTemplateMatcher;

    const fs::path projectRoot = findProjectRoot();
    const fs::path replayRoot = projectRoot / "data" / "replay" / "penzui";
    const fs::path imageDir = replayRoot / "imagecatch";
    const fs::path laserPath = replayRoot / "simulatedLaser.txt";
    const fs::path weldPath = replayRoot / "simulatedWeldpt.txt";
    if (!fs::exists(imageDir) || !fs::exists(laserPath) || !fs::exists(weldPath))
    {
        std::printf("SKIP: penzui replay data not found\n");
        return 0;
    }

    const auto parseResult = CSimulatedLaserReplayParser::parse(
        readTextFile(laserPath),
        readTextFile(weldPath));
    const std::vector<FrameData> frames = collectImages(imageDir);
    if (!parseResult.valid || frames.empty())
    {
        std::printf("SKIP: penzui replay data is empty\n");
        return 0;
    }

    const fs::path reportDir = projectRoot / "output" / "reports";
    fs::create_directories(reportDir);
    fs::path reportPath = reportDir / "image_pro_feng_replay_report.csv";
    std::ofstream report(
        reportPath.string(),
        std::ios::out | std::ios::trunc);
    if (!report.is_open())
    {
        reportPath = reportDir / (
            std::string("image_pro_feng_replay_report_")
            + std::to_string(std::chrono::steady_clock::now()
                .time_since_epoch().count())
            + ".csv");
        report.open(reportPath.string(), std::ios::out | std::ios::trunc);
    }
    if (!report.is_open())
    {
        reportPath = projectRoot / (
            std::string("image_pro_feng_replay_report_")
            + std::to_string(std::chrono::steady_clock::now()
                .time_since_epoch().count())
            + ".csv");
        report.open(reportPath.string(), std::ios::out | std::ios::trunc);
    }
    if (!report.is_open())
    {
        std::printf(
            "SKIP: failed to create image pro feng replay report: %s, errno=%d, %s\n",
            reportPath.string().c_str(),
            errno,
            std::strerror(errno));
        return 0;
    }
    report << "frame,method,valid,weld_x,weld_y,matched_points,"
        "skeleton_points,mean_residual,confidence,coarse_valid,hough_fallback,"
        "template_fallback,chamfer_fallback,template_method,template_score,"
        "template_angle,template_model_build_ms,template_add_ms,"
        "template_match_ms,template_total_ms,elapsed_ms,error_to_sim_weld_px,"
        "message\n";

    SimulatedLaserMatchingOptions options;
    options.skeletonThresholdValue = 35;
    options.minConfidence = 0.0;
    int processedFrames = 0;
    int validRegistrationFrames = 0;
    int validTemplateFrames = 0;
    const int maxFrames = 20;
    for (const FrameData &frameData : frames)
    {
        if (frameData.frameIndex
            >= static_cast<int>(parseResult.observations.size()))
        {
            continue;
        }

        const cv::Mat gray8 = readGray8(frameData.imagePath);
        if (gray8.empty())
        {
            continue;
        }

        const auto &observation = parseResult.observations[
            static_cast<std::size_t>(frameData.frameIndex)];

        // 1. 对应旧 EmulateImagePt：仿真激光线配准独立运行。
        double registrationElapsedMs = 0.0;
        const auto registrationResult = runTimed(
            [&gray8, &observation, &options]() {
                return CSimulatedLaserRegistrationMatcher::match(
                    gray8,
                    observation,
                    options);
            },
            &registrationElapsedMs);
        if (registrationResult.valid)
        {
            ++validRegistrationFrames;
        }
        writeReportRow(
            report,
            frameData.frameIndex,
            "registration",
            registrationResult,
            observation,
            registrationElapsedMs);

        // 2. 对应旧 TemplateMatching_change：shape_based_matching 主路径
        //    独立运行，失败时在该入口内部退回 Chamfer。
        double templateElapsedMs = 0.0;
        const auto templateResult = runTimed(
            [&gray8, &observation, &options]() {
                return CSimulatedLaserTemplateMatcher::match(
                    gray8,
                    observation,
                    options);
            },
            &templateElapsedMs);
        if (templateResult.valid)
        {
            ++validTemplateFrames;
        }
        writeReportRow(
            report,
            frameData.frameIndex,
            "template_matching",
            templateResult,
            observation,
            templateElapsedMs);

        ++processedFrames;
        if (processedFrames >= maxFrames)
        {
            break;
        }
    }

    if (processedFrames == 0)
    {
        return 1;
    }

    std::printf(
        "PASS: vision image pro feng replay tests, frames=%d registration=%d template=%d\n",
        processedFrames,
        validRegistrationFrames,
        validTemplateFrames);
    return 0;
}


