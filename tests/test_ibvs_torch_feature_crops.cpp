/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      test_ibvs_torch_feature_crops.cpp
 * @brief     Offline torch red-feature extraction test for cropped images.
 * @details   Reads repo-local `data/IBVS/torch_feature/crops/set_001`
 *            images and writes visual artifacts for human review.
 */

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "ibvs/IbvsRedFeatureDetector.h"

#ifndef I2R_RLC_PROJECT_SOURCE_DIR
#define I2R_RLC_PROJECT_SOURCE_DIR "."
#endif

namespace {

using namespace i2r_rlc::ibvs;

struct TorchFeatureMetric
{
    std::string imageName;
    int imageWidth = 0;
    int imageHeight = 0;
    bool accepted = false;
    std::size_t candidateCount = 0;
    PixelFeaturePair featurePoints;
    double distancePx = 0.0;
    std::string status;
};

std::string makeTimestamp()
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t timeValue = std::chrono::system_clock::to_time_t(now);
    std::tm localTime;
    localtime_s(&localTime, &timeValue);

    std::ostringstream stream;
    stream << std::put_time(&localTime, "%Y%m%d_%H%M%S");
    return stream.str();
}

std::filesystem::path makeArtifactDir()
{
    const std::filesystem::path root =
        std::filesystem::path(I2R_RLC_PROJECT_SOURCE_DIR)
        / "output"
        / "ibvs_offline_validation"
        / "i2r_rlc_ibvs_torch_feature_crops_tests"
        / makeTimestamp();
    std::filesystem::create_directories(root);
    return root;
}

bool hasImageExtension(const std::filesystem::path& path)
{
    std::string extension = path.extension().string();
    for (char& ch : extension)
    {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return extension == ".jpg"
        || extension == ".jpeg"
        || extension == ".png"
        || extension == ".bmp";
}

std::vector<std::filesystem::path> listCropImages(
    const std::filesystem::path& cropDir)
{
    std::vector<std::filesystem::path> paths;
    if (!std::filesystem::exists(cropDir))
    {
        return paths;
    }

    for (const auto& entry : std::filesystem::directory_iterator(cropDir))
    {
        if (entry.is_regular_file() && hasImageExtension(entry.path()))
        {
            paths.push_back(entry.path());
        }
    }

    std::sort(paths.begin(), paths.end());
    return paths;
}

cv::Mat makeRedMaskForArtifact(const cv::Mat& bgrImage)
{
    cv::Mat hsv;
    cv::cvtColor(bgrImage, hsv, cv::COLOR_BGR2HSV);

    cv::Mat lowRedMask;
    cv::Mat highRedMask;
    cv::Mat redMask;
    cv::inRange(hsv, cv::Scalar(0, 30, 10), cv::Scalar(15, 255, 255), lowRedMask);
    cv::inRange(
        hsv,
        cv::Scalar(160, 30, 10),
        cv::Scalar(180, 255, 255),
        highRedMask);
    cv::bitwise_or(lowRedMask, highRedMask, redMask);

    const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(3, 3));
    cv::morphologyEx(redMask, redMask, cv::MORPH_OPEN, kernel);
    return redMask;
}

void drawPoint(
    cv::Mat& image,
    const PixelPoint& point,
    const cv::Scalar& color,
    int radius)
{
    cv::circle(
        image,
        cv::Point(
            static_cast<int>(std::lround(point.u)),
            static_cast<int>(std::lround(point.v))),
        radius,
        color,
        cv::FILLED,
        cv::LINE_AA);
}

void writeAnnotatedImage(
    const cv::Mat& image,
    const TorchFeatureMetric& metric,
    const std::filesystem::path& path)
{
    cv::Mat visual = image.clone();

    const cv::Scalar statusColor =
        metric.accepted ? cv::Scalar(0, 180, 0) : cv::Scalar(0, 0, 255);
    cv::rectangle(
        visual,
        cv::Rect(0, 0, visual.cols, visual.rows),
        statusColor,
        2);

    if (metric.accepted)
    {
        drawPoint(visual, metric.featurePoints.first, cv::Scalar(0, 0, 255), 6);
        drawPoint(visual, metric.featurePoints.second, cv::Scalar(0, 0, 255), 6);
        cv::line(
            visual,
            cv::Point(
                static_cast<int>(std::lround(metric.featurePoints.first.u)),
                static_cast<int>(std::lround(metric.featurePoints.first.v))),
            cv::Point(
                static_cast<int>(std::lround(metric.featurePoints.second.u)),
                static_cast<int>(std::lround(metric.featurePoints.second.v))),
            cv::Scalar(0, 255, 255),
            2,
            cv::LINE_AA);
    }

    std::ostringstream label;
    label << "accepted=" << (metric.accepted ? 1 : 0)
          << ", candidates=" << metric.candidateCount
          << ", distance=" << std::fixed << std::setprecision(1)
          << metric.distancePx;
    cv::putText(
        visual,
        label.str(),
        cv::Point(12, 28),
        cv::FONT_HERSHEY_SIMPLEX,
        0.65,
        statusColor,
        2,
        cv::LINE_AA);

    cv::imwrite(path.string(), visual);
}

bool writeMetricsCsv(
    const std::filesystem::path& path,
    const std::vector<TorchFeatureMetric>& metrics)
{
    std::ofstream file(path);
    if (!file.is_open())
    {
        std::cerr << "FAIL: cannot write metrics csv: " << path.string() << '\n';
        return false;
    }

    file << "image,width,height,accepted,candidates,u1,v1,u2,v2,distance_px,status\n";
    for (const TorchFeatureMetric& metric : metrics)
    {
        file << metric.imageName << ','
             << metric.imageWidth << ','
             << metric.imageHeight << ','
             << (metric.accepted ? 1 : 0) << ','
             << metric.candidateCount << ','
             << metric.featurePoints.first.u << ','
             << metric.featurePoints.first.v << ','
             << metric.featurePoints.second.u << ','
             << metric.featurePoints.second.v << ','
             << metric.distancePx << ','
             << metric.status << '\n';
    }
    return true;
}

bool writeSummary(
    const std::filesystem::path& path,
    const std::vector<TorchFeatureMetric>& metrics,
    int acceptedCount,
    double averageDistancePx,
    double minDistancePx,
    double maxDistancePx)
{
    std::ofstream file(path);
    if (!file.is_open())
    {
        std::cerr << "FAIL: cannot write summary: " << path.string() << '\n';
        return false;
    }

    file << "images=" << metrics.size() << '\n'
         << "accepted=" << acceptedCount << '\n'
         << "rejected=" << (static_cast<int>(metrics.size()) - acceptedCount) << '\n'
         << "acceptanceRatio="
         << (metrics.empty()
             ? 0.0
             : static_cast<double>(acceptedCount)
                   / static_cast<double>(metrics.size())) << '\n'
         << "averageDistancePx=" << averageDistancePx << '\n'
         << "minDistancePx=" << minDistancePx << '\n'
         << "maxDistancePx=" << maxDistancePx << '\n';
    return true;
}

} // namespace

/**
 * @brief Runs red-feature extraction on repo-local torch crop images.
 */
int main()
{
    const std::filesystem::path cropDir =
        std::filesystem::path(I2R_RLC_PROJECT_SOURCE_DIR)
        / "data"
        / "IBVS"
        / "torch_feature"
        / "crops"
        / "set_001";

    const std::vector<std::filesystem::path> imagePaths = listCropImages(cropDir);
    if (imagePaths.empty())
    {
        std::cout << "SKIP: no torch feature crop images found: "
                  << cropDir.string() << '\n';
        return 0;
    }

    const std::filesystem::path artifactDir = makeArtifactDir();
    std::filesystem::create_directories(artifactDir / "annotated");
    std::filesystem::create_directories(artifactDir / "red_mask");

    IbvsRedFeatureDetectorConfig config;
    config.applyGamma = true;
    config.applyBilateral = true;
    config.morphKernelSize = 3;
    config.minPairDistancePx = 15.0;
    config.maxPairDistancePx = 80.0;

    const IbvsRedFeatureDetector detector(config);

    std::vector<TorchFeatureMetric> metrics;
    metrics.reserve(imagePaths.size());
    int acceptedCount = 0;
    double distanceSumPx = 0.0;
    double minDistancePx = std::numeric_limits<double>::infinity();
    double maxDistancePx = 0.0;

    try
    {
        for (std::size_t i = 0; i < imagePaths.size(); ++i)
        {
            const std::filesystem::path& imagePath = imagePaths[i];
            const cv::Mat image = cv::imread(imagePath.string(), cv::IMREAD_COLOR);
            if (image.empty())
            {
                std::cerr << "FAIL: cannot read crop image: "
                          << imagePath.string() << '\n';
                return 1;
            }

            const auto result = detector.detect(image);

            TorchFeatureMetric metric;
            metric.imageName = imagePath.filename().string();
            metric.imageWidth = image.cols;
            metric.imageHeight = image.rows;
            metric.accepted = result.accepted && result.error.isOk();
            metric.candidateCount = result.candidates.size();
            metric.featurePoints = result.featurePoints;
            metric.distancePx = result.distancePx;
            metric.status = result.error.isOk() ? "ok" : result.error.message;

            if (metric.accepted)
            {
                ++acceptedCount;
                distanceSumPx += metric.distancePx;
                minDistancePx = std::min(minDistancePx, metric.distancePx);
                maxDistancePx = std::max(maxDistancePx, metric.distancePx);
            }

            const std::string indexText = std::to_string(i);
            cv::imwrite(
                (artifactDir / "red_mask" / ("red_mask_" + indexText + ".png")).string(),
                makeRedMaskForArtifact(image));
            writeAnnotatedImage(
                image,
                metric,
                artifactDir / "annotated" / ("annotated_" + indexText + ".jpg"));

            metrics.push_back(metric);
        }
    }
    catch (const std::exception& exception)
    {
        std::cerr << "FAIL: torch feature crop test exception: "
                  << exception.what() << '\n';
        return 1;
    }

    const double averageDistancePx =
        acceptedCount > 0 ? distanceSumPx / static_cast<double>(acceptedCount) : 0.0;
    if (acceptedCount == 0)
    {
        minDistancePx = 0.0;
    }

    if (!writeMetricsCsv(artifactDir / "metrics.csv", metrics)
        || !writeSummary(
            artifactDir / "summary.txt",
            metrics,
            acceptedCount,
            averageDistancePx,
            minDistancePx,
            maxDistancePx))
    {
        return 1;
    }

    std::cout << "INFO: torch feature artifacts=" << artifactDir.string() << '\n'
              << "INFO: torch feature images=" << metrics.size()
              << ", accepted=" << acceptedCount
              << ", acceptanceRatio="
              << static_cast<double>(acceptedCount)
                     / static_cast<double>(metrics.size())
              << ", averageDistancePx=" << averageDistancePx << '\n';

    if (acceptedCount == 0)
    {
        std::cerr << "FAIL: no accepted torch red feature pair in crop images\n";
        return 1;
    }

    std::cout << "PASS: IBVS torch feature crop tests\n";
    return 0;
}
