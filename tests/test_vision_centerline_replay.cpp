/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_centerline_replay.cpp
 * @brief       vision centerline replay 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "stripe/DpStripeCenterlineDetector.h"

namespace {

namespace fs = std::filesystem;

constexpr std::size_t kMaxReplayImageCount = 120U;

struct ReplayRow {
    int index = 0;
    fs::path inputFile;
    int imageWidth = 0;
    int imageHeight = 0;
    bool valid = false;
    std::size_t pointCount = 0U;
    double validColumnRatio = 0.0;
    double brokenStripeRatio = 0.0;
    double meanConfidence = 0.0;
    double processingTimeMs = 0.0;
    std::string message;
};

struct ReplaySummary {
    int totalImages = 0;
    int validImages = 0;
    int invalidImages = 0;
    double averagePointCount = 0.0;
    double averageValidColumnRatio = 0.0;
    double averageBrokenStripeRatio = 0.0;
    double averageMeanConfidence = 0.0;
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

bool isReplayImageFile(const fs::path &path)
{
    const std::string extension = toLower(path.extension().string());
    return extension == ".png"
        || extension == ".jpg"
        || extension == ".jpeg"
        || extension == ".bmp";
}

fs::path findProjectRoot(void)
{
#ifdef I2R_RLC_PROJECT_SOURCE_DIR
    const fs::path sourceDir = I2R_RLC_PROJECT_SOURCE_DIR;
    if (fs::exists(sourceDir / "CMakeLists.txt")
        && fs::exists(sourceDir / "Vision")
        && fs::exists(sourceDir / "tests"))
    {
        return sourceDir;
    }
#endif

    fs::path current = fs::current_path();
    while (!current.empty())
    {
        if (fs::exists(current / "CMakeLists.txt")
            && fs::exists(current / "Vision")
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

std::vector<fs::path> collectReplayImages(const fs::path &replayDir)
{
    std::vector<fs::path> imageFiles;
    if (!fs::exists(replayDir) || !fs::is_directory(replayDir))
    {
        return imageFiles;
    }

    for (const fs::directory_entry &entry :
         fs::recursive_directory_iterator(replayDir))
    {
        if (entry.is_regular_file() && isReplayImageFile(entry.path()))
        {
            imageFiles.push_back(entry.path());
        }
    }

    std::sort(imageFiles.begin(), imageFiles.end());
    if (imageFiles.size() > kMaxReplayImageCount)
    {
        imageFiles.resize(kMaxReplayImageCount);
    }
    return imageFiles;
}

i2r_rlc::vision::DpStripeDetectionOptions makeReplayOptions(void)
{
    i2r_rlc::vision::DpStripeDetectionOptions options;
    options.blurSigma = 1.2;
    options.adaptiveThresholdRatio = 0.45;
    options.minProminenceGray = 6.0;
    options.prominenceAbsMinGray = 1.5;
    options.maxCandidatesPerColumn = 6;
    options.maxJumpPixel = 20.0;
    options.minValidPointCount = 12;
    return options;
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

std::string escapeCsv(const std::string &text)
{
    if (text.find_first_of(",\"\n\r") == std::string::npos)
    {
        return text;
    }

    std::string escaped = "\"";
    for (char character : text)
    {
        if (character == '"')
        {
            escaped += "\"\"";
        }
        else
        {
            escaped += character;
        }
    }
    escaped += '"';
    return escaped;
}

bool writeReport(const fs::path &reportPath, const std::vector<ReplayRow> &rows)
{
    fs::create_directories(reportPath.parent_path());

    std::ofstream report(
        reportPath.string(),
        std::ios::out | std::ios::trunc);
    if (!report.is_open())
    {
        return false;
    }

    report << "index,inputFile,imageWidth,imageHeight,valid,pointCount,"
           << "validColumnRatio,brokenStripeRatio,meanConfidence,"
           << "processingTimeMs,message\n";

    report << std::fixed << std::setprecision(6);
    for (const ReplayRow &row : rows)
    {
        report << row.index << ','
               << escapeCsv(row.inputFile.string()) << ','
               << row.imageWidth << ','
               << row.imageHeight << ','
               << (row.valid ? 1 : 0) << ','
               << row.pointCount << ','
               << row.validColumnRatio << ','
               << row.brokenStripeRatio << ','
               << row.meanConfidence << ','
               << row.processingTimeMs << ','
               << escapeCsv(row.message) << '\n';
    }

    return true;
}

ReplaySummary calculateSummary(const std::vector<ReplayRow> &rows)
{
    ReplaySummary summary;
    summary.totalImages = static_cast<int>(rows.size());
    for (const ReplayRow &row : rows)
    {
        if (row.valid)
        {
            ++summary.validImages;
        }
        else
        {
            ++summary.invalidImages;
        }

        summary.averagePointCount += static_cast<double>(row.pointCount);
        summary.averageValidColumnRatio += row.validColumnRatio;
        summary.averageBrokenStripeRatio += row.brokenStripeRatio;
        summary.averageMeanConfidence += row.meanConfidence;
    }

    if (!rows.empty())
    {
        const double count = static_cast<double>(rows.size());
        summary.averagePointCount /= count;
        summary.averageValidColumnRatio /= count;
        summary.averageBrokenStripeRatio /= count;
        summary.averageMeanConfidence /= count;
    }

    return summary;
}

void printSummary(const ReplaySummary &summary, const fs::path &reportPath)
{
    std::cout << "Total images: " << summary.totalImages << '\n';
    std::cout << "Valid: " << summary.validImages << '\n';
    std::cout << "Invalid: " << summary.invalidImages << '\n';
    std::cout << "Average point count: " << summary.averagePointCount << '\n';
    std::cout << "Average validColumnRatio: "
              << summary.averageValidColumnRatio << '\n';
    std::cout << "Average brokenStripeRatio: "
              << summary.averageBrokenStripeRatio << '\n';
    std::cout << "Average meanConfidence: "
              << summary.averageMeanConfidence << '\n';
    std::cout << "Report path: " << reportPath.string() << '\n';
    std::cout << "Safety boundary: no Qt in vision / no HALCON / no PCL / "
              << "no real camera / no DX200 / no TCP\n";
}

} // namespace

int main()
{
    using i2r_rlc::vision::CDpStripeCenterlineDetector;

    const fs::path projectRoot = findProjectRoot();
    const fs::path replayDir = projectRoot / "data" / "replay";
    const fs::path reportPath =
        projectRoot / "output" / "reports" / "centerline_replay_report.csv";

    std::vector<ReplayRow> rows;
    const std::vector<fs::path> imageFiles = collectReplayImages(replayDir);
    if (imageFiles.empty())
    {
        std::cout << "SKIP: no replay images found at "
                  << replayDir.string() << '\n';
    }

    const auto options = makeReplayOptions();
    int index = 0;
    for (const fs::path &imageFile : imageFiles)
    {
        ReplayRow row;
        row.index = index++;
        row.inputFile = imageFile;

        const cv::Mat gray8 = readGray8(imageFile);
        if (gray8.empty())
        {
            row.message = "image read failed";
            rows.push_back(row);
            continue;
        }

        row.imageWidth = gray8.cols;
        row.imageHeight = gray8.rows;

        const auto result = CDpStripeCenterlineDetector::detectCenterline(
            gray8,
            options);
        row.valid = result.valid;
        row.pointCount = result.points.size();
        row.validColumnRatio = result.validColumnRatio;
        row.brokenStripeRatio = result.brokenStripeRatio;
        row.meanConfidence = result.meanConfidence;
        row.processingTimeMs = result.processingTimeMs;
        row.message = result.message;
        rows.push_back(row);
    }

    if (!writeReport(reportPath, rows))
    {
        std::cerr << "Failed to write report: " << reportPath.string() << '\n';
        return 1;
    }

    const ReplaySummary summary = calculateSummary(rows);
    printSummary(summary, reportPath);

    std::printf("PASS: vision centerline replay tests\n");
    return 0;
}
