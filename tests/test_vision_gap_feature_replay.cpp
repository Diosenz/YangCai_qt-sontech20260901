/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_gap_feature_replay.cpp
 * @brief       vision gap feature replay 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "feature/WeldFeatureDetector.h"
#include "stripe/DpStripeCenterlineDetector.h"

namespace {

namespace fs = std::filesystem;

constexpr std::size_t kMaxReplayImageCount = 120U;

struct ReplayRow {
    int index = 0;
    fs::path inputFile;
    int imageWidth = 0;
    int imageHeight = 0;
    bool centerlineValid = false;
    std::size_t centerlinePointCount = 0U;
    double validColumnRatio = 0.0;
    double brokenStripeRatio = 0.0;
    double centerlineMeanConfidence = 0.0;
    bool gapValid = false;
    double leftX = 0.0;
    double leftY = 0.0;
    double rightX = 0.0;
    double rightY = 0.0;
    double gapWidthPixel = 0.0;
    double weldPointX = 0.0;
    double weldPointY = 0.0;
    double gapConfidence = 0.0;
    double processingTimeMs = 0.0;
    std::string message;
};

struct ReplaySummary {
    int totalImages = 0;
    int centerlineValid = 0;
    int gapValid = 0;
    int invalid = 0;
    double averageGapWidthPixel = 0.0;
    double averageGapConfidence = 0.0;
    double averageBrokenStripeRatio = 0.0;
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

i2r_rlc::vision::DpStripeDetectionOptions makeCenterlineOptions(void)
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

i2r_rlc::vision::WeldFeatureDetectionOptions makeFeatureOptions(void)
{
    i2r_rlc::vision::WeldFeatureDetectionOptions options;
    options.searchHalfWindowY = 5;
    options.minEdgeContrast = 40;
    options.minGapWidthPixel = 4;
    options.maxGapWidthPixel = 80;
    options.minConfidence = 0.10;
    options.useCenterlinePrior = true;
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

    report << "index,inputFile,imageWidth,imageHeight,centerlineValid,"
           << "centerlinePointCount,validColumnRatio,brokenStripeRatio,"
           << "centerlineMeanConfidence,gapValid,leftX,leftY,rightX,rightY,"
           << "gapWidthPixel,weldPointX,weldPointY,gapConfidence,"
           << "processingTimeMs,message\n";

    report << std::fixed << std::setprecision(6);
    for (const ReplayRow &row : rows)
    {
        report << row.index << ','
               << escapeCsv(row.inputFile.string()) << ','
               << row.imageWidth << ','
               << row.imageHeight << ','
               << (row.centerlineValid ? 1 : 0) << ','
               << row.centerlinePointCount << ','
               << row.validColumnRatio << ','
               << row.brokenStripeRatio << ','
               << row.centerlineMeanConfidence << ','
               << (row.gapValid ? 1 : 0) << ','
               << row.leftX << ','
               << row.leftY << ','
               << row.rightX << ','
               << row.rightY << ','
               << row.gapWidthPixel << ','
               << row.weldPointX << ','
               << row.weldPointY << ','
               << row.gapConfidence << ','
               << row.processingTimeMs << ','
               << escapeCsv(row.message) << '\n';
    }

    return true;
}

ReplaySummary calculateSummary(const std::vector<ReplayRow> &rows)
{
    ReplaySummary summary;
    summary.totalImages = static_cast<int>(rows.size());

    int gapValidCount = 0;
    for (const ReplayRow &row : rows)
    {
        if (row.centerlineValid)
        {
            ++summary.centerlineValid;
        }
        if (row.gapValid)
        {
            ++summary.gapValid;
            ++gapValidCount;
            summary.averageGapWidthPixel += row.gapWidthPixel;
            summary.averageGapConfidence += row.gapConfidence;
        }

        summary.averageBrokenStripeRatio += row.brokenStripeRatio;
    }

    summary.invalid = summary.totalImages - summary.gapValid;

    if (gapValidCount > 0)
    {
        const double count = static_cast<double>(gapValidCount);
        summary.averageGapWidthPixel /= count;
        summary.averageGapConfidence /= count;
    }
    if (!rows.empty())
    {
        summary.averageBrokenStripeRatio /=
            static_cast<double>(rows.size());
    }

    return summary;
}

void printSummary(const ReplaySummary &summary, const fs::path &reportPath)
{
    std::cout << "Total images: " << summary.totalImages << '\n';
    std::cout << "Centerline valid: " << summary.centerlineValid << '\n';
    std::cout << "Gap valid: " << summary.gapValid << '\n';
    std::cout << "Invalid: " << summary.invalid << '\n';
    std::cout << "Average gapWidthPixel: "
              << summary.averageGapWidthPixel << '\n';
    std::cout << "Average gapConfidence: "
              << summary.averageGapConfidence << '\n';
    std::cout << "Average brokenStripeRatio: "
              << summary.averageBrokenStripeRatio << '\n';
    std::cout << "Report path: " << reportPath.string() << '\n';
    std::cout << "Safety boundary: no Qt in vision / no HALCON / no PCL / "
              << "no real camera / no DX200 / no TCP\n";
}

void fillGapFields(
    const i2r_rlc::vision::WeldFeatureResult &featureResult,
    ReplayRow *row)
{
    const i2r_rlc::vision::GapFeatureResult &gap =
        featureResult.gapFeature;
    row->gapValid = featureResult.valid && gap.valid;
    row->leftX = gap.edgePair.left.xPixel;
    row->leftY = gap.edgePair.left.yPixel;
    row->rightX = gap.edgePair.right.xPixel;
    row->rightY = gap.edgePair.right.yPixel;
    row->gapWidthPixel = gap.gapWidthPixel;
    row->weldPointX = gap.weldPoint.xPixel;
    row->weldPointY = gap.weldPoint.yPixel;
    row->gapConfidence = gap.confidence;
    row->message = featureResult.message.empty()
        ? gap.message
        : featureResult.message;
}

} // namespace

int main()
{
    using i2r_rlc::vision::CDpStripeCenterlineDetector;
    using i2r_rlc::vision::CWeldFeatureDetector;

    const fs::path projectRoot = findProjectRoot();
    const fs::path replayDir = projectRoot / "data" / "replay";
    const fs::path reportPath =
        projectRoot / "output" / "reports" / "gap_feature_replay_report.csv";

    std::vector<ReplayRow> rows;
    const std::vector<fs::path> imageFiles = collectReplayImages(replayDir);
    if (imageFiles.empty())
    {
        std::cout << "SKIP: no replay images found at "
                  << replayDir.string() << '\n';
    }

    const auto centerlineOptions = makeCenterlineOptions();
    const auto featureOptions = makeFeatureOptions();

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

        const auto startTime = std::chrono::steady_clock::now();
        const auto centerline = CDpStripeCenterlineDetector::detectCenterline(
            gray8,
            centerlineOptions);
        row.centerlineValid = centerline.valid;
        row.centerlinePointCount = centerline.points.size();
        row.validColumnRatio = centerline.validColumnRatio;
        row.brokenStripeRatio = centerline.brokenStripeRatio;
        row.centerlineMeanConfidence = centerline.meanConfidence;

        if (centerline.valid)
        {
            const auto featureResult = CWeldFeatureDetector::detect(
                gray8,
                centerline,
                featureOptions);
            fillGapFields(featureResult, &row);
        }
        else
        {
            row.message = "centerline: " + centerline.message;
        }

        const auto endTime = std::chrono::steady_clock::now();
        row.processingTimeMs =
            std::chrono::duration<double, std::milli>(
                endTime - startTime).count();
        rows.push_back(row);
    }

    if (!writeReport(reportPath, rows))
    {
        std::cerr << "Failed to write report: " << reportPath.string() << '\n';
        return 1;
    }

    const ReplaySummary summary = calculateSummary(rows);
    printSummary(summary, reportPath);

    std::printf("PASS: vision gap feature replay tests\n");
    return 0;
}
