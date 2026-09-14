/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_active_contour_replay.cpp
 * @brief       vision active contour replay 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
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

#include "feature/ActiveContourWeldFeatureDetector.h"

namespace {

namespace fs = std::filesystem;

struct FrameData {
    int frameIndex = -1;
    fs::path imagePath;
};

struct ReplayPointSet {
    std::vector<cv::Point2d> points;
};

struct ReplayRow {
    int frameIndex = -1;
    std::string inputFile;
    int imageWidth = 0;
    int imageHeight = 0;
    bool valid = false;
    std::string seamType;
    double weldPointX = 0.0;
    double weldPointY = 0.0;
    double confidence = 0.0;
    bool usedHistoryFallback = false;
    bool curveValid = false;
    std::size_t controlPointCount = 0U;
    std::size_t curvePointCount = 0U;
    std::size_t fittedLineCount = 0U;
    std::size_t featurePointCount = 0U;
    bool leftGapValid = false;
    double leftGapX = 0.0;
    double leftGapY = 0.0;
    bool rightGapValid = false;
    double rightGapX = 0.0;
    double rightGapY = 0.0;
    bool referenceValid = false;
    double referenceX = 0.0;
    double referenceY = 0.0;
    double referenceDistancePixel = -1.0;
    double processingTimeMs = 0.0;
    std::string message;
};

struct ReplaySummary {
    int totalFrames = 0;
    int processedFrames = 0;
    int validFrames = 0;
    int curveValidFrames = 0;
    int fallbackFrames = 0;
    int referenceFrames = 0;
    double confidenceSum = 0.0;
    double referenceDistanceSum = 0.0;
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

bool isPngImage(const fs::path &path)
{
    return toLower(path.extension().string()) == ".png";
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

std::vector<FrameData> collectPenzuiImages(const fs::path &imageDir)
{
    std::vector<FrameData> frames;
    if (!fs::exists(imageDir) || !fs::is_directory(imageDir))
    {
        return frames;
    }

    for (const fs::directory_entry &entry : fs::directory_iterator(imageDir))
    {
        if (!entry.is_regular_file() || !isPngImage(entry.path()))
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

std::vector<ReplayPointSet> readPointSets(const fs::path &textPath)
{
    std::vector<ReplayPointSet> pointSets;
    std::ifstream input(textPath);
    if (!input.is_open())
    {
        return pointSets;
    }

    std::string line;
    while (std::getline(input, line))
    {
        std::istringstream lineStream(line);
        std::vector<double> values;
        double value = 0.0;
        while (lineStream >> value)
        {
            values.push_back(value);
        }

        ReplayPointSet pointSet;
        for (std::size_t index = 0U; index + 1U < values.size(); index += 2U)
        {
            pointSet.points.push_back(cv::Point2d(
                values[index],
                values[index + 1U]));
        }
        pointSets.push_back(pointSet);
    }
    return pointSets;
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

i2r_rlc::vision::ActiveContourDetectionOptions makeOptions(void)
{
    i2r_rlc::vision::ActiveContourDetectionOptions options;
    options.laserWidthThresholdGrayValue = 180;
    options.laserWidthMaxPixel = 20;
    options.laserWidthSafeScale = 1.5;
    options.initialThresholdGrayValue = 110;
    options.initialClosingRadiusPixel = 3;
    options.initialControlPointStepPixel = 8;
    options.interpolationSamplesPerSegment = 6;
    options.snakeIterationCount = 30;
    options.snakeMinGrayValue = 160;
    options.firstFrameRoiHalfWidthPixel = 360;
    options.tJointRoiHalfWidthPixel = 220;
    options.buttRoiHalfWidthPixel = 220;
    options.lapRoiHalfWidthPixel = 220;
    options.tJointSplitDistancePixel = 5.0;
    options.buttSplitDistancePixel = 6.0;
    options.lapSplitDistancePixel = 10.0;
    options.ransacIterations = 40;
    options.tJointRansacDistancePixel = 4.0;
    options.buttRansacDistancePixel = 5.0;
    options.lapRansacDistancePixel = 7.0;
    options.ransacMinInlierRatio = 0.60;
    options.gapBrightThresholdGrayValue = 180;
    options.buttGapEndpointMarginPixel = 30;
    options.buttGapScanPaddingPixel = 20;
    options.maxTJointJumpPixel = 120.0;
    options.maxLapJumpPixel = 60.0;
    options.lapJumpFilterStartFrame = 8;
    options.historySize = 6;
    options.minConfidence = 0.02;
    return options;
}

std::string seamTypeName(i2r_rlc::vision::WeldSeamType seamType)
{
    using i2r_rlc::vision::WeldSeamType;
    switch (seamType)
    {
    case WeldSeamType::Butt:
        return "Butt";
    case WeldSeamType::TJoint:
        return "TJoint";
    case WeldSeamType::Lap:
        return "Lap";
    case WeldSeamType::Unknown:
    default:
        return "Unknown";
    }
}

std::string csvEscape(const std::string &text)
{
    if (text.find_first_of(",\"\n\r") == std::string::npos)
    {
        return text;
    }

    std::string escaped;
    escaped.push_back('"');
    for (char character : text)
    {
        if (character == '"')
        {
            escaped += "\"\"";
        }
        else
        {
            escaped.push_back(character);
        }
    }
    escaped.push_back('"');
    return escaped;
}

void writeReportHeader(std::ofstream *report)
{
    *report
        << "frameIndex,inputFile,imageWidth,imageHeight,valid,seamType,"
        << "weldPointX,weldPointY,confidence,usedHistoryFallback,"
        << "curveValid,controlPointCount,curvePointCount,fittedLineCount,"
        << "featurePointCount,leftGapValid,leftGapX,leftGapY,"
        << "rightGapValid,rightGapX,rightGapY,referenceValid,referenceX,"
        << "referenceY,referenceDistancePixel,processingTimeMs,message\n";
}

void writeReportRow(std::ofstream *report, const ReplayRow &row)
{
    *report
        << row.frameIndex << ','
        << csvEscape(row.inputFile) << ','
        << row.imageWidth << ','
        << row.imageHeight << ','
        << (row.valid ? 1 : 0) << ','
        << row.seamType << ','
        << row.weldPointX << ','
        << row.weldPointY << ','
        << row.confidence << ','
        << (row.usedHistoryFallback ? 1 : 0) << ','
        << (row.curveValid ? 1 : 0) << ','
        << row.controlPointCount << ','
        << row.curvePointCount << ','
        << row.fittedLineCount << ','
        << row.featurePointCount << ','
        << (row.leftGapValid ? 1 : 0) << ','
        << row.leftGapX << ','
        << row.leftGapY << ','
        << (row.rightGapValid ? 1 : 0) << ','
        << row.rightGapX << ','
        << row.rightGapY << ','
        << (row.referenceValid ? 1 : 0) << ','
        << row.referenceX << ','
        << row.referenceY << ','
        << row.referenceDistancePixel << ','
        << row.processingTimeMs << ','
        << csvEscape(row.message) << '\n';
}

ReplayRow processFrame(
    const FrameData &frame,
    const std::vector<ReplayPointSet> &simulatedWeldPointFrames,
    const i2r_rlc::vision::ActiveContourDetectionOptions &options,
    i2r_rlc::vision::ActiveContourDetectionState *state)
{
    ReplayRow row;
    row.frameIndex = frame.frameIndex;
    row.inputFile = frame.imagePath.string();

    const cv::Mat grayImage = readGray8(frame.imagePath);
    if (grayImage.empty())
    {
        row.seamType = "Unknown";
        row.message = "failed to read image";
        return row;
    }
    row.imageWidth = grayImage.cols;
    row.imageHeight = grayImage.rows;

    const auto startTime = std::chrono::steady_clock::now();
    const auto result = i2r_rlc::vision::CActiveContourWeldFeatureDetector
        ::detect(
            grayImage,
            i2r_rlc::vision::ActiveContourSeamMode::Automatic,
            options,
            state);
    const auto endTime = std::chrono::steady_clock::now();

    row.valid = result.valid;
    row.seamType = seamTypeName(result.seamType);
    row.weldPointX = result.weldPoint.xPixel;
    row.weldPointY = result.weldPoint.yPixel;
    row.confidence = result.confidence;
    row.usedHistoryFallback = result.usedHistoryFallback;
    row.curveValid = result.activeCurve.valid;
    row.controlPointCount = result.activeCurve.controlPoints.size();
    row.curvePointCount = result.activeCurve.curvePoints.size();
    row.fittedLineCount = result.fittedLines.size();
    row.featurePointCount = result.featurePoints.size();
    row.leftGapValid = result.leftGapPoint.valid;
    row.leftGapX = result.leftGapPoint.xPixel;
    row.leftGapY = result.leftGapPoint.yPixel;
    row.rightGapValid = result.rightGapPoint.valid;
    row.rightGapX = result.rightGapPoint.xPixel;
    row.rightGapY = result.rightGapPoint.yPixel;
    row.message = result.message;
    row.processingTimeMs =
        std::chrono::duration<double, std::milli>(endTime - startTime)
            .count();

    if (frame.frameIndex >= 0
        && frame.frameIndex < static_cast<int>(simulatedWeldPointFrames.size())
        && !simulatedWeldPointFrames[frame.frameIndex].points.empty())
    {
        const cv::Point2d reference =
            simulatedWeldPointFrames[frame.frameIndex].points.front();
        row.referenceValid = true;
        row.referenceX = reference.x;
        row.referenceY = reference.y;
        if (result.weldPoint.valid)
        {
            const double dx = result.weldPoint.xPixel - reference.x;
            const double dy = result.weldPoint.yPixel - reference.y;
            row.referenceDistancePixel = std::sqrt(dx * dx + dy * dy);
        }
    }

    return row;
}

} // namespace

int main()
{
    const fs::path projectRoot = findProjectRoot();
    const fs::path penzuiDir = projectRoot / "data" / "replay" / "penzui";
    const fs::path imageDir = penzuiDir / "imagecatch";
    const fs::path simulatedWeldPointPath = penzuiDir / "simulatedWeldpt.txt";
    const fs::path reportPath =
        projectRoot / "output" / "reports"
        / "active_contour_replay_report.csv";

    fs::create_directories(reportPath.parent_path());
    std::ofstream report(reportPath.string(), std::ios::out | std::ios::trunc);
    if (!report.is_open())
    {
        std::cerr << "Failed to create report: "
                  << reportPath.string() << '\n';
        return 1;
    }
    report << std::fixed << std::setprecision(3);
    writeReportHeader(&report);

    const std::vector<FrameData> frames = collectPenzuiImages(imageDir);
    if (frames.empty() || !fs::exists(simulatedWeldPointPath))
    {
        std::cout << "SKIP: penzui replay data is incomplete at "
                  << penzuiDir.string() << '\n';
        std::cout << "Report path: " << reportPath.string() << '\n';
        return 0;
    }

    const std::vector<ReplayPointSet> simulatedWeldPointFrames =
        readPointSets(simulatedWeldPointPath);
    if (simulatedWeldPointFrames.empty())
    {
        std::cout << "SKIP: penzui simulated weld point file is empty\n";
        std::cout << "Report path: " << reportPath.string() << '\n';
        return 0;
    }

    const auto options = makeOptions();
    i2r_rlc::vision::ActiveContourDetectionState state;
    ReplaySummary summary;
    const int maxFramesToProcess = 40;
    const int frameCount = std::min<int>(
        maxFramesToProcess,
        static_cast<int>(frames.size()));

    for (int index = 0; index < frameCount; ++index)
    {
        const ReplayRow row = processFrame(
            frames[index],
            simulatedWeldPointFrames,
            options,
            &state);
        writeReportRow(&report, row);

        ++summary.totalFrames;
        if (row.imageWidth > 0 && row.imageHeight > 0)
        {
            ++summary.processedFrames;
        }
        if (row.valid)
        {
            ++summary.validFrames;
            summary.confidenceSum += row.confidence;
        }
        if (row.curveValid)
        {
            ++summary.curveValidFrames;
        }
        if (row.usedHistoryFallback)
        {
            ++summary.fallbackFrames;
        }
        if (row.referenceDistancePixel >= 0.0)
        {
            ++summary.referenceFrames;
            summary.referenceDistanceSum += row.referenceDistancePixel;
        }
    }

    const double averageConfidence = summary.validFrames > 0
        ? summary.confidenceSum / static_cast<double>(summary.validFrames)
        : 0.0;
    const double averageReferenceDistance = summary.referenceFrames > 0
        ? summary.referenceDistanceSum
            / static_cast<double>(summary.referenceFrames)
        : std::numeric_limits<double>::quiet_NaN();

    std::cout << "Total sampled frames: " << summary.totalFrames << '\n';
    std::cout << "Processed frames: " << summary.processedFrames << '\n';
    std::cout << "Valid active-contour frames: "
              << summary.validFrames << '\n';
    std::cout << "Curve valid frames: "
              << summary.curveValidFrames << '\n';
    std::cout << "History fallback frames: "
              << summary.fallbackFrames << '\n';
    std::cout << "Average confidence: "
              << averageConfidence << '\n';
    if (summary.referenceFrames > 0)
    {
        std::cout << "Average reference distance pixel: "
                  << averageReferenceDistance << '\n';
    }
    std::cout << "Report path: " << reportPath.string() << '\n';
    std::cout << "Safety boundary: no Qt in vision / no HALCON / no PCL / "
              << "no real camera / no DX200 / no TCP\n";

    if (summary.processedFrames == 0)
    {
        std::cerr << "No penzui image could be decoded\n";
        return 2;
    }
    if (summary.curveValidFrames == 0)
    {
        std::cerr << "Active contour could not initialize on penzui replay\n";
        return 3;
    }

    std::printf("PASS: vision active contour replay tests\n");
    return 0;
}
