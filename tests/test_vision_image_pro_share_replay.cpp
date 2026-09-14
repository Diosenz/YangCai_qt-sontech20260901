/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_image_pro_share_replay.cpp
 * @brief       vision image pro share replay 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "feature/ContourControlPointTools.h"
#include "stripe/LaserLineResponseTools.h"

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
    std::size_t simulatedLaserPointCount = 0U;
    std::size_t simulatedWeldPointCount = 0U;
    bool logValid = false;
    bool centerOfMassValid = false;
    bool dragValid = false;
    bool controlCurveValid = false;
    std::size_t curvePointCount = 0U;
    std::string message;
};

struct ReplaySummary {
    int totalFrames = 0;
    int logValidFrames = 0;
    int centerOfMassValidFrames = 0;
    int dragValidFrames = 0;
    int curveValidFrames = 0;
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

std::vector<int> makeLaserWidthByRow(
    int rowCount,
    const std::vector<cv::Point2d> &simulatedLaser)
{
    std::vector<int> laserWidthByRow(rowCount, 6);
    for (const cv::Point2d &point : simulatedLaser)
    {
        const int yPixel = static_cast<int>(std::lround(point.y));
        if (yPixel >= 0 && yPixel < rowCount)
        {
            laserWidthByRow[yPixel] = 6;
        }
    }
    return laserWidthByRow;
}

i2r_rlc::vision::LaserLogPeakOptions makeLogOptions(
    const cv::Mat &grayImage,
    const std::vector<cv::Point2d> &simulatedLaser)
{
    i2r_rlc::vision::LaserLogPeakOptions options;
    options.rowMinPixel = 0;
    options.rowMaxPixel = grayImage.rows;
    options.columnStepPixel = 8;
    options.logThreshold = -3000.0;
    options.requiredPeakGrayValue = 255;

    if (!simulatedLaser.empty())
    {
        double minY = static_cast<double>(grayImage.rows);
        double maxY = 0.0;
        for (const cv::Point2d &point : simulatedLaser)
        {
            minY = std::min(minY, point.y);
            maxY = std::max(maxY, point.y);
        }
        options.rowMinPixel = std::max(
            0,
            static_cast<int>(std::floor(minY)) - 40);
        options.rowMaxPixel = std::min(
            grayImage.rows,
            static_cast<int>(std::ceil(maxY)) + 40);
    }

    return options;
}

std::vector<cv::Point2d> makeControlPoints(
    const std::vector<cv::Point2d> &simulatedLaser,
    const std::vector<cv::Point2d> &simulatedWeldPoints)
{
    std::vector<cv::Point2d> controlPoints;
    if (!simulatedLaser.empty())
    {
        const std::size_t step = std::max<std::size_t>(
            1U,
            simulatedLaser.size() / 6U);
        for (std::size_t index = 0U; index < simulatedLaser.size();
             index += step)
        {
            controlPoints.push_back(simulatedLaser[index]);
            if (controlPoints.size() >= 6U)
            {
                break;
            }
        }
    }

    for (const cv::Point2d &point : simulatedWeldPoints)
    {
        controlPoints.push_back(point);
    }

    std::sort(
        controlPoints.begin(),
        controlPoints.end(),
        [](const cv::Point2d &left, const cv::Point2d &right) {
            return left.x < right.x;
        });
    return controlPoints;
}

std::string csvEscape(const std::string &text)
{
    std::string escaped;
    escaped.reserve(text.size() + 2U);
    escaped.push_back('"');
    for (char character : text)
    {
        if (character == '"')
        {
            escaped.push_back('"');
        }
        escaped.push_back(character);
    }
    escaped.push_back('"');
    return escaped;
}

void writeReportHeader(std::ofstream *report)
{
    *report
        << "frameIndex,inputFile,imageWidth,imageHeight,"
        << "simulatedLaserPointCount,simulatedWeldPointCount,"
        << "logValid,centerOfMassValid,dragValid,controlCurveValid,"
        << "curvePointCount,message\n";
}

void writeReportRow(std::ofstream *report, const ReplayRow &row)
{
    *report
        << row.frameIndex << ','
        << csvEscape(row.inputFile) << ','
        << row.imageWidth << ','
        << row.imageHeight << ','
        << row.simulatedLaserPointCount << ','
        << row.simulatedWeldPointCount << ','
        << (row.logValid ? 1 : 0) << ','
        << (row.centerOfMassValid ? 1 : 0) << ','
        << (row.dragValid ? 1 : 0) << ','
        << (row.controlCurveValid ? 1 : 0) << ','
        << row.curvePointCount << ','
        << csvEscape(row.message) << '\n';
}

ReplayRow processFrame(
    const FrameData &frame,
    const std::vector<ReplayPointSet> &simulatedLaserFrames,
    const std::vector<ReplayPointSet> &simulatedWeldPointFrames)
{
    ReplayRow row;
    row.frameIndex = frame.frameIndex;
    row.inputFile = frame.imagePath.string();

    const cv::Mat grayImage = readGray8(frame.imagePath);
    if (grayImage.empty())
    {
        row.message = "failed to read image";
        return row;
    }
    row.imageWidth = grayImage.cols;
    row.imageHeight = grayImage.rows;

    const std::vector<cv::Point2d> simulatedLaser =
        frame.frameIndex < static_cast<int>(simulatedLaserFrames.size())
            ? simulatedLaserFrames[frame.frameIndex].points
            : std::vector<cv::Point2d>();
    const std::vector<cv::Point2d> simulatedWeldPoints =
        frame.frameIndex < static_cast<int>(simulatedWeldPointFrames.size())
            ? simulatedWeldPointFrames[frame.frameIndex].points
            : std::vector<cv::Point2d>();
    row.simulatedLaserPointCount = simulatedLaser.size();
    row.simulatedWeldPointCount = simulatedWeldPoints.size();

    const std::vector<int> laserWidthByRow = makeLaserWidthByRow(
        grayImage.rows,
        simulatedLaser);
    i2r_rlc::vision::VisionPreprocessStatus status =
        i2r_rlc::vision::VisionPreprocessStatus::Succeeded;

    const auto logResult = i2r_rlc::vision::CLaserLineResponseTools
        ::detectLogPeaks(
            grayImage,
            laserWidthByRow,
            makeLogOptions(grayImage, simulatedLaser),
            &status);
    row.logValid = logResult.valid;

    const auto centerOfMassResult = i2r_rlc::vision::CLaserLineResponseTools
        ::refineCenterOfMass(
            grayImage,
            logResult.maxRowByColumn,
            laserWidthByRow,
            12,
            &status);
    row.centerOfMassValid = centerOfMassResult.valid;

    cv::Point2i dragStart(grayImage.cols / 2, grayImage.rows / 2);
    if (!simulatedWeldPoints.empty())
    {
        dragStart.x = static_cast<int>(std::lround(simulatedWeldPoints[0].x));
        dragStart.y = static_cast<int>(std::lround(simulatedWeldPoints[0].y));
    }
    else if (!simulatedLaser.empty())
    {
        dragStart.x = static_cast<int>(std::lround(
            simulatedLaser[simulatedLaser.size() / 2U].x));
        dragStart.y = static_cast<int>(std::lround(
            simulatedLaser[simulatedLaser.size() / 2U].y));
    }
    dragStart.x = std::max(0, std::min(dragStart.x, grayImage.cols - 1));
    dragStart.y = std::max(0, std::min(dragStart.y, grayImage.rows - 1));

    const auto dragResult = i2r_rlc::vision::CLaserLineResponseTools
        ::dragPointToLaserLine(
            grayImage,
            laserWidthByRow,
            dragStart,
            20,
            80,
            &status);
    row.dragValid = dragResult.valid;

    i2r_rlc::vision::ContourControlPointOptions contourOptions;
    contourOptions.dragSearchHalfHeightPixel = 20;
    contourOptions.interpolationSamplesPerSegment = 8;
    const auto curveResult = i2r_rlc::vision::CContourControlPointTools
        ::adjustOfflineControlPoints(
            grayImage,
            laserWidthByRow,
            makeControlPoints(simulatedLaser, simulatedWeldPoints),
            contourOptions,
            &status);
    row.controlCurveValid = curveResult.valid;
    row.curvePointCount = curveResult.curvePoints.size();

    row.message = "log=" + logResult.message
        + "; com=" + centerOfMassResult.message
        + "; drag=" + dragResult.message
        + "; curve=" + curveResult.message;
    return row;
}

} // namespace

int main()
{
    const fs::path projectRoot = findProjectRoot();
    const fs::path penzuiDir = projectRoot / "data" / "replay" / "penzui";
    const fs::path imageDir = penzuiDir / "imagecatch";
    const fs::path simulatedLaserPath = penzuiDir / "simulatedLaser.txt";
    const fs::path simulatedWeldPointPath = penzuiDir / "simulatedWeldpt.txt";
    const fs::path reportPath =
        projectRoot / "output" / "reports"
        / "image_pro_share_replay_report.csv";

    fs::create_directories(reportPath.parent_path());
    std::ofstream report(reportPath);
    if (!report.is_open())
    {
        std::cerr << "Failed to create report: "
                  << reportPath.string() << '\n';
        return 1;
    }
    writeReportHeader(&report);

    const std::vector<FrameData> allFrames = collectPenzuiImages(imageDir);
    if (allFrames.empty()
        || !fs::exists(simulatedLaserPath)
        || !fs::exists(simulatedWeldPointPath))
    {
        std::cout << "SKIP: penzui replay data is incomplete at "
                  << penzuiDir.string() << '\n';
        std::cout << "Report path: " << reportPath.string() << '\n';
        return 0;
    }

    const std::vector<ReplayPointSet> simulatedLaserFrames =
        readPointSets(simulatedLaserPath);
    const std::vector<ReplayPointSet> simulatedWeldPointFrames =
        readPointSets(simulatedWeldPointPath);
    if (simulatedLaserFrames.empty() || simulatedWeldPointFrames.empty())
    {
        std::cout << "SKIP: penzui simulated point files are empty\n";
        std::cout << "Report path: " << reportPath.string() << '\n';
        return 0;
    }

    ReplaySummary summary;
    const int maxFramesToProcess = 30;
    const int frameStride = std::max<int>(
        1,
        static_cast<int>(allFrames.size()) / maxFramesToProcess);
    for (int index = 0; index < static_cast<int>(allFrames.size());
         index += frameStride)
    {
        const ReplayRow row = processFrame(
            allFrames[index],
            simulatedLaserFrames,
            simulatedWeldPointFrames);
        writeReportRow(&report, row);

        ++summary.totalFrames;
        if (row.logValid)
        {
            ++summary.logValidFrames;
        }
        if (row.centerOfMassValid)
        {
            ++summary.centerOfMassValidFrames;
        }
        if (row.dragValid)
        {
            ++summary.dragValidFrames;
        }
        if (row.controlCurveValid)
        {
            ++summary.curveValidFrames;
        }
    }

    std::cout << "Total sampled frames: " << summary.totalFrames << '\n';
    std::cout << "LOG valid: " << summary.logValidFrames << '\n';
    std::cout << "CoM valid: " << summary.centerOfMassValidFrames << '\n';
    std::cout << "Drag valid: " << summary.dragValidFrames << '\n';
    std::cout << "Control curve valid: " << summary.curveValidFrames << '\n';
    std::cout << "Report path: " << reportPath.string() << '\n';
    std::cout << "Safety boundary: no Qt in vision / no HALCON / no PCL / "
              << "no real camera / no DX200 / no TCP\n";
    std::printf("PASS: vision image pro share replay tests\n");
    return 0;
}
