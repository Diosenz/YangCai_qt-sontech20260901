/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_skeleton_halcon_diff_experiment.cpp
 * @brief       vision skeleton halcon diff experiment 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <system_error>

#include "feature/SimulatedLaserWeldDetector.h"
#include "stripe/WeldSkeletonFeatureTools.h"

namespace {

namespace fs = std::filesystem;

struct FrameData {
    int frameIndex = -1;
    fs::path imagePath;
};

struct SkeletonReference {
    bool valid = false;
    cv::Mat mask;
    std::vector<cv::Point2d> points;
};

struct SkeletonMetrics {
    bool hasReference = false;
    int currentPointCount = 0;
    int referencePointCount = 0;
    double pixelIou = 0.0;
    double chamferDistancePixel = 0.0;
    double hausdorffDistancePixel = 0.0;
    double meanColumnYErrorPixel = 0.0;
    int currentEndpointCount = 0;
    int referenceEndpointCount = 0;
    int currentBranchPointCount = 0;
    int referenceBranchPointCount = 0;
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
        if (frameIndex >= 0)
        {
            frames.push_back(FrameData{frameIndex, entry.path()});
        }
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

std::vector<std::string> splitCsvLine(const std::string &line)
{
    std::vector<std::string> cells;
    std::string cell;
    std::istringstream stream(line);
    while (std::getline(stream, cell, ','))
    {
        cells.push_back(cell);
    }
    return cells;
}

bool startsWithDigitOrSign(const std::string &text)
{
    if (text.empty())
    {
        return false;
    }

    const char first = text.front();
    return std::isdigit(static_cast<unsigned char>(first))
        || first == '-'
        || first == '+';
}

std::map<int, std::vector<cv::Point2d>> readSkeletonPointCsv(
    const fs::path &path)
{
    std::map<int, std::vector<cv::Point2d>> pointsByFrame;
    std::ifstream input(path);
    if (!input.is_open())
    {
        return pointsByFrame;
    }

    std::string line;
    while (std::getline(input, line))
    {
        const std::vector<std::string> cells = splitCsvLine(line);
        if (cells.size() < 3U || !startsWithDigitOrSign(cells[0]))
        {
            continue;
        }

        const int frame = std::stoi(cells[0]);
        const double x = std::stod(cells[1]);
        const double y = std::stod(cells[2]);
        pointsByFrame[frame].push_back(cv::Point2d(x, y));
    }
    return pointsByFrame;
}

cv::Mat makeMaskFromPoints(
    const cv::Size &size,
    const std::vector<cv::Point2d> &points)
{
    cv::Mat mask = cv::Mat::zeros(size, CV_8UC1);
    for (const cv::Point2d &point : points)
    {
        const int x = static_cast<int>(std::round(point.x));
        const int y = static_cast<int>(std::round(point.y));
        if (x >= 0 && y >= 0 && x < mask.cols && y < mask.rows)
        {
            mask.at<unsigned char>(y, x) = 255;
        }
    }
    return mask;
}

std::vector<cv::Point2d> collectMaskPoints(const cv::Mat &mask)
{
    return i2r_rlc::vision::CWeldSkeletonFeatureTools::collectSkeletonPoints(mask);
}

cv::Mat readReferenceMask(const fs::path &maskDir, int frameIndex)
{
    const std::vector<fs::path> candidates = {
        maskDir / ("image_" + std::to_string(frameIndex) + ".png"),
        maskDir / (std::to_string(frameIndex) + ".png")
    };
    for (const fs::path &candidate : candidates)
    {
        if (!fs::exists(candidate))
        {
            continue;
        }

        cv::Mat mask = cv::imread(candidate.string(), cv::IMREAD_GRAYSCALE);
        if (!mask.empty())
        {
            cv::threshold(mask, mask, 0, 255, cv::THRESH_BINARY);
            return mask;
        }
    }
    return cv::Mat();
}

int neighborCount(const cv::Mat &mask, int xPixel, int yPixel)
{
    int count = 0;
    for (int dy = -1; dy <= 1; ++dy)
    {
        for (int dx = -1; dx <= 1; ++dx)
        {
            if (dx == 0 && dy == 0)
            {
                continue;
            }

            const int x = xPixel + dx;
            const int y = yPixel + dy;
            if (x < 0 || y < 0 || x >= mask.cols || y >= mask.rows)
            {
                continue;
            }

            if (mask.at<unsigned char>(y, x) > 0)
            {
                ++count;
            }
        }
    }
    return count;
}

void countTopology(const cv::Mat &mask, int *endpointCount, int *branchCount)
{
    *endpointCount = 0;
    *branchCount = 0;
    if (mask.empty())
    {
        return;
    }

    for (int y = 0; y < mask.rows; ++y)
    {
        for (int x = 0; x < mask.cols; ++x)
        {
            if (mask.at<unsigned char>(y, x) == 0)
            {
                continue;
            }

            const int neighbors = neighborCount(mask, x, y);
            if (neighbors == 1)
            {
                ++(*endpointCount);
            }
            else if (neighbors >= 3)
            {
                ++(*branchCount);
            }
        }
    }
}

double nearestDistance(
    const cv::Point2d &point,
    const std::vector<cv::Point2d> &candidates)
{
    double best = std::numeric_limits<double>::max();
    for (const cv::Point2d &candidate : candidates)
    {
        const double dx = point.x - candidate.x;
        const double dy = point.y - candidate.y;
        best = std::min(best, std::sqrt(dx * dx + dy * dy));
    }
    return best == std::numeric_limits<double>::max() ? 0.0 : best;
}

double directedMeanDistance(
    const std::vector<cv::Point2d> &source,
    const std::vector<cv::Point2d> &target)
{
    if (source.empty() || target.empty())
    {
        return 0.0;
    }

    double sum = 0.0;
    for (const cv::Point2d &point : source)
    {
        sum += nearestDistance(point, target);
    }
    return sum / static_cast<double>(source.size());
}

double directedHausdorffDistance(
    const std::vector<cv::Point2d> &source,
    const std::vector<cv::Point2d> &target)
{
    if (source.empty() || target.empty())
    {
        return 0.0;
    }

    double maxDistance = 0.0;
    for (const cv::Point2d &point : source)
    {
        maxDistance = std::max(maxDistance, nearestDistance(point, target));
    }
    return maxDistance;
}

double pixelIou(const cv::Mat &leftMask, const cv::Mat &rightMask)
{
    if (leftMask.empty() || rightMask.empty()
        || leftMask.size() != rightMask.size())
    {
        return 0.0;
    }

    cv::Mat intersectionMask;
    cv::Mat unionMask;
    cv::bitwise_and(leftMask, rightMask, intersectionMask);
    cv::bitwise_or(leftMask, rightMask, unionMask);
    const int unionCount = cv::countNonZero(unionMask);
    if (unionCount == 0)
    {
        return 0.0;
    }
    return static_cast<double>(cv::countNonZero(intersectionMask))
        / static_cast<double>(unionCount);
}

double meanColumnYError(
    const std::vector<cv::Point2d> &current,
    const std::vector<cv::Point2d> &reference)
{
    std::map<int, std::vector<double>> currentRows;
    std::map<int, std::vector<double>> referenceRows;
    for (const cv::Point2d &point : current)
    {
        currentRows[static_cast<int>(std::round(point.x))].push_back(point.y);
    }
    for (const cv::Point2d &point : reference)
    {
        referenceRows[static_cast<int>(std::round(point.x))].push_back(point.y);
    }

    double sum = 0.0;
    int count = 0;
    for (const auto &entry : currentRows)
    {
        const auto refIt = referenceRows.find(entry.first);
        if (refIt == referenceRows.end())
        {
            continue;
        }

        const auto meanOf = [](const std::vector<double> &values) {
            double total = 0.0;
            for (double value : values)
            {
                total += value;
            }
            return total / static_cast<double>(values.size());
        };
        sum += std::abs(meanOf(entry.second) - meanOf(refIt->second));
        ++count;
    }
    return count > 0 ? sum / static_cast<double>(count) : 0.0;
}

SkeletonMetrics computeMetrics(
    const cv::Mat &currentMask,
    const std::vector<cv::Point2d> &currentPoints,
    const SkeletonReference &reference)
{
    SkeletonMetrics metrics;
    metrics.currentPointCount = static_cast<int>(currentPoints.size());
    countTopology(
        currentMask,
        &metrics.currentEndpointCount,
        &metrics.currentBranchPointCount);

    if (!reference.valid)
    {
        return metrics;
    }

    metrics.hasReference = true;
    metrics.referencePointCount = static_cast<int>(reference.points.size());
    metrics.pixelIou = pixelIou(currentMask, reference.mask);
    metrics.chamferDistancePixel = 0.5 * (
        directedMeanDistance(currentPoints, reference.points)
        + directedMeanDistance(reference.points, currentPoints));
    metrics.hausdorffDistancePixel = std::max(
        directedHausdorffDistance(currentPoints, reference.points),
        directedHausdorffDistance(reference.points, currentPoints));
    metrics.meanColumnYErrorPixel =
        meanColumnYError(currentPoints, reference.points);
    countTopology(
        reference.mask,
        &metrics.referenceEndpointCount,
        &metrics.referenceBranchPointCount);
    return metrics;
}

void writeReportHeader(std::ofstream &report)
{
    report << "source,frame,has_halcon_baseline,current_valid,"
        "current_points,halcon_points,pixel_iou,chamfer_distance_px,"
        "hausdorff_distance_px,mean_column_y_error_px,current_endpoints,"
        "halcon_endpoints,endpoint_delta,current_junctions,halcon_junctions,"
        "junction_delta,registration_error_to_sim_weld_px,message\n";
}

void writeReportRow(
    std::ofstream &report,
    const char *source,
    int frameIndex,
    bool currentValid,
    const SkeletonMetrics &metrics,
    double registrationError,
    const std::string &message)
{
    report << source << ','
        << frameIndex << ','
        << (metrics.hasReference ? 1 : 0) << ','
        << (currentValid ? 1 : 0) << ','
        << metrics.currentPointCount << ','
        << metrics.referencePointCount << ','
        << metrics.pixelIou << ','
        << metrics.chamferDistancePixel << ','
        << metrics.hausdorffDistancePixel << ','
        << metrics.meanColumnYErrorPixel << ','
        << metrics.currentEndpointCount << ','
        << metrics.referenceEndpointCount << ','
        << (metrics.currentEndpointCount - metrics.referenceEndpointCount) << ','
        << metrics.currentBranchPointCount << ','
        << metrics.referenceBranchPointCount << ','
        << (metrics.currentBranchPointCount - metrics.referenceBranchPointCount)
        << ','
        << registrationError << ','
        << '"' << message << '"' << '\n';
}

std::ofstream openReportFile(
    const fs::path &preferredPath,
    fs::path *actualPath)
{
    std::error_code createError;
    fs::create_directories(preferredPath.parent_path(), createError);

    std::ofstream report(
        preferredPath.string(),
        std::ios::out | std::ios::trunc);
    if (report.is_open())
    {
        *actualPath = preferredPath;
        return report;
    }

    const fs::path fallbackPath = fs::current_path() / preferredPath.filename();
    report.open(fallbackPath.string(), std::ios::out | std::ios::trunc);
    if (report.is_open())
    {
        *actualPath = fallbackPath;
    }
    return report;
}

cv::Mat makeSyntheticRegion(int kind)
{
    cv::Mat image(120, 160, CV_8UC1, cv::Scalar(0));
    if (kind == 0)
    {
        cv::line(image, cv::Point(15, 60), cv::Point(145, 60),
            cv::Scalar(255), 9, cv::LINE_8);
    }
    else if (kind == 1)
    {
        cv::line(image, cv::Point(20, 85), cv::Point(140, 35),
            cv::Scalar(255), 9, cv::LINE_8);
    }
    else if (kind == 2)
    {
        cv::line(image, cv::Point(20, 80), cv::Point(80, 45),
            cv::Scalar(255), 9, cv::LINE_8);
        cv::line(image, cv::Point(80, 45), cv::Point(140, 80),
            cv::Scalar(255), 9, cv::LINE_8);
        cv::line(image, cv::Point(80, 20), cv::Point(80, 45),
            cv::Scalar(255), 9, cv::LINE_8);
    }
    else
    {
        cv::line(image, cv::Point(20, 70), cv::Point(65, 55),
            cv::Scalar(255), 7, cv::LINE_8);
        cv::line(image, cv::Point(95, 55), cv::Point(140, 70),
            cv::Scalar(255), 7, cv::LINE_8);
    }
    return image;
}

SkeletonReference makeReferenceFromMask(const cv::Mat &mask)
{
    SkeletonReference reference;
    reference.valid = !mask.empty();
    reference.mask = mask.clone();
    reference.points = collectMaskPoints(reference.mask);
    return reference;
}

SkeletonReference loadReference(
    const fs::path &maskDir,
    const std::map<int, std::vector<cv::Point2d>> &pointsByFrame,
    int frameIndex,
    const cv::Size &size)
{
    SkeletonReference reference;
    cv::Mat mask = readReferenceMask(maskDir, frameIndex);
    const auto pointsIt = pointsByFrame.find(frameIndex);
    if (mask.empty() && pointsIt != pointsByFrame.end())
    {
        mask = makeMaskFromPoints(size, pointsIt->second);
    }

    if (mask.empty())
    {
        return reference;
    }

    reference.valid = true;
    reference.mask = mask;
    reference.points = pointsIt != pointsByFrame.end()
        ? pointsIt->second
        : collectMaskPoints(mask);
    return reference;
}

double registrationErrorToSimWeld(
    const cv::Mat &gray8,
    const i2r_rlc::digital_twin::DigitalTwinVisionLaserObservation &observation)
{
    if (!observation.hasWeldPointImage_px)
    {
        return 0.0;
    }

    i2r_rlc::vision::SimulatedLaserMatchingOptions options;
    options.skeletonThresholdValue = 35;
    options.minConfidence = 0.0;
    const auto result = i2r_rlc::vision::CSimulatedLaserRegistrationMatcher
        ::match(gray8, observation, options);
    if (!result.valid)
    {
        return 0.0;
    }

    const double dx = result.weldPoint.x - observation.weldPointImage_px.x_px;
    const double dy = result.weldPoint.y - observation.weldPointImage_px.y_px;
    return std::sqrt(dx * dx + dy * dy);
}

} // namespace

int main()
{
    using i2r_rlc::vision::LaserSkeletonExtractionOptions;
    using i2r_rlc::vision::CSimulatedLaserReplayParser;
    using i2r_rlc::vision::CWeldSkeletonFeatureTools;

    const fs::path projectRoot = findProjectRoot();
    fs::path reportPath;
    std::ofstream report = openReportFile(
        projectRoot / "output" / "reports" / "skeleton_halcon_diff_report.csv",
        &reportPath);
    if (!report.is_open())
    {
        std::printf("SKIP: failed to create skeleton diff report\n");
        return 0;
    }
    writeReportHeader(report);

    int processedRows = 0;
    int referenceRows = 0;

    // 1. 合成图自检：用当前 skeleton 作为 reference，验证指标计算链路。
    for (int kind = 0; kind < 4; ++kind)
    {
        const cv::Mat region = makeSyntheticRegion(kind);
        const cv::Mat currentMask =
            CWeldSkeletonFeatureTools::thinBinaryMask(region, 100);
        const std::vector<cv::Point2d> currentPoints =
            CWeldSkeletonFeatureTools::collectSkeletonPoints(currentMask);
        const SkeletonReference reference = makeReferenceFromMask(currentMask);
        const SkeletonMetrics metrics =
            computeMetrics(currentMask, currentPoints, reference);
        writeReportRow(
            report,
            "synthetic_self_reference",
            kind,
            !currentPoints.empty(),
            metrics,
            0.0,
            "self reference metric smoke test");
        ++processedRows;
        ++referenceRows;
    }

    // 2. replay 图：当前 OpenCV skeleton 与外部 HALCON skeleton 导出对比。
    const fs::path replayRoot = projectRoot / "data" / "replay" / "penzui";
    const fs::path imageDir = replayRoot / "imagecatch";
    const fs::path laserPath = replayRoot / "simulatedLaser.txt";
    const fs::path weldPath = replayRoot / "simulatedWeldpt.txt";
    const std::vector<FrameData> frames = collectImages(imageDir);
    const auto parseResult = CSimulatedLaserReplayParser::parse(
        readTextFile(laserPath),
        readTextFile(weldPath));
    const std::map<int, std::vector<cv::Point2d>> halconPoints =
        readSkeletonPointCsv(replayRoot / "halcon_skeleton_points.csv");
    const fs::path halconMaskDir = replayRoot / "halcon_skeleton_masks";

    LaserSkeletonExtractionOptions options;
    options.thresholdValue = 35;
    options.minComponentArea = 20;
    options.enableLaserRegionPruning = true;
    options.maxThinningIterations = 40;
    options.minConfidence = 0.0;
    options.enableHoughFallback = false;
    const int maxReplayFrames = 5;
    int replayFrames = 0;
    for (const FrameData &frame : frames)
    {
        if (replayFrames >= maxReplayFrames)
        {
            break;
        }

        const cv::Mat gray8 = readGray8(frame.imagePath);
        if (gray8.empty())
        {
            continue;
        }

        const auto current =
            CWeldSkeletonFeatureTools::extractLaserSkeleton(gray8, options);
        const SkeletonReference reference = loadReference(
            halconMaskDir,
            halconPoints,
            frame.frameIndex,
            gray8.size());
        if (reference.valid)
        {
            ++referenceRows;
        }

        double registrationError = 0.0;
        if (parseResult.valid
            && frame.frameIndex
                < static_cast<int>(parseResult.observations.size()))
        {
            registrationError = registrationErrorToSimWeld(
                gray8,
                parseResult.observations[
                    static_cast<std::size_t>(frame.frameIndex)]);
        }

        const SkeletonMetrics metrics = computeMetrics(
            current.skeletonMask,
            current.skeletonPoints,
            reference);
        writeReportRow(
            report,
            "penzui_replay",
            frame.frameIndex,
            current.valid,
            metrics,
            registrationError,
            reference.valid
                ? current.message
                : "missing HALCON skeleton baseline");
        ++processedRows;
        ++replayFrames;
    }

    std::ofstream summary(
        reportPath.parent_path() / "skeleton_halcon_diff_summary.md");
    if (summary.is_open())
    {
        summary << "# Skeleton HALCON Difference Summary\n\n";
        summary << "- Rows processed: " << processedRows << "\n";
        summary << "- Rows with reference: " << referenceRows << "\n";
        summary << "- Expected HALCON point CSV: "
            << "`data/replay/penzui/halcon_skeleton_points.csv`\n";
        summary << "- Expected HALCON mask directory: "
            << "`data/replay/penzui/halcon_skeleton_masks/`\n";
        summary << "- Missing HALCON export is not a failure; it leaves replay "
            "rows as current-only diagnostics.\n";
    }

    if (processedRows == 0)
    {
        return 1;
    }

    std::printf(
        "PASS: skeleton HALCON diff experiment scaffold, rows=%d reference=%d\n",
        processedRows,
        referenceRows);
    return 0;
}


