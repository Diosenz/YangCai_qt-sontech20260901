/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_shape_matching_experiment.cpp
 * @brief       vision shape matching experiment 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <system_error>

#include "feature/SimulatedLaserWeldDetector.h"

namespace {

namespace fs = std::filesystem;

struct FrameData {
    int frameIndex = -1;
    fs::path imagePath;
};

struct BaselinePoint {
    bool valid = false;
    double x = 0.0;
    double y = 0.0;
};

struct MethodStats {
    int totalCount = 0;
    int validCount = 0;
    int halconComparableCount = 0;
    double sumElapsedMs = 0.0;
    double sumSimErrorPixel = 0.0;
    double sumHalconErrorPixel = 0.0;
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

        frames.push_back(FrameData{frameIndex, entry.path()});
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

std::map<int, BaselinePoint> readHalconBaseline(const fs::path &path)
{
    std::map<int, BaselinePoint> baseline;
    std::ifstream input(path);
    if (!input.is_open())
    {
        return baseline;
    }

    std::string line;
    bool firstLine = true;
    while (std::getline(input, line))
    {
        if (line.empty())
        {
            continue;
        }

        const std::vector<std::string> cells = splitCsvLine(line);
        if (cells.size() < 4U)
        {
            continue;
        }

        if (firstLine && !std::isdigit(
            static_cast<unsigned char>(cells[0].empty() ? 'x' : cells[0][0])))
        {
            firstLine = false;
            continue;
        }
        firstLine = false;

        BaselinePoint point;
        point.valid = std::stoi(cells[1]) != 0;
        point.x = std::stod(cells[2]);
        point.y = std::stod(cells[3]);
        baseline[std::stoi(cells[0])] = point;
    }
    return baseline;
}

double errorToPoint(
    const cv::Point2d &point,
    const BaselinePoint &baseline)
{
    if (!baseline.valid)
    {
        return 0.0;
    }

    const double dx = point.x - baseline.x;
    const double dy = point.y - baseline.y;
    return std::sqrt(dx * dx + dy * dy);
}

double errorToSimWeld(
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

void writeResultRow(
    std::ofstream &report,
    int frameIndex,
    const char *method,
    const i2r_rlc::vision::SimulatedLaserMatchingResult &result,
    const i2r_rlc::digital_twin::DigitalTwinVisionLaserObservation &observation,
    const std::map<int, BaselinePoint> &baseline,
    double elapsedMs)
{
    const auto baselineIt = baseline.find(frameIndex);
    const BaselinePoint halcon = baselineIt == baseline.end()
        ? BaselinePoint{}
        : baselineIt->second;
    report << frameIndex << ','
        << method << ','
        << (result.valid ? 1 : 0) << ','
        << result.weldPoint.x << ','
        << result.weldPoint.y << ','
        << result.templateScore << ','
        << result.templateAngleDegree << ','
        << elapsedMs << ','
        << errorToSimWeld(result, observation) << ','
        << errorToPoint(result.weldPoint, halcon) << ','
        << (halcon.valid ? 1 : 0) << ','
        << '"' << result.message << '"' << '\n';
}

void updateStats(
    MethodStats *stats,
    const i2r_rlc::vision::SimulatedLaserMatchingResult &result,
    const i2r_rlc::digital_twin::DigitalTwinVisionLaserObservation &observation,
    const BaselinePoint &halcon,
    double elapsedMs)
{
    ++stats->totalCount;
    stats->sumElapsedMs += elapsedMs;
    if (!result.valid)
    {
        return;
    }

    ++stats->validCount;
    stats->sumSimErrorPixel += errorToSimWeld(result, observation);
    if (halcon.valid)
    {
        ++stats->halconComparableCount;
        stats->sumHalconErrorPixel += errorToPoint(result.weldPoint, halcon);
    }
}

void writeHalconRow(
    std::ofstream &report,
    int frameIndex,
    const BaselinePoint &baseline,
    const i2r_rlc::digital_twin::DigitalTwinVisionLaserObservation &observation)
{
    i2r_rlc::vision::SimulatedLaserMatchingResult result;
    result.valid = baseline.valid;
    result.weldPoint = cv::Point2d(baseline.x, baseline.y);
    result.message = baseline.valid
        ? "external HALCON baseline"
        : "HALCON baseline invalid";
    report << frameIndex << ','
        << "halcon_baseline" << ','
        << (result.valid ? 1 : 0) << ','
        << result.weldPoint.x << ','
        << result.weldPoint.y << ','
        << result.templateScore << ','
        << result.templateAngleDegree << ','
        << 0.0 << ','
        << errorToSimWeld(result, observation) << ','
        << 0.0 << ','
        << (baseline.valid ? 1 : 0) << ','
        << '"' << result.message << '"' << '\n';
}

void writeSummary(
    const fs::path &path,
    const std::map<std::string, MethodStats> &stats,
    int processedFrames,
    int halconBaselineCount)
{
    std::ofstream summary(path);
    if (!summary.is_open())
    {
        return;
    }

    summary << "# Shape Matching Experiment Summary\n\n";
    summary << "- Frames processed: " << processedFrames << "\n";
    summary << "- HALCON baseline rows: " << halconBaselineCount << "\n";
    summary << "- shape_based_matching adapter: enabled as a bounded diagnostic "
        "sample; Chamfer rows keep shape_based disabled.\n\n";
    summary << "| method | total | valid | valid_rate | avg_elapsed_ms | "
        "avg_error_to_sim_weld_px | avg_error_to_halcon_px |\n";
    summary << "| --- | ---: | ---: | ---: | ---: | ---: | ---: |\n";
    for (const auto &entry : stats)
    {
        const MethodStats &method = entry.second;
        const double validRate = method.totalCount > 0
            ? static_cast<double>(method.validCount) / method.totalCount
            : 0.0;
        const double avgElapsed = method.totalCount > 0
            ? method.sumElapsedMs / method.totalCount
            : 0.0;
        const double avgSimError = method.validCount > 0
            ? method.sumSimErrorPixel / method.validCount
            : 0.0;
        const double avgHalconError = method.halconComparableCount > 0
            ? method.sumHalconErrorPixel / method.halconComparableCount
            : 0.0;
        summary << "| " << entry.first << " | "
            << method.totalCount << " | "
            << method.validCount << " | "
            << validRate << " | "
            << avgElapsed << " | "
            << avgSimError << " | "
            << avgHalconError << " |\n";
    }
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

} // namespace

int main()
{
    using i2r_rlc::vision::SimulatedLaserMatchingOptions;
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

    fs::path reportPath;
    std::ofstream report = openReportFile(
        projectRoot / "output" / "reports"
            / "shape_matching_experiment_penzui.csv",
        &reportPath);
    if (!report.is_open())
    {
        std::printf("SKIP: failed to create shape matching experiment report\n");
        return 0;
    }

    const std::map<int, BaselinePoint> halconBaseline =
        readHalconBaseline(replayRoot / "halcon_template_matching_baseline.csv");
    // 实验 CSV 固定三类方法列：外部 HALCON baseline、当前 Chamfer、
    // 以及当前 shape_based_matching adapter 的小样本诊断。
    report << "frame,method,valid,x,y,score,angle_deg,elapsed_ms,"
        "error_to_sim_weld_px,error_to_halcon_px,has_halcon_baseline,"
        "failure_reason\n";

    SimulatedLaserMatchingOptions chamferOptions;
    chamferOptions.enableShapeBasedTemplateMatching = false;
    chamferOptions.logSearchMinGrayValue = 35;
    chamferOptions.templateSearchRadiusPixel = 12;
    chamferOptions.templateSearchStepPixel = 2;
    chamferOptions.templateAngleRangeDegree = 4.0;
    chamferOptions.templateAngleStepDegree = 2.0;
    chamferOptions.templateMinScore = 0.0;

    SimulatedLaserMatchingOptions shapeOptions = chamferOptions;
    shapeOptions.enableShapeBasedTemplateMatching = true;
    shapeOptions.enableChamferTemplateFallback = false;
    shapeOptions.shapeBasedTemplateSizePixel = 51;
    shapeOptions.shapeBasedRoiHalfSizePixel = 45;
    shapeOptions.shapeBasedPyramidLevels = {8};
    shapeOptions.templateAngleRangeDegree = 0.0;
    shapeOptions.templateAngleStepDegree = 1.0;
    shapeOptions.shapeBasedMinSimilarity = 40.0;

    std::map<std::string, MethodStats> stats;
    int processedFrames = 0;
    int shapeProcessedFrames = 0;
    const int maxFrames = 20;
    const int maxShapeFrames = 3;
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

        const auto baselineIt = halconBaseline.find(frameData.frameIndex);
        const BaselinePoint halcon = baselineIt == halconBaseline.end()
            ? BaselinePoint{}
            : baselineIt->second;
        if (baselineIt != halconBaseline.end())
        {
            writeHalconRow(report, frameData.frameIndex, baselineIt->second,
                observation);
            i2r_rlc::vision::SimulatedLaserMatchingResult halconResult;
            halconResult.valid = baselineIt->second.valid;
            halconResult.weldPoint =
                cv::Point2d(baselineIt->second.x, baselineIt->second.y);
            updateStats(
                &stats["halcon_baseline"],
                halconResult,
                observation,
                BaselinePoint{},
                0.0);
        }

        double chamferElapsedMs = 0.0;
        const auto chamferResult = runTimed(
            [&gray8, &observation, &chamferOptions]() {
                return CSimulatedLaserTemplateMatcher::match(
                    gray8,
                    observation,
                    chamferOptions);
            },
            &chamferElapsedMs);
        writeResultRow(
            report,
            frameData.frameIndex,
            "chamfer_template",
            chamferResult,
            observation,
            halconBaseline,
            chamferElapsedMs);
        updateStats(
            &stats["chamfer_template"],
            chamferResult,
            observation,
            halcon,
            chamferElapsedMs);

        i2r_rlc::vision::SimulatedLaserMatchingResult shapeResult;
        double shapeElapsedMs = 0.0;
        if (shapeProcessedFrames < maxShapeFrames)
        {
            shapeResult = runTimed(
                [&gray8, &observation, &shapeOptions]() {
                    return CSimulatedLaserTemplateMatcher::match(
                        gray8,
                        observation,
                        shapeOptions);
                },
                &shapeElapsedMs);
            ++shapeProcessedFrames;
        }
        else
        {
            shapeResult.valid = false;
            shapeResult.templateMethod = "shape_based_matching";
            shapeResult.message =
                "shape_based_matching diagnostic frame limit reached";
        }
        writeResultRow(
            report,
            frameData.frameIndex,
            "shape_based_matching",
            shapeResult,
            observation,
            halconBaseline,
            shapeElapsedMs);
        updateStats(
            &stats["shape_based_matching"],
            shapeResult,
            observation,
            halcon,
            shapeElapsedMs);

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
        "PASS: shape matching experiment scaffold, frames=%d halcon=%zu\n",
        processedFrames,
        halconBaseline.size());
    writeSummary(
        reportPath.parent_path() / "shape_matching_experiment_summary.md",
        stats,
        processedFrames,
        static_cast<int>(halconBaseline.size()));
    return 0;
}


