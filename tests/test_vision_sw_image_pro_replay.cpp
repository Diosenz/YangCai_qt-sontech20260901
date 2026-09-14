/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_sw_image_pro_replay.cpp
 * @brief       vision sw image pro replay 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "pipeline/WeldVisionAlgorithmPipeline.h"
#include "preprocess/ImageQualityEvaluator.h"

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

bool isImageFile(const fs::path &path)
{
    const std::string extension = toLower(path.extension().string());
    return extension == ".png"
        || extension == ".jpg"
        || extension == ".jpeg"
        || extension == ".bmp";
}

int parseFrameIndex(const fs::path &path)
{
    const std::string stem = path.stem().string();
    std::string digits;
    for (auto iter = stem.rbegin(); iter != stem.rend(); ++iter)
    {
        if (!std::isdigit(static_cast<unsigned char>(*iter)))
        {
            break;
        }
        digits.push_back(*iter);
    }

    if (digits.empty())
    {
        return -1;
    }

    std::reverse(digits.begin(), digits.end());
    return std::stoi(digits);
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

std::vector<FrameData> collectReplayFrames(const fs::path &caseRoot)
{
    const fs::path imageRoot = fs::exists(caseRoot / "imagecatch")
        ? caseRoot / "imagecatch"
        : caseRoot;

    std::vector<FrameData> frames;
    if (!fs::exists(imageRoot) || !fs::is_directory(imageRoot))
    {
        return frames;
    }

    for (const fs::directory_entry &entry : fs::directory_iterator(imageRoot))
    {
        if (!entry.is_regular_file() || !isImageFile(entry.path()))
        {
            continue;
        }

        FrameData frame;
        frame.frameIndex = parseFrameIndex(entry.path());
        frame.imagePath = entry.path();
        frames.push_back(frame);
    }

    std::sort(frames.begin(), frames.end(),
        [](const FrameData &left, const FrameData &right) {
            if (left.frameIndex != right.frameIndex)
            {
                return left.frameIndex < right.frameIndex;
            }
            return left.imagePath.string() < right.imagePath.string();
        });
    return frames;
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
    if (text.find_first_of(",\"\r\n") == std::string::npos)
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

std::string seamTypeKey(i2r_rlc::vision::WeldSeamType seamType)
{
    using i2r_rlc::vision::WeldSeamType;
    switch (seamType)
    {
    case WeldSeamType::Butt:
        return "butt";
    case WeldSeamType::TJoint:
        return "t_joint";
    case WeldSeamType::Lap:
        return "lap";
    case WeldSeamType::Unknown:
    default:
        return "unknown";
    }
}

void drawPoint(
    cv::Mat &canvas,
    const i2r_rlc::vision::WeldFeaturePoint &point,
    const cv::Scalar &color,
    int radius)
{
    if (!point.valid)
    {
        return;
    }

    cv::circle(
        canvas,
        cv::Point(
            static_cast<int>(std::lround(point.xPixel)),
            static_cast<int>(std::lround(point.yPixel))),
        radius,
        color,
        -1,
        cv::LINE_AA);
}

void drawPolyline(
    cv::Mat &canvas,
    const std::vector<cv::Point2d> &points,
    const cv::Scalar &color)
{
    for (std::size_t index = 1U; index < points.size(); ++index)
    {
        cv::line(
            canvas,
            cv::Point(
                static_cast<int>(std::lround(points[index - 1U].x)),
                static_cast<int>(std::lround(points[index - 1U].y))),
            cv::Point(
                static_cast<int>(std::lround(points[index].x)),
                static_cast<int>(std::lround(points[index].y))),
            color,
            1,
            cv::LINE_AA);
    }
}

void drawText(cv::Mat &canvas, const std::string &text, int row)
{
    cv::putText(
        canvas,
        text,
        cv::Point(8, 20 + row * 18),
        cv::FONT_HERSHEY_SIMPLEX,
        0.45,
        cv::Scalar(255, 255, 255),
        2,
        cv::LINE_AA);
    cv::putText(
        canvas,
        text,
        cv::Point(8, 20 + row * 18),
        cv::FONT_HERSHEY_SIMPLEX,
        0.45,
        cv::Scalar(20, 20, 20),
        1,
        cv::LINE_AA);
}

cv::Mat makeCanvas(const cv::Mat &gray8)
{
    cv::Mat canvas;
    cv::cvtColor(gray8, canvas, cv::COLOR_GRAY2BGR);
    return canvas;
}

void writePipelineOverlay(
    const fs::path &outputPath,
    const cv::Mat &gray8,
    const i2r_rlc::vision::WeldVisionPipelineResult &result,
    const std::string &algorithm)
{
    cv::Mat canvas = makeCanvas(gray8);

    if (algorithm == "endpoint")
    {
        drawPolyline(
            canvas,
            result.startPointResult.activeContourResult.activeCurve.curvePoints,
            cv::Scalar(0, 220, 0));
        drawPoint(canvas, result.startPoint, cv::Scalar(255, 80, 0), 4);
        drawPoint(canvas, result.endPoint, cv::Scalar(0, 80, 255), 4);
        drawPoint(canvas, result.weldPoint, cv::Scalar(0, 255, 255), 5);
        drawPoint(
            canvas,
            result.startPointResult.leftGapPoint,
            cv::Scalar(255, 0, 255),
            3);
        drawPoint(
            canvas,
            result.startPointResult.rightGapPoint,
            cv::Scalar(255, 0, 255),
            3);
    }
    else
    {
        drawPolyline(
            canvas,
            result.stripeResult.centerlinePoints,
            cv::Scalar(0, 220, 0));
        for (const cv::Point2d &point : result.stripeResult.repairedPoints)
        {
            cv::circle(
                canvas,
                cv::Point(
                    static_cast<int>(std::lround(point.x)),
                    static_cast<int>(std::lround(point.y))),
                2,
                cv::Scalar(0, 140, 255),
                -1,
                cv::LINE_AA);
        }
    }

    drawText(canvas, algorithm + (result.valid ? " valid" : " invalid"), 0);
    drawText(canvas, result.message.substr(0U, 80U), 1);

    fs::create_directories(outputPath.parent_path());
    cv::imwrite(outputPath.string(), canvas);
}

void writeQualityOverlay(
    const fs::path &outputPath,
    const cv::Mat &gray8,
    const std::map<i2r_rlc::vision::ImageQualityMetric,
                   i2r_rlc::vision::ImageQualityResult> &qualityResults)
{
    cv::Mat canvas = makeCanvas(gray8);
    int row = 0;
    for (const auto &entry : qualityResults)
    {
        std::ostringstream text;
        text << i2r_rlc::vision::CImageQualityEvaluator::metricToKey(entry.first)
             << '=' << std::fixed << std::setprecision(2)
             << entry.second.value;
        drawText(canvas, text.str(), row);
        ++row;
    }

    fs::create_directories(outputPath.parent_path());
    cv::imwrite(outputPath.string(), canvas);
}

void configureReplayOptions(
    i2r_rlc::vision::WeldVisionPipelineOptions &options)
{
    options.minImageWidthPixel = 8;
    options.minImageHeightPixel = 8;
    options.stripeOptions.thresholdGrayValue = 140;
    options.stripeOptions.logThreshold = -5000.0;
    options.stripeOptions.defaultLaserWidthPixel = 6;
    options.stripeOptions.minTotalPointCount = 8;
    options.stripeOptions.minSegmentPointCount = 4;
    options.stripeOptions.repairMaxGapPixel = 24;
    options.startPointOptions.minConfidence = 0.02;
}

void writePipelineRow(
    std::ofstream &report,
    const std::string &caseName,
    const FrameData &frame,
    const std::string &algorithm,
    int width,
    int height,
    const i2r_rlc::vision::WeldVisionPipelineResult &result)
{
    report << caseName << ','
           << frame.frameIndex << ','
           << algorithm << ','
           << (result.valid ? 1 : 0) << ','
           << width << ','
           << height << ','
           << result.curveOrCenterlinePoints.size() << ','
           << result.stripeResult.segments.size() << ','
           << std::fixed << std::setprecision(3)
           << result.stripeResult.qualityScore << ','
           << result.confidence << ','
           << result.weldPoint.xPixel << ','
           << result.weldPoint.yPixel << ','
           << result.startPoint.xPixel << ','
           << result.startPoint.yPixel << ','
           << result.endPoint.xPixel << ','
           << result.endPoint.yPixel << ','
           << result.processingTimeMs << ','
           << seamTypeKey(result.seamType) << ','
           << escapeCsv(result.message) << '\n';
}

void writeQualityRow(
    std::ofstream &report,
    const std::string &caseName,
    const FrameData &frame,
    int width,
    int height,
    const std::map<i2r_rlc::vision::ImageQualityMetric,
                   i2r_rlc::vision::ImageQualityResult> &qualityResults)
{
    std::ostringstream message;
    double average = 0.0;
    int count = 0;
    for (const auto &entry : qualityResults)
    {
        if (entry.second.valid)
        {
            average += entry.second.value;
            ++count;
        }
        message << i2r_rlc::vision::CImageQualityEvaluator::metricToKey(entry.first)
                << '=' << std::fixed << std::setprecision(2)
                << entry.second.value << ';';
    }
    if (count > 0)
    {
        average /= static_cast<double>(count);
    }

    report << caseName << ','
           << frame.frameIndex << ','
           << "image_quality" << ','
           << (count > 0 ? 1 : 0) << ','
           << width << ','
           << height << ','
           << 0 << ','
           << 0 << ','
           << std::fixed << std::setprecision(3)
           << average << ','
           << 0.0 << ','
           << 0.0 << ','
           << 0.0 << ','
           << 0.0 << ','
           << 0.0 << ','
           << 0.0 << ','
           << 0.0 << ','
           << 0.0 << ','
           << "unknown" << ','
           << escapeCsv(message.str()) << '\n';
}

std::vector<std::pair<int, std::string>> replayTasks(void)
{
    return {
        {0, "sam"},
        {1, "isffa"},
        {2, "steger"},
        {4, "endpoint"}
    };
}

} // namespace

int main()
{
    namespace vision = i2r_rlc::vision;

    const fs::path projectRoot = findProjectRoot();
    const fs::path replayRoot = projectRoot / "data" / "replay";
    const fs::path reportDir =
        fs::current_path() / "Testing" / "Temporary"
        / "sw_image_pro_replay";
    const fs::path overlayRoot = reportDir / "overlays";
    const fs::path reportPath =
        reportDir / "sw_image_pro_replay_report.csv";

    fs::create_directories(reportDir);

    std::ofstream report(reportPath.string(), std::ios::out | std::ios::trunc);
    if (!report)
    {
        std::cerr << "failed to create report: " << reportPath
                  << ", errno=" << errno << ", " << std::strerror(errno)
                  << '\n';
        return 1;
    }

    report << "case,frame,algorithm,valid,width,height,point_count,"
              "segment_count,quality,confidence,x,y,start_x,start_y,end_x,"
              "end_y,elapsed_ms,seam_type,message\n";

    const std::vector<std::string> cases = {
        "penzui",
        "chufengkoufan",
        "zhixian"
    };

    constexpr int kMaxFramesPerCase = 12;
    int processedFrames = 0;

    for (const std::string &caseName : cases)
    {
        const fs::path caseRoot = replayRoot / caseName;
        std::vector<FrameData> frames = collectReplayFrames(caseRoot);
        if (frames.empty())
        {
            std::cout << "SKIP case without images: " << caseName << '\n';
            continue;
        }

        if (static_cast<int>(frames.size()) > kMaxFramesPerCase)
        {
            frames.resize(kMaxFramesPerCase);
        }

        std::map<std::string, vision::WeldVisionPipelineState> states;

        for (const FrameData &frame : frames)
        {
            const cv::Mat gray8 = readGray8(frame.imagePath);
            if (gray8.empty())
            {
                continue;
            }

            vision::WeldVisionFrameInput input;
            input.gray8 = gray8;
            input.frameId = frame.frameIndex;
            input.sourcePath = frame.imagePath.string();

            for (const auto &taskSpec : replayTasks())
            {
                vision::WeldVisionPipelineOptions options;
                configureReplayOptions(options);
                options.task = vision::WeldVisionTask::fromLegacyImageProType(
                    taskSpec.first);

                vision::WeldVisionPipelineResult result =
                    vision::CWeldVisionAlgorithmPipeline::process(
                        input,
                        options,
                        &states[taskSpec.second]);

                writePipelineRow(
                    report,
                    caseName,
                    frame,
                    taskSpec.second,
                    gray8.cols,
                    gray8.rows,
                    result);

                const fs::path overlayPath =
                    overlayRoot / caseName / taskSpec.second
                    / (taskSpec.second + "_"
                       + std::to_string(frame.frameIndex) + ".png");
                writePipelineOverlay(
                    overlayPath,
                    gray8,
                    result,
                    taskSpec.second);
            }

            const std::map<vision::ImageQualityMetric,
                           vision::ImageQualityResult> qualityResults =
                vision::CImageQualityEvaluator::evaluateAll(
                    gray8,
                    vision::ImageQualityOptions());
            writeQualityRow(
                report,
                caseName,
                frame,
                gray8.cols,
                gray8.rows,
                qualityResults);
            writeQualityOverlay(
                overlayRoot / caseName / "image_quality"
                    / ("image_quality_" + std::to_string(frame.frameIndex)
                       + ".png"),
                gray8,
                qualityResults);

            ++processedFrames;
        }
    }

    if (processedFrames == 0)
    {
        std::cout << "SKIP: no replay images found at "
                  << replayRoot.string() << '\n';
        return 0;
    }

    std::printf(
        "PASS: SWv1 ImagePro replay smoke, frames=%d, report=%s\n",
        processedFrames,
        reportPath.string().c_str());
    return 0;
}
