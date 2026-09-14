/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      test_ibvs_start_point_image_chain.cpp
 * @brief     IBVS 裁切工件图像起始点识别测试。
 */

#include <algorithm>
#include <cctype>
#include <chrono>
#include <ctime>
#include <exception>
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

#include "ibvs/IbvsStartPointImageDetector.h"

#ifndef I2R_RLC_PROJECT_SOURCE_DIR
#define I2R_RLC_PROJECT_SOURCE_DIR "."
#endif

namespace {

using namespace i2r_rlc::ibvs;

struct WorkpieceProfile
{
    std::string id;
    std::string shape;
    std::string cropImages;
    std::string curveSample;
    std::string weldSample;
    cv::Vec3d rvec0 = cv::Vec3d(0.0, 0.0, 0.0);
    cv::Vec3d tvec0 = cv::Vec3d(0.0, 0.0, 1.0);
};

std::string trim(const std::string& text)
{
    const auto first = std::find_if_not(
        text.begin(),
        text.end(),
        [](unsigned char ch) { return std::isspace(ch) != 0; });
    const auto last = std::find_if_not(
        text.rbegin(),
        text.rend(),
        [](unsigned char ch) { return std::isspace(ch) != 0; }).base();
    return first >= last ? std::string() : std::string(first, last);
}

std::string lower(std::string text)
{
    for (char& ch : text)
    {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return text;
}

std::string csvEscape(std::string text)
{
    std::replace(text.begin(), text.end(), ',', ';');
    return text;
}

std::vector<double> parseDoubles(const std::string& text)
{
    std::string normalized = text;
    std::replace(normalized.begin(), normalized.end(), ',', ' ');

    std::vector<double> values;
    std::istringstream stream(normalized);
    double value = 0.0;
    while (stream >> value)
    {
        values.push_back(value);
    }
    return values;
}

bool parseVec3(const std::string& text, cv::Vec3d* output)
{
    const std::vector<double> values = parseDoubles(text);
    if (values.size() != 3 || output == nullptr)
    {
        return false;
    }

    *output = cv::Vec3d(values[0], values[1], values[2]);
    return true;
}

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

bool hasImageExtension(const std::filesystem::path& path)
{
    const std::string extension = lower(path.extension().string());
    return extension == ".jpg"
        || extension == ".jpeg"
        || extension == ".png"
        || extension == ".bmp";
}

std::vector<std::filesystem::path> listImages(const std::filesystem::path& imageDir)
{
    std::vector<std::filesystem::path> paths;
    if (!std::filesystem::exists(imageDir))
    {
        return paths;
    }

    for (const auto& entry : std::filesystem::directory_iterator(imageDir))
    {
        if (entry.is_regular_file() && hasImageExtension(entry.path()))
        {
            paths.push_back(entry.path());
        }
    }
    std::sort(paths.begin(), paths.end());
    return paths;
}

std::map<std::string, WorkpieceProfile> readProfiles(
    const std::filesystem::path& path)
{
    std::ifstream file(path);
    std::map<std::string, WorkpieceProfile> profiles;
    WorkpieceProfile* current = nullptr;
    std::string line;
    while (std::getline(file, line))
    {
        line = trim(line);
        if (line.empty() || line[0] == '#')
        {
            continue;
        }
        if (line.rfind("[workpiece ", 0) == 0 && line.back() == ']')
        {
            const std::string id =
                line.substr(std::string("[workpiece ").size(),
                            line.size() - std::string("[workpiece ").size() - 1);
            profiles[id].id = id;
            current = &profiles[id];
            continue;
        }

        const std::size_t equalPos = line.find('=');
        if (current == nullptr || equalPos == std::string::npos)
        {
            continue;
        }
        const std::string key = trim(line.substr(0, equalPos));
        const std::string value = trim(line.substr(equalPos + 1));
        if (key == "shape") { current->shape = value; }
        else if (key == "crop_images") { current->cropImages = value; }
        else if (key == "curve_sample") { current->curveSample = value; }
        else if (key == "weld_sample") { current->weldSample = value; }
        else if (key == "rvec0_rad") { parseVec3(value, &current->rvec0); }
        else if (key == "tvec0_mm") { parseVec3(value, &current->tvec0); }
    }
    return profiles;
}

std::vector<std::vector<cv::Point3f>> readTemplateEdges(
    const std::filesystem::path& path)
{
    std::ifstream file(path);
    std::vector<std::vector<cv::Point3f>> edges;
    std::vector<cv::Point3f> current;
    std::string line;
    while (std::getline(file, line))
    {
        line = trim(line);
        if (line.empty())
        {
            continue;
        }
        if (line[0] == '#')
        {
            if (!current.empty())
            {
                edges.push_back(current);
                current.clear();
            }
            continue;
        }

        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream stream(line);
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        if (stream >> x >> y >> z)
        {
            current.emplace_back(x, y, z);
        }
    }
    if (!current.empty())
    {
        edges.push_back(current);
    }
    return edges;
}

std::size_t countTemplatePoints(
    const std::vector<std::vector<cv::Point3f>>& edges)
{
    std::size_t count = 0;
    for (const auto& edge : edges)
    {
        count += edge.size();
    }
    return count;
}

std::filesystem::path makeArtifactDir()
{
    const std::filesystem::path root =
        std::filesystem::path(I2R_RLC_PROJECT_SOURCE_DIR)
        / "output"
        / "ibvs_offline_validation"
        / "i2r_rlc_ibvs_start_point_image_chain_tests"
        / makeTimestamp();
    std::filesystem::create_directories(root);
    return root;
}

void writeImageIfPresent(const std::filesystem::path& path, const cv::Mat& image)
{
    if (!image.empty())
    {
        cv::imwrite(path.string(), image);
    }
}

cv::Mat toBgrPreview(const cv::Mat& image)
{
    if (image.empty())
    {
        return cv::Mat();
    }
    if (image.channels() == 1)
    {
        cv::Mat bgr;
        cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
        return bgr;
    }
    return image.clone();
}

cv::Mat makeDistanceMapPreview(const cv::Mat& edges)
{
    if (edges.empty())
    {
        return cv::Mat();
    }
    cv::Mat inverted;
    cv::bitwise_not(edges, inverted);
    cv::Mat distanceMap;
    cv::distanceTransform(inverted, distanceMap, cv::DIST_L2, 3);
    cv::Mat normalized;
    cv::normalize(distanceMap, normalized, 0.0, 255.0, cv::NORM_MINMAX);
    normalized.convertTo(normalized, CV_8UC1);
    cv::Mat colored;
    cv::applyColorMap(normalized, colored, cv::COLORMAP_JET);
    return colored;
}

cv::Mat makeTile(const cv::Mat& image, const std::string& label)
{
    const cv::Size tileSize(320, 240);
    cv::Mat tile(tileSize, CV_8UC3, cv::Scalar(24, 24, 24));
    const cv::Mat bgr = toBgrPreview(image);
    if (!bgr.empty())
    {
        const double scale = std::min(
            static_cast<double>(tileSize.width) / bgr.cols,
            static_cast<double>(tileSize.height - 28) / bgr.rows);
        const int width = std::max(1, static_cast<int>(bgr.cols * scale));
        const int height = std::max(1, static_cast<int>(bgr.rows * scale));
        cv::Mat resized;
        cv::resize(bgr, resized, cv::Size(width, height), 0.0, 0.0, cv::INTER_AREA);
        resized.copyTo(tile(cv::Rect(
            (tileSize.width - width) / 2,
            28 + (tileSize.height - 28 - height) / 2,
            width,
            height)));
    }
    cv::putText(tile, label, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX,
                0.5, cv::Scalar(245, 245, 245), 1, cv::LINE_AA);
    return tile;
}

cv::Mat makeProcessOverview(const IbvsStartPointImageResult& result)
{
    const std::vector<std::pair<std::string, cv::Mat>> steps = {
        { "01 edges", result.edges },
        { "02 contours", result.contoursOverlay },
        { "03 curve initial", result.curveInitialTemplateOverlay },
        { "04 curve aligned", result.curveAlignedTemplateOverlay },
        { "05 distance", makeDistanceMapPreview(result.edges) },
        { "06 weld prior", result.weldPriorOverlay },
        { "07 weld points", result.weldSubpixelOverlay },
        { "08 start", result.startPointOverlay }
    };

    cv::Mat overview(480, 1280, CV_8UC3, cv::Scalar(18, 18, 18));
    for (std::size_t i = 0; i < steps.size(); ++i)
    {
        const cv::Mat tile = makeTile(steps[i].second, steps[i].first);
        tile.copyTo(overview(cv::Rect(
            static_cast<int>(i % 4) * 320,
            static_cast<int>(i / 4) * 240,
            320,
            240)));
    }
    return overview;
}

IbvsStartPointImageDetectorConfig makeConfig(const WorkpieceProfile& profile)
{
    IbvsStartPointImageDetectorConfig config;
    config.workpieceId = profile.id;
    config.shape = lower(profile.shape) == "curve"
        ? IbvsStartPointWorkpieceShape::Curve
        : IbvsStartPointWorkpieceShape::Straight;
    config.strategy = config.shape == IbvsStartPointWorkpieceShape::Curve
        ? IbvsStartPointStrategy::CurveFixedArcFraction
        : IbvsStartPointStrategy::Intersection;
    config.rvec0 = profile.rvec0;
    config.tvec0 = profile.tvec0;
    return config;
}

void writeFrameArtifacts(
    const std::filesystem::path& frameDir,
    const IbvsStartPointImageResult& result)
{
    std::filesystem::create_directories(frameDir);
    writeImageIfPresent(frameDir / "01_edges_binary.png", result.edges);
    writeImageIfPresent(frameDir / "02_edges_contours.jpg", result.contoursOverlay);
    writeImageIfPresent(
        frameDir / "03_curve_initial_overlay.jpg",
        result.curveInitialTemplateOverlay);
    writeImageIfPresent(
        frameDir / "04_curve_aligned_overlay.jpg",
        result.curveAlignedTemplateOverlay);
    writeImageIfPresent(frameDir / "06_weld_prior_overlay.jpg", result.weldPriorOverlay);
    writeImageIfPresent(frameDir / "07_weld_subpixel.jpg", result.weldSubpixelOverlay);
    writeImageIfPresent(frameDir / "08_start_point.jpg", result.startPointOverlay);
    writeImageIfPresent(frameDir / "process_overview.jpg", makeProcessOverview(result));
}

void writeSummary(
    const std::filesystem::path& path,
    const std::string& id,
    const std::filesystem::path& imagePath,
    const IbvsStartPointImageResult& result,
    double elapsedMs)
{
    std::ofstream summary(path);
    summary << "workpiece=" << id << '\n'
            << "image=" << imagePath.filename().string() << '\n'
            << "success=" << (result.success ? 1 : 0) << '\n'
            << "templateCost=" << result.templateCostPx << '\n'
            << "inlierRatio=" << result.templateInlierRatio << '\n'
            << "elapsedMs=" << elapsedMs << '\n'
            << "failureReason=" << result.failureReason << '\n';
}

} // namespace

int main()
{
    std::cout.setf(std::ios::unitbuf);
    std::cerr.setf(std::ios::unitbuf);

    try
    {
        const std::filesystem::path dataRoot =
            std::filesystem::path(I2R_RLC_PROJECT_SOURCE_DIR) / "data" / "IBVS";
        const auto profiles = readProfiles(dataRoot / "config" / "workpiece_profiles.txt");
        if (profiles.empty())
        {
            std::cout << "SKIP: no IBVS workpiece profiles found\n";
            return 0;
        }

        const std::filesystem::path artifactDir = makeArtifactDir();
        const std::filesystem::path quickDir = artifactDir / "quick_overview";
        std::filesystem::create_directories(quickDir);

        std::ofstream metrics(artifactDir / "metrics.csv");
        metrics << "workpiece,image,success,edge_ok,alignment_ok,weld_ok,start_ok,"
                << "curve_points,weld_prior_points,weld_points,start_u,start_v,"
                << "dx_mm,dy_mm,dpsi_deg,template_cost,inlier_ratio,elapsed_ms,"
                << "failure_reason\n";

        std::cout << "INFO: artifactDir=" << artifactDir.string() << '\n';
        std::cout << "INFO: quickOverviewDir=" << quickDir.string() << '\n';

        IbvsStartPointImageDetector detector;
        int testedWorkpieces = 0;
        int successfulWorkpieces = 0;

        for (const auto& [id, profile] : profiles)
        {
            const auto images = listImages(dataRoot / profile.cropImages);
            const auto curveEdges = readTemplateEdges(dataRoot / profile.curveSample);
            const auto weldEdges = readTemplateEdges(dataRoot / profile.weldSample);
            const std::filesystem::path workpieceDir = artifactDir / id;
            std::filesystem::create_directories(workpieceDir);

            std::cout << "INFO: workpiece=" << id
                      << ", images=" << images.size()
                      << ", curveEdges=" << curveEdges.size()
                      << ", curvePoints=" << countTemplatePoints(curveEdges)
                      << ", weldEdges=" << weldEdges.size()
                      << ", weldPoints=" << countTemplatePoints(weldEdges) << '\n';

            if (images.empty() || curveEdges.empty() || weldEdges.empty())
            {
                std::cout << "SKIP_WORKPIECE: " << id << '\n';
                continue;
            }

            ++testedWorkpieces;
            const std::filesystem::path imagePath = images.front();
            cv::Mat image = cv::imread(imagePath.string(), cv::IMREAD_COLOR);
            if (image.empty())
            {
                std::cout << "WARN: cannot read " << imagePath.string() << '\n';
                continue;
            }

            std::cout << "INFO: run first image only: "
                      << id << "/" << imagePath.filename().string() << '\n';
            const auto start = std::chrono::steady_clock::now();
            IbvsStartPointImageResult result;
            try
            {
                result = detector.detect(image, curveEdges, weldEdges, makeConfig(profile));
            }
            catch (const cv::Exception& exception)
            {
                result.error = { IbvsStatus::RUNTIME_ERROR, exception.what() };
                result.failureReason = std::string("cv_exception: ") + exception.what();
            }
            catch (const std::exception& exception)
            {
                result.error = { IbvsStatus::RUNTIME_ERROR, exception.what() };
                result.failureReason = std::string("exception: ") + exception.what();
            }
            const auto end = std::chrono::steady_clock::now();
            const double elapsedMs =
                std::chrono::duration<double, std::milli>(end - start).count();

            const std::filesystem::path frameDir =
                workpieceDir / ("0_" + imagePath.stem().string());
            writeFrameArtifacts(frameDir, result);
            const cv::Mat overview = makeProcessOverview(result);
            writeImageIfPresent(quickDir / (id + "_process_overview.jpg"), overview);
            writeSummary(workpieceDir / "summary.txt", id, imagePath, result, elapsedMs);

            metrics << id << ','
                    << imagePath.filename().string() << ','
                    << (result.success ? 1 : 0) << ','
                    << (result.edgeExtractionSucceeded ? 1 : 0) << ','
                    << (result.templateAlignmentSucceeded ? 1 : 0) << ','
                    << (result.weldExtractionSucceeded ? 1 : 0) << ','
                    << (result.startPointSucceeded ? 1 : 0) << ','
                    << result.curveAlignedTemplatePoints.size() << ','
                    << result.weldPriorPoints.size() << ','
                    << result.weldPoints.size() << ','
                    << result.startPoint.u << ','
                    << result.startPoint.v << ','
                    << result.bestDeltaXMm << ','
                    << result.bestDeltaYMm << ','
                    << result.bestDeltaPsiDeg << ','
                    << result.templateCostPx << ','
                    << result.templateInlierRatio << ','
                    << elapsedMs << ','
                    << csvEscape(result.failureReason) << '\n';

            if (result.success)
            {
                ++successfulWorkpieces;
            }
            std::cout << "INFO: done " << id
                      << ", success=" << (result.success ? 1 : 0)
                      << ", cost=" << result.templateCostPx
                      << ", inlier=" << result.templateInlierRatio
                      << ", elapsedMs=" << elapsedMs
                      << ", reason=" << result.failureReason << '\n';
        }

        std::ofstream summary(artifactDir / "summary.txt");
        summary << "testedWorkpieces=" << testedWorkpieces << '\n'
                << "successfulWorkpieces=" << successfulWorkpieces << '\n'
                << "artifactDir=" << artifactDir.string() << '\n'
                << "quickOverviewDir=" << quickDir.string() << '\n';

        std::cout << "INFO: testedWorkpieces=" << testedWorkpieces
                  << ", successfulWorkpieces=" << successfulWorkpieces << '\n';
        std::cout << "INFO: quickOverviewDir=" << quickDir.string() << '\n';

        if (testedWorkpieces == 0)
        {
            std::cout << "SKIP: no complete start-point workpiece datasets found\n";
            return 0;
        }
        if (successfulWorkpieces != testedWorkpieces)
        {
            std::cerr << "FAIL: at least one workpiece has no successful start-point frame\n";
            return 1;
        }
        std::cout << "PASS: IBVS start point image chain tests\n";
        return 0;
    }
    catch (const cv::Exception& exception)
    {
        std::cerr << "FAIL: OpenCV exception: " << exception.what() << '\n';
        return 1;
    }
    catch (const std::exception& exception)
    {
        std::cerr << "FAIL: exception: " << exception.what() << '\n';
        return 1;
    }
}
