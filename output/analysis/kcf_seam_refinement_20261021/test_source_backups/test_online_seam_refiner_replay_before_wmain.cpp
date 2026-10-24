/**
 * @file test_online_seam_refiner_replay.cpp
 * @brief 回放稀疏原图并输出精定位接受/拒绝证据，不宣称连续 KCF 回放。
 */
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "feature/OnlineSeamRefiner.h"

namespace
{
namespace fs = std::filesystem;

std::vector<std::string> split(const std::string &line)
{
    std::vector<std::string> result;
    std::istringstream stream(line);
    std::string part;
    while (std::getline(stream, part, '\t'))
    {
        result.push_back(part);
    }
    if (!line.empty() && line.back() == '\t')
    {
        result.emplace_back();
    }
    return result;
}

/** 空字段、非数值和非有限值必须成为无效输入，不能使整批回放异常退出。 */
bool parseCoordinate(const std::string &text, double *value)
{
    std::istringstream stream(text);
    double parsed = 0.0;
    if (!(stream >> parsed) || !std::isfinite(parsed))
    {
        return false;
    }
    stream >> std::ws;
    if (!stream.eof())
    {
        return false;
    }
    *value = parsed;
    return true;
}

/** 原始录像目录不是正式测试必备数据，不存在时返回显式跳过。 */
fs::path locateRecording()
{
    const fs::path parent = fs::path(I2R_RLC_PROJECT_SOURCE_DIR)
        / "output" / "online_correction";
    if (fs::exists(parent))
    {
        for (const auto &entry : fs::directory_iterator(parent))
        {
            if (entry.is_directory()
                && entry.path().filename().wstring().find(
                    L"20261021_201400_950") == 0)
            {
                return entry.path();
            }
        }
    }
    return {};
}

cv::Mat readImage(const fs::path &path)
{
    std::ifstream input(path, std::ios::binary);
    const std::vector<unsigned char> bytes(
        (std::istreambuf_iterator<char>(input)),
        std::istreambuf_iterator<char>());
    return bytes.empty() ? cv::Mat()
        : cv::imdecode(bytes, cv::IMREAD_GRAYSCALE);
}
}

int main(int argc, char **argv)
{
    const fs::path recording = argc > 1
        ? fs::u8path(argv[1]) : locateRecording();
    if (recording.empty() || !fs::exists(recording / "vision_images"))
    {
        std::cout << "SKIP: original sparse recording unavailable\n";
        return 77;
    }
    std::ifstream input(recording / "vision_diagnostics.tsv");
    std::string line;
    if (!std::getline(input, line))
    {
        std::cerr << "No diagnostic header\n";
        return 1;
    }
    const auto headers = split(line);
    std::map<std::string, std::size_t> columns;
    for (std::size_t index = 0; index < headers.size(); ++index)
    {
        columns[headers[index]] = index;
    }
    // UTF-8 BOM affects the first column only; required columns are elsewhere.
    for (const auto &name : {"frame_id", "before_fusion_x_px",
        "before_fusion_y_px"})
    {
        if (columns.count(name) == 0)
        {
            std::cerr << "Missing diagnostic column: " << name << '\n';
            return 1;
        }
    }
    std::map<int, cv::Point2d> coarsePoints;
    int invalidDiagnosticRows = 0;
    int invalidCoordinateRows = 0;
    while (std::getline(input, line))
    {
        const auto values = split(line);
        if (values.size() != headers.size())
        {
            ++invalidDiagnosticRows;
            continue;
        }
        std::istringstream frameInput(values[columns.at("frame_id")]);
        int frame = -1;
        if (!(frameInput >> frame) || frame < 0)
        {
            ++invalidDiagnosticRows;
            continue;
        }
        double x = std::numeric_limits<double>::quiet_NaN();
        double y = std::numeric_limits<double>::quiet_NaN();
        if (!parseCoordinate(values[columns.at("before_fusion_x_px")], &x)
            || !parseCoordinate(values[columns.at("before_fusion_y_px")], &y))
        {
            x = std::numeric_limits<double>::quiet_NaN();
            y = std::numeric_limits<double>::quiet_NaN();
            ++invalidCoordinateRows;
        }
        coarsePoints.emplace(frame, cv::Point2d(x, y));
    }
    std::ofstream file;
    if (argc > 2)
    {
        file.open(fs::u8path(argv[2]));
        if (!file)
        {
            std::cerr << "Cannot open requested evidence output\n";
            return 1;
        }
    }
    std::ostream &output = file.is_open() ? file : std::cout;
    output << "frame,height_policy,coarse_x,coarse_y,horizontal_valid,valid,status,"
        "point_x,point_y,left_x,left_y,right_x,right_y,confidence,"
        "support_ratio,fit_rms,x_uncertainty,height_jump,ambiguous_column_ratio\n";
    int processed = 0;
    int invalidCoarseImages = 0;
    for (const auto &entry : fs::directory_iterator(recording / "vision_images"))
    {
        const std::wstring name = entry.path().filename().wstring();
        const auto frameStart = name.find(L"_frame_");
        const auto suffix = name.find(L"_source.png");
        if (frameStart == std::wstring::npos || suffix == std::wstring::npos)
        {
            continue;
        }
        const int frame = std::stoi(name.substr(frameStart + 7,
            suffix - frameStart - 7));
        const auto coarse = coarsePoints.find(frame);
        const cv::Mat image = readImage(entry.path());
        if (coarse == coarsePoints.end() || image.empty())
        {
            std::cerr << "Missing matching input for frame " << frame << '\n';
            return 1;
        }
        if (!std::isfinite(coarse->second.x) || !std::isfinite(coarse->second.y))
        {
            ++invalidCoarseImages;
        }
        for (const auto policy : {
            i2r_rlc::vision::OnlineSeamHeightPolicy::RequireContinuous,
            i2r_rlc::vision::OnlineSeamHeightPolicy::GapCenter})
        {
            i2r_rlc::vision::OnlineSeamRefinementOptions options;
            options.heightPolicy = policy;
            const auto result = i2r_rlc::vision::COnlineSeamRefiner::refine(
                image, coarse->second, options);
            output << frame << ',' << static_cast<int>(policy) << ','
            << coarse->second.x << ',' << coarse->second.y
            << ',' << result.horizontalValid << ',' << result.valid << ','
            << static_cast<int>(result.status) << ',' << result.point.x << ','
            << result.point.y << ',' << result.leftEndpoint.x << ','
            << result.leftEndpoint.y << ',' << result.rightEndpoint.x << ','
            << result.rightEndpoint.y << ',' << result.confidence << ','
            << result.supportRatio << ',' << result.fitRmsPixel << ','
            << result.xUncertaintyPixel << ',' << result.heightJumpPixel << ','
            << result.ambiguousColumnRatio << '\n';
        }
        ++processed;
    }
    std::cerr << "Sparse original frames processed: " << processed
        << "; invalid diagnostic rows skipped: " << invalidDiagnosticRows
        << "; invalid coordinate rows retained as InvalidInput: "
        << invalidCoordinateRows
        << "; saved images with invalid coarse point: " << invalidCoarseImages
        << "; no independent ground truth assumed\n";
    return processed == 29 ? 0 : 1;
}
