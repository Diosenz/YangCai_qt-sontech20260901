/*******************************************************************************
 *       ______   ______   _______         __                  __
 *      /      | /      \ /       \       /  |                /  |
 *      $$$$$$/ /$$$$$$  |$$$$$$$  |      $$ |        ______  $$ |____
 *        $$ |  $$____$$ |$$ |__$$ |      $$ |       /      \ $$      \
 *        $$ |   /    $$/ $$    $$<       $$ |       $$$$$$  |$$$$$$$  |
 *        $$ |  /$$$$$$/  $$$$$$$  |      $$ |       /    $$ |$$ |  $$ |
 *       _$$ |_ $$ |_____ $$ |  $$ |      $$ |_____ /$$$$$$$ |$$ |__$$ |
 *      / $$   |$$       |$$ |  $$ |      $$       |$$    $$ |$$    $$/
 *      $$$$$$/ $$$$$$$$/ $$/   $$/       $$$$$$$$/  $$$$$$$/ $$$$$$$/
 *
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_kcf_penzui_replay.cpp
 * @brief       penzui KCF 离线回放测试
 * @details     使用仿真激光线初始化模板，对真实图像做 Gabor 增强后执行
 *              生产路径使用的标准 KCF 烟测，并输出 CSV 与效果图。
 *
 * @version     1.1
 * @author      LT
 * @date        2026 年 6 月 6 日
 *
 * @par         修改日志
 * <table>
 * <tr><th>Version  <th>Author      <th>Date        <th>Description         </tr>
 * <tr><td>1.0      <td>LT          <td>260606      <td>创建文件            </tr>
 * <tr><td>1.1      <td>LT          <td>260606      <td>初步完成            </tr>
 * </table>
 *
 ******************************************************************************/
#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#endif

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "feature/SimulatedLaserWeldDetector.h"
#include "preprocess/ImageDenoise.h"
#include "track/KcfTracker.h"

namespace {

namespace fs = std::filesystem;

constexpr int kRoiWidthPixel = 250;
constexpr int kRoiHeightPixel = 200;
constexpr int kTemplateLineWidthPixel = 10;
constexpr int kReinitializeIntervalFrame = 50;
constexpr int kGaborWriteBackOffsetYPixel = -5;
constexpr double kReplayRatio = 0.80;

struct FrameData {
    int frameIndex = -1;
    fs::path imagePath;
};

struct TrackerCase {
    std::string methodName;
    i2r_rlc::vision::CKcfTracker tracker;
    int frameSinceInitialize = 0;
    bool initialized = false;

    TrackerCase(
        std::string name,
        i2r_rlc::vision::KcfFeatureMode mode,
        int templateCount)
        : methodName(std::move(name))
        , tracker(true, true, false, false, mode, templateCount)
    {
    }
};

struct MethodStats {
    std::vector<double> distancesToKcf;
    int validRows = 0;
};

std::string toLower(std::string text)
{
    for (char &character : text) {
        character = static_cast<char>(
            std::tolower(static_cast<unsigned char>(character)));
    }
    return text;
}

std::string escapeCsv(std::string text)
{
    std::string escaped;
    escaped.reserve(text.size());
    for (char character : text) {
        if (character == '"') {
            escaped += "\"\"";
        } else if (character == '\n' || character == '\r') {
            escaped += ' ';
        } else {
            escaped += character;
        }
    }
    return escaped;
}

int parseImageFrameIndex(const fs::path &path)
{
    const std::string stem = path.stem().string();
    const std::string prefix = "image_";
    if (stem.rfind(prefix, 0U) != 0U) {
        return -1;
    }

    const std::string numberText = stem.substr(prefix.size());
    if (numberText.empty()) {
        return -1;
    }

    for (char character : numberText) {
        if (!std::isdigit(static_cast<unsigned char>(character))) {
            return -1;
        }
    }
    return std::stoi(numberText);
}

std::size_t parsePositiveSizeArgument(char *text, std::size_t defaultValue)
{
    if (text == nullptr) {
        return defaultValue;
    }

    char *end = nullptr;
    const unsigned long value = std::strtoul(text, &end, 10);
    if (end == text || value == 0UL) {
        return defaultValue;
    }
    return static_cast<std::size_t>(value);
}

std::vector<FrameData> collectImages(const fs::path &imageDir)
{
    std::vector<FrameData> frames;
    if (!fs::exists(imageDir) || !fs::is_directory(imageDir)) {
        return frames;
    }

    for (const fs::directory_entry &entry : fs::directory_iterator(imageDir)) {
        if (!entry.is_regular_file()
            || toLower(entry.path().extension().string()) != ".png") {
            continue;
        }

        const int frameIndex = parseImageFrameIndex(entry.path());
        if (frameIndex < 0) {
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

bool createSingleDirectory(const fs::path &directory)
{
    if (fs::exists(directory)) {
        return fs::is_directory(directory);
    }
    errno = 0;
#ifdef _WIN32
    fs::path nativeDirectory = directory;
    nativeDirectory.make_preferred();
    const int result = _wmkdir(nativeDirectory.wstring().c_str());
#else
    const int result = mkdir(directory.string().c_str(), 0775);
#endif
    const bool created = result == 0 || errno == EEXIST;
    if (!created) {
        std::printf(
            "WARN: create directory failed path=%s errno=%d\n",
            directory.string().c_str(),
            errno);
        std::fflush(stdout);
    }
    return created;
}

bool ensureOutputDirectories(const fs::path &projectRoot, const fs::path &reportDir)
{
    return createSingleDirectory(projectRoot / "output")
        && createSingleDirectory(projectRoot / "output" / "reports")
        && createSingleDirectory(reportDir)
        && createSingleDirectory(
            reportDir / "kcf_penzui_replay_overlays");
}

std::string readTextFile(const fs::path &path)
{
    std::ifstream input(path);
    if (!input.is_open()) {
        return std::string();
    }

    std::ostringstream output;
    output << input.rdbuf();
    return output.str();
}

cv::Mat readGray8(const fs::path &imagePath)
{
    const cv::Mat image = cv::imread(imagePath.string(), cv::IMREAD_UNCHANGED);
    if (image.empty()) {
        return cv::Mat();
    }
    if (image.type() == CV_8UC1) {
        return image;
    }

    cv::Mat gray8;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray8, cv::COLOR_BGR2GRAY);
    } else if (image.channels() == 4) {
        cv::cvtColor(image, gray8, cv::COLOR_BGRA2GRAY);
    } else {
        image.convertTo(gray8, CV_8UC1);
    }
    return gray8;
}

i2r_rlc::vision::MyLaserImagePoint toMyLaserImagePoint(
    const i2r_rlc::digital_twin::DigitalTwinImagePoint2D_px &point)
{
    i2r_rlc::vision::MyLaserImagePoint output;
    output.px = point.x_px;
    output.py = point.y_px;
    return output;
}

std::vector<i2r_rlc::vision::MyLaserImagePoint> toMyLaserImagePoints(
    const std::vector<i2r_rlc::digital_twin::DigitalTwinImagePoint2D_px> &points)
{
    std::vector<i2r_rlc::vision::MyLaserImagePoint> output;
    output.reserve(points.size());
    for (const auto &point : points) {
        output.push_back(toMyLaserImagePoint(point));
    }
    return output;
}

cv::Rect clampRoiToImage(const cv::Rect &roi, const cv::Size &imageSize)
{
    if (imageSize.width <= 0 || imageSize.height <= 0) {
        return cv::Rect();
    }

    const int width = std::min(roi.width, imageSize.width);
    const int height = std::min(roi.height, imageSize.height);
    const int maxX = std::max(0, imageSize.width - width);
    const int maxY = std::max(0, imageSize.height - height);
    const int x = std::max(0, std::min(roi.x, maxX));
    const int y = std::max(0, std::min(roi.y, maxY));
    return cv::Rect(x, y, width, height);
}

cv::Rect roiFromWeldPoint(
    const i2r_rlc::vision::MyLaserImagePoint &weldPoint,
    const cv::Size &imageSize)
{
    const int x = static_cast<int>(std::lround(weldPoint.px))
        - kRoiWidthPixel / 2;
    const int y = static_cast<int>(std::lround(weldPoint.py))
        - kRoiHeightPixel / 2;
    return clampRoiToImage(
        cv::Rect(x, y, kRoiWidthPixel, kRoiHeightPixel),
        imageSize);
}

void applyLegacyKcfRoiBoundary(
    i2r_rlc::vision::CKcfTracker &tracker,
    const cv::Size &imageSize)
{
    if (tracker.m_roi.x <= 0.0F) {
        tracker.m_roi.x = 1.0F;
    }
    if (tracker.m_roi.y <= 0.0F) {
        tracker.m_roi.y = 1.0F;
    }
    if (tracker.m_roi.x + tracker.m_roi.width >= imageSize.width - 1) {
        tracker.m_roi.x = static_cast<float>(imageSize.width - 2)
            - tracker.m_roi.width;
    }
    if (tracker.m_roi.y + tracker.m_roi.height >= imageSize.height - 1) {
        tracker.m_roi.y = static_cast<float>(imageSize.height - 2)
            - tracker.m_roi.height;
    }
}

std::vector<cv::Point> toPolyline(
    const std::vector<i2r_rlc::vision::MyLaserImagePoint> &points)
{
    std::vector<cv::Point> polyline;
    polyline.reserve(points.size());
    for (const auto &point : points) {
        polyline.emplace_back(
            static_cast<int>(std::lround(point.px)),
            static_cast<int>(std::lround(point.py)));
    }
    return polyline;
}

cv::Mat buildSimulatedTemplateImage(
    const cv::Size &imageSize,
    const std::vector<i2r_rlc::vision::MyLaserImagePoint> &laserPoints)
{
    cv::Mat background = cv::Mat::zeros(imageSize, CV_8UC3);
    const std::vector<cv::Point> polyline = toPolyline(laserPoints);
    for (std::size_t index = 1U; index < polyline.size(); ++index) {
        cv::line(
            background,
            polyline[index - 1U],
            polyline[index],
            cv::Scalar(255, 255, 255),
            kTemplateLineWidthPixel,
            cv::LINE_AA);
    }
    return background;
}

cv::Mat makeEnhancedUpdateImage(
    const cv::Mat &gray8,
    const cv::Rect &roi,
    const std::vector<i2r_rlc::vision::MyLaserImagePoint> &laserPoints,
    const i2r_rlc::vision::MyLaserImagePoint &weldPoint,
    bool *gaborSucceeded)
{
    *gaborSucceeded = false;
    cv::Mat updateImage = gray8.clone();
    if (roi.width <= 0 || roi.height <= 0) {
        return updateImage;
    }

    i2r_rlc::vision::VisionPreprocessStatus status;
    cv::Mat roiImage = gray8(roi).clone();
    cv::Mat enhanced = i2r_rlc::vision::CImageDenoise::gaborDenoise(
        roiImage,
        laserPoints,
        weldPoint,
        &status);
    if (!enhanced.empty() && status == i2r_rlc::vision::VisionPreprocessStatus::Succeeded) {
        cv::threshold(enhanced, enhanced, 0.0, 255.0, cv::THRESH_BINARY | cv::THRESH_OTSU);
        cv::Rect targetRoi = roi;
        targetRoi.y += kGaborWriteBackOffsetYPixel;
        cv::Rect imageBounds(0, 0, updateImage.cols, updateImage.rows);
        cv::Rect clippedTarget = targetRoi & imageBounds;
        if (!clippedTarget.empty()) {
            const cv::Rect sourceRoi(
                clippedTarget.x - targetRoi.x,
                clippedTarget.y - targetRoi.y,
                clippedTarget.width,
                clippedTarget.height);
            enhanced(sourceRoi).copyTo(updateImage(clippedTarget));
        }
        *gaborSucceeded = true;
    }
    return updateImage;
}

double centerErrorToWeldPoint(
    const cv::Rect &roi,
    const i2r_rlc::vision::MyLaserImagePoint &weldPoint)
{
    const double centerX = static_cast<double>(roi.x) + roi.width * 0.5;
    const double centerY = static_cast<double>(roi.y) + roi.height * 0.5;
    const double dx = centerX - weldPoint.px;
    const double dy = centerY - weldPoint.py;
    return std::sqrt(dx * dx + dy * dy);
}

double percentile(std::vector<double> values, double ratio)
{
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const double index = ratio * static_cast<double>(values.size() - 1U);
    const std::size_t lower = static_cast<std::size_t>(std::floor(index));
    const std::size_t upper = std::min<std::size_t>(lower + 1U, values.size() - 1U);
    const double fraction = index - static_cast<double>(lower);
    return values[lower] * (1.0 - fraction) + values[upper] * fraction;
}

double maxValue(const std::vector<double> &values)
{
    if (values.empty()) {
        return 0.0;
    }
    return *std::max_element(values.begin(), values.end());
}

void drawOverlay(
    const cv::Mat &gray8,
    const cv::Mat &updateImage,
    const cv::Rect &roi,
    const std::string &methodName,
    int frameIndex,
    double errorToWeldPoint,
    bool reinitialized,
    const fs::path &outputPath)
{
    cv::Mat left;
    cv::Mat right;
    cv::cvtColor(gray8, left, cv::COLOR_GRAY2BGR);
    cv::cvtColor(updateImage, right, cv::COLOR_GRAY2BGR);

    cv::rectangle(left, roi, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    cv::rectangle(right, roi, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    const cv::Point center(roi.x + roi.width / 2, roi.y + roi.height / 2);
    cv::drawMarker(left, center, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 18, 2);
    cv::drawMarker(right, center, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 18, 2);

    cv::Mat canvas(left.rows, left.cols + right.cols, CV_8UC3, cv::Scalar(0, 0, 0));
    left.copyTo(canvas(cv::Rect(0, 0, left.cols, left.rows)));
    right.copyTo(canvas(cv::Rect(left.cols, 0, right.cols, right.rows)));

    const std::string title = methodName + " frame=" + std::to_string(frameIndex)
        + " err=" + std::to_string(errorToWeldPoint)
        + (reinitialized ? " reinit" : "");
    cv::putText(
        canvas,
        title,
        cv::Point(16, 28),
        cv::FONT_HERSHEY_SIMPLEX,
        0.7,
        cv::Scalar(0, 255, 255),
        2,
        cv::LINE_AA);
    cv::putText(
        canvas,
        "left: raw frame  right: gabor+otsu update frame",
        cv::Point(16, 56),
        cv::FONT_HERSHEY_SIMPLEX,
        0.55,
        cv::Scalar(255, 255, 255),
        1,
        cv::LINE_AA);

    cv::imwrite(outputPath.string(), canvas);
}

void writeCsvRow(
    std::ofstream &report,
    int frameIndex,
    const std::string &methodName,
    bool valid,
    const cv::Rect &roi,
    double errorToWeldPoint,
    double distanceToKcf,
    bool reinitialized,
    const std::string &message)
{
    const double centerX = static_cast<double>(roi.x) + roi.width * 0.5;
    const double centerY = static_cast<double>(roi.y) + roi.height * 0.5;
    report << frameIndex << ','
           << methodName << ','
           << (valid ? 1 : 0) << ','
           << roi.x << ','
           << roi.y << ','
           << roi.width << ','
           << roi.height << ','
           << centerX << ','
           << centerY << ','
           << errorToWeldPoint << ','
           << distanceToKcf << ','
           << (reinitialized ? 1 : 0) << ','
           << '"' << escapeCsv(message) << '"' << '\n';
}

fs::path findProjectRoot(void)
{
#ifdef I2R_RLC_PROJECT_SOURCE_DIR
    const fs::path sourceDir = I2R_RLC_PROJECT_SOURCE_DIR;
    if (fs::exists(sourceDir / "CMakeLists.txt")
        && fs::exists(sourceDir / "Vision")
        && fs::exists(sourceDir / "tests")) {
        return sourceDir;
    }
#endif

    fs::path current = fs::current_path();
    while (!current.empty()) {
        if (fs::exists(current / "CMakeLists.txt")
            && fs::exists(current / "Vision")
            && fs::exists(current / "tests")) {
            return current;
        }
        const fs::path parent = current.parent_path();
        if (parent == current) {
            break;
        }
        current = parent;
    }
    return fs::current_path();
}

} // namespace

/**
 * @brief penzui KCF / 2TMKCF / 3TMKCF replay 测试入口。
 */
int main(int argc, char **argv)
{
    using i2r_rlc::vision::CSimulatedLaserReplayParser;
    using i2r_rlc::vision::KcfFeatureMode;

    const fs::path projectRoot = findProjectRoot();
    const fs::path replayRoot = projectRoot / "data" / "replay" / "penzui";
    const fs::path imageDir = replayRoot / "imagecatch";
    const fs::path laserPath = replayRoot / "simulatedLaser.txt";
    const fs::path weldPath = replayRoot / "simulatedWeldpt.txt";
    std::printf(
        "INFO: KCF penzui replay root=%s\n",
        replayRoot.string().c_str());
    std::fflush(stdout);
    if (!fs::exists(imageDir) || !fs::exists(laserPath) || !fs::exists(weldPath)) {
        std::printf("SKIP: penzui replay data not found\n");
        return 0;
    }

    std::printf("INFO: parsing simulated laser and weld point data\n");
    std::fflush(stdout);
    const auto parseResult = CSimulatedLaserReplayParser::parse(
        readTextFile(laserPath),
        readTextFile(weldPath));
    std::printf(
        "INFO: parsed observations=%zu valid=%d\n",
        parseResult.observations.size(),
        parseResult.valid ? 1 : 0);
    std::fflush(stdout);
    std::vector<FrameData> frames = collectImages(imageDir);
    if (!parseResult.valid || frames.empty()) {
        std::printf("SKIP: penzui replay data is empty\n");
        return 0;
    }

    const std::size_t frameLimit = std::max<std::size_t>(
        1U,
        static_cast<std::size_t>(
            std::floor(static_cast<double>(frames.size()) * kReplayRatio)));
    if (frames.size() > frameLimit) {
        frames.resize(frameLimit);
    }
    if (argc > 1) {
        const std::size_t maxFrames = parsePositiveSizeArgument(
            argv[1],
            frames.size());
        if (frames.size() > maxFrames) {
            frames.resize(maxFrames);
        }
    }
    const std::size_t overlayStride = argc > 2
        ? parsePositiveSizeArgument(argv[2], 1U)
        : 9999U;
    std::printf(
        "INFO: selected_frames=%zu overlay_stride=%zu\n",
        frames.size(),
        overlayStride);
    std::fflush(stdout);

    const fs::path reportDir = fs::current_path() / "kcf_penzui_replay_isolated";
    const fs::path overlayRoot = reportDir / "kcf_penzui_replay_overlays";
    std::printf(
        "INFO: creating report directories under=%s\n",
        overlayRoot.string().c_str());
    std::fflush(stdout);
    if (!ensureOutputDirectories(projectRoot, reportDir)) {
        std::fprintf(stderr, "FAIL: failed to create KCF replay output directories\n");
        return 1;
    }
    std::printf("INFO: report directories ready\n");
    std::fflush(stdout);
    const fs::path reportPath = reportDir / "kcf_penzui_replay_report.csv";
    std::printf("INFO: opening report=%s\n", reportPath.string().c_str());
    std::fflush(stdout);
    errno = 0;
    std::ofstream report(reportPath, std::ios::out | std::ios::trunc);
    if (!report.is_open()) {
        std::fprintf(
            stderr,
            "FAIL: failed to create KCF replay report path=%s errno=%d parent_exists=%d\n",
            reportPath.string().c_str(),
            errno,
            fs::exists(reportPath.parent_path()) ? 1 : 0);
        return 1;
    }
    std::printf("INFO: report opened\n");
    std::fflush(stdout);
    report << "frame,method,valid,roi_x,roi_y,roi_w,roi_h,center_x,center_y,"
              "error_to_sim_weld_px,distance_to_kcf_px,reinitialized,message\n";
    report.flush();

    std::vector<TrackerCase> trackers;
    trackers.reserve(3U);
    trackers.emplace_back("kcf", KcfFeatureMode::Standard, 2);
    trackers.emplace_back("2tmkcf", KcfFeatureMode::Mkcf, 2);
    trackers.emplace_back("3tmkcf", KcfFeatureMode::Mkcf, 3);
    std::printf("INFO: trackers created\n");
    std::fflush(stdout);
    for (const TrackerCase &trackerCase : trackers) {
        if (!createSingleDirectory(overlayRoot / trackerCase.methodName)) {
            std::printf(
                "SKIP: failed to create overlay directory for %s\n",
                trackerCase.methodName.c_str());
            return 0;
        }
    }

    int processedRows = 0;
    int skippedFrames = 0;
    std::map<std::string, MethodStats> statsByMethod;
    std::size_t frameOrdinal = 0U;
    for (const FrameData &frameData : frames) {
        ++frameOrdinal;
        if (frameOrdinal == 1U || frameOrdinal % 30U == 0U) {
            std::printf(
                "INFO: processing frame %zu/%zu image_%d\n",
                frameOrdinal,
                frames.size(),
                frameData.frameIndex);
            std::fflush(stdout);
        }

        if (frameData.frameIndex < 0
            || frameData.frameIndex >= static_cast<int>(parseResult.observations.size())) {
            ++skippedFrames;
            continue;
        }

        const auto &observation = parseResult.observations[
            static_cast<std::size_t>(frameData.frameIndex)];
        if (!observation.hasWeldPointImage_px
            || observation.laserLineImagePoints_px.size() < 2U) {
            ++skippedFrames;
            continue;
        }
        const i2r_rlc::vision::MyLaserImagePoint observationWeldPoint =
            toMyLaserImagePoint(observation.weldPointImage_px);
        const std::vector<i2r_rlc::vision::MyLaserImagePoint>
            observationLaserPoints =
                toMyLaserImagePoints(observation.laserLineImagePoints_px);

        const cv::Mat gray8 = readGray8(frameData.imagePath);
        if (gray8.empty()) {
            ++skippedFrames;
            continue;
        }

        const cv::Rect simulatedRoi =
            roiFromWeldPoint(observationWeldPoint, gray8.size());
        if (simulatedRoi.width <= 0 || simulatedRoi.height <= 0) {
            ++skippedFrames;
            continue;
        }
        const cv::Mat simulatedTemplate =
            buildSimulatedTemplateImage(gray8.size(), observationLaserPoints);

        bool hasKcfReference = false;
        cv::Point2d kcfReferenceCenter;
        for (TrackerCase &trackerCase : trackers) {
            const cv::Rect2f previousRoi = trackerCase.tracker.m_roi;
            const bool wasInitialized = trackerCase.initialized;
            const bool shouldInitialize =
                !trackerCase.initialized
                || trackerCase.frameSinceInitialize > kReinitializeIntervalFrame;
            if (shouldInitialize) {
                if (trackerCase.methodName != "kcf") {
                    trackerCase.tracker.setSimulatedTemplate(
                        simulatedTemplate,
                        simulatedRoi);
                }
                trackerCase.tracker.init(simulatedRoi, simulatedTemplate);
                trackerCase.initialized = true;
                trackerCase.frameSinceInitialize = 0;
                if (wasInitialized) {
                    trackerCase.tracker.m_roi.x = previousRoi.x;
                    trackerCase.tracker.m_roi.y = previousRoi.y;
                }
            }

            applyLegacyKcfRoiBoundary(trackerCase.tracker, gray8.size());
            const cv::Rect currentRoi = clampRoiToImage(
                cv::Rect(
                    static_cast<int>(std::lround(trackerCase.tracker.m_roi.x)),
                    static_cast<int>(std::lround(trackerCase.tracker.m_roi.y)),
                    static_cast<int>(std::lround(trackerCase.tracker.m_roi.width)),
                    static_cast<int>(std::lround(trackerCase.tracker.m_roi.height))),
                gray8.size());

            bool gaborSucceeded = false;
            const cv::Mat updateImage = makeEnhancedUpdateImage(
                gray8,
                currentRoi,
                observationLaserPoints,
                observationWeldPoint,
                &gaborSucceeded);

            cv::Rect resultRoi;
            std::string message = gaborSucceeded ? "ok" : "gabor_failed_raw_roi";
            bool valid = false;
            try {
                trackerCase.tracker.m_Kalman_status_x =
                    observationWeldPoint.px - trackerCase.tracker.m_roi.width * 0.5;
                trackerCase.tracker.m_Kalman_status_y =
                    observationWeldPoint.py - trackerCase.tracker.m_roi.height * 0.5;
                resultRoi = trackerCase.tracker.update(updateImage);
                resultRoi = clampRoiToImage(resultRoi, gray8.size());
                valid = resultRoi.width > 0 && resultRoi.height > 0;
            } catch (const std::exception &exception) {
                resultRoi = currentRoi;
                message = std::string("exception: ") + exception.what();
                valid = false;
            } catch (...) {
                resultRoi = currentRoi;
                message = "unknown_exception";
                valid = false;
            }

            const double errorToWeldPoint =
                centerErrorToWeldPoint(resultRoi, observationWeldPoint);
            double distanceToKcf = 0.0;
            if (valid) {
                const cv::Point2d resultCenter(
                    static_cast<double>(resultRoi.x) + resultRoi.width * 0.5,
                    static_cast<double>(resultRoi.y) + resultRoi.height * 0.5);
                if (trackerCase.methodName == "kcf") {
                    hasKcfReference = true;
                    kcfReferenceCenter = resultCenter;
                } else if (hasKcfReference) {
                    const double dx = resultCenter.x - kcfReferenceCenter.x;
                    const double dy = resultCenter.y - kcfReferenceCenter.y;
                    distanceToKcf = std::sqrt(dx * dx + dy * dy);
                    MethodStats &methodStats = statsByMethod[trackerCase.methodName];
                    ++methodStats.validRows;
                    methodStats.distancesToKcf.push_back(distanceToKcf);
                }
            }
            writeCsvRow(
                report,
                frameData.frameIndex,
                trackerCase.methodName,
                valid,
                resultRoi,
                errorToWeldPoint,
                distanceToKcf,
                shouldInitialize,
                message);

            if ((frameOrdinal - 1U) % overlayStride == 0U) {
                const fs::path overlayPath = overlayRoot / trackerCase.methodName
                    / ("frame_" + std::to_string(frameData.frameIndex) + ".png");
                drawOverlay(
                    gray8,
                    updateImage,
                    resultRoi,
                    trackerCase.methodName,
                    frameData.frameIndex,
                    errorToWeldPoint,
                    shouldInitialize,
                    overlayPath);
            }

            ++trackerCase.frameSinceInitialize;
            ++processedRows;
        }
        report.flush();
    }

    if (processedRows == 0) {
        std::printf("SKIP: no valid KCF replay frames, skipped=%d\n", skippedFrames);
        return 0;
    }

    constexpr double kMaxDistanceToKcfP95Pixel = 2.0;
    constexpr double kMaxDistanceToKcfPixel = 5.0;
    for (const char *methodName : {"2tmkcf", "3tmkcf"}) {
        const MethodStats &methodStats = statsByMethod[methodName];
        if (methodStats.distancesToKcf.empty()) {
            std::fprintf(stderr, "FAIL: %s replay produced no rows comparable to KCF\n", methodName);
            return 1;
        }
        const double methodP95 = percentile(methodStats.distancesToKcf, 0.95);
        const double methodMax = maxValue(methodStats.distancesToKcf);
        std::printf(
            "INFO: %s distance_to_kcf_p95=%.3f distance_to_kcf_max=%.3f\n",
            methodName,
            methodP95,
            methodMax);
        if (methodP95 > kMaxDistanceToKcfP95Pixel || methodMax > kMaxDistanceToKcfPixel) {
            std::fprintf(
                stderr,
                "FAIL: %s is not consistent with KCF on penzui replay: "
                "distance_to_kcf_p95=%.3f distance_to_kcf_max=%.3f limits=(%.3f, %.3f)\n",
                methodName,
                methodP95,
                methodMax,
                kMaxDistanceToKcfP95Pixel,
                kMaxDistanceToKcfPixel);
            return 1;
        }
    }

    std::printf(
        "PASS: KCF penzui replay rows=%d skipped_frames=%d report=%s overlays=%s\n",
        processedRows,
        skippedFrames,
        reportPath.string().c_str(),
        overlayRoot.string().c_str());
    return 0;
}
