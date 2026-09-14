/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      test_ibvs_image_chain.cpp
 * @brief     IBVS YOLO ROI、红色双点和期望特征的离线图像链路测试。
 * @details   测试只读取本机 `C:\Feature_detection` 离线数据，输出诊断图和
 *            CSV 到 `output/ibvs_offline_validation`，不接 UI、相机线程或机器人。
 * @author    I2R Laboratory
 * @version   1.0
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "ibvs/IbvsFeatureGeometry.h"
#include "ibvs/IbvsRedFeatureDetector.h"
#include "ibvs/IbvsYoloDetector.h"

#ifndef I2R_RLC_IBVS_YOLO_MODEL_PATH
#define I2R_RLC_IBVS_YOLO_MODEL_PATH ""
#endif

#ifndef I2R_RLC_PROJECT_SOURCE_DIR
#define I2R_RLC_PROJECT_SOURCE_DIR "."
#endif

namespace {

using namespace i2r_rlc::ibvs;

struct ImageChainFrameMetric
{
    std::string imageName;
    int detectionCount = 0;
    double topScore = 0.0;
    cv::Rect2f topRoi;
    bool roiValid = false;
    bool redAccepted = false;
    std::size_t redCandidateCount = 0;
    PixelFeaturePair featurePoints;
    PixelPoint startPoint;
    PixelFeaturePair desiredPoints;
    double featureDistancePx = 0.0;
    double desiredErrorPx = 0.0;
};

std::string makeTimestamp(void)
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t timeValue = std::chrono::system_clock::to_time_t(now);
    std::tm localTime;
    localtime_s(&localTime, &timeValue);

    std::ostringstream stream;
    stream << std::put_time(&localTime, "%Y%m%d_%H%M%S");
    return stream.str();
}

std::filesystem::path makeArtifactDir(const std::string& testName)
{
    const std::filesystem::path root =
        std::filesystem::path(I2R_RLC_PROJECT_SOURCE_DIR)
        / "output"
        / "ibvs_offline_validation"
        / testName
        / makeTimestamp();
    std::filesystem::create_directories(root);
    return root;
}

bool writeTextFile(
    const std::filesystem::path& path,
    const std::string& content)
{
    std::ofstream file(path);
    if (!file.is_open())
    {
        std::cerr << "FAIL: cannot write artifact: " << path.string() << '\n';
        return false;
    }

    file << content;
    return true;
}

cv::Rect clampToImage(const cv::Rect2f& box, const cv::Size& imageSize)
{
    const cv::Rect2f imageBounds(
        0.0f,
        0.0f,
        static_cast<float>(imageSize.width),
        static_cast<float>(imageSize.height));
    const cv::Rect2f clipped = box & imageBounds;
    return cv::Rect(
        static_cast<int>(std::floor(clipped.x)),
        static_cast<int>(std::floor(clipped.y)),
        static_cast<int>(std::ceil(clipped.width)),
        static_cast<int>(std::ceil(clipped.height)));
}

bool isValidDetection(
    const IbvsYoloDetection& detection,
    const cv::Size& imageSize)
{
    if (!std::isfinite(detection.score)
        || detection.score < 0.0f
        || detection.score > 1.0f)
    {
        return false;
    }

    if (detection.box.width <= 0.0f || detection.box.height <= 0.0f)
    {
        return false;
    }

    return clampToImage(detection.box, imageSize).area() > 0;
}

cv::Mat makeRedMaskForArtifact(const cv::Mat& bgrRoi)
{
    cv::Mat hsv;
    cv::cvtColor(bgrRoi, hsv, cv::COLOR_BGR2HSV);

    cv::Mat lowRedMask;
    cv::Mat highRedMask;
    cv::Mat redMask;
    cv::inRange(hsv, cv::Scalar(0, 30, 10), cv::Scalar(15, 255, 255), lowRedMask);
    cv::inRange(
        hsv,
        cv::Scalar(160, 30, 10),
        cv::Scalar(180, 255, 255),
        highRedMask);
    cv::bitwise_or(lowRedMask, highRedMask, redMask);

    const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(3, 3));
    cv::morphologyEx(redMask, redMask, cv::MORPH_OPEN, kernel);
    return redMask;
}

PixelFeaturePair offsetFeaturePair(
    const PixelFeaturePair& localPair,
    const cv::Rect& roi)
{
    return {
        { localPair.first.u + roi.x, localPair.first.v + roi.y },
        { localPair.second.u + roi.x, localPair.second.v + roi.y }
    };
}

PixelPoint estimateStartPoint(const PixelFeaturePair& featurePoints)
{
    const double centerU =
        0.5 * (featurePoints.first.u + featurePoints.second.u);
    const double centerV =
        0.5 * (featurePoints.first.v + featurePoints.second.v);
    return { centerU, centerV - 30.0 };
}

void drawPoint(
    cv::Mat& image,
    const PixelPoint& point,
    const cv::Scalar& color,
    int radius)
{
    cv::circle(
        image,
        cv::Point(
            static_cast<int>(std::lround(point.u)),
            static_cast<int>(std::lround(point.v))),
        radius,
        color,
        cv::FILLED,
        cv::LINE_AA);
}

void drawFeatureArtifact(
    const cv::Mat& image,
    const cv::Rect& roi,
    const ImageChainFrameMetric& metric,
    const std::filesystem::path& path)
{
    cv::Mat visual = image.clone();
    cv::rectangle(visual, roi, cv::Scalar(0, 255, 255), 2);

    if (metric.redAccepted)
    {
        drawPoint(visual, metric.featurePoints.first, cv::Scalar(0, 0, 255), 6);
        drawPoint(visual, metric.featurePoints.second, cv::Scalar(0, 0, 255), 6);
        drawPoint(visual, metric.startPoint, cv::Scalar(255, 0, 255), 6);
        drawPoint(visual, metric.desiredPoints.first, cv::Scalar(0, 255, 0), 5);
        drawPoint(visual, metric.desiredPoints.second, cv::Scalar(0, 255, 0), 5);

        cv::arrowedLine(
            visual,
            cv::Point(
                static_cast<int>(std::lround(metric.featurePoints.first.u)),
                static_cast<int>(std::lround(metric.featurePoints.first.v))),
            cv::Point(
                static_cast<int>(std::lround(metric.desiredPoints.first.u)),
                static_cast<int>(std::lround(metric.desiredPoints.first.v))),
            cv::Scalar(0, 180, 0),
            2,
            cv::LINE_AA);
        cv::arrowedLine(
            visual,
            cv::Point(
                static_cast<int>(std::lround(metric.featurePoints.second.u)),
                static_cast<int>(std::lround(metric.featurePoints.second.v))),
            cv::Point(
                static_cast<int>(std::lround(metric.desiredPoints.second.u)),
                static_cast<int>(std::lround(metric.desiredPoints.second.v))),
            cv::Scalar(0, 180, 0),
            2,
            cv::LINE_AA);
    }

    cv::imwrite(path.string(), visual);
}

void drawFrameStatusArtifact(
    const cv::Mat& image,
    const ImageChainFrameMetric& metric,
    const std::filesystem::path& path)
{
    cv::Mat visual = image.clone();
    std::ostringstream text;
    text << "detections=" << metric.detectionCount
         << ", roiValid=" << (metric.roiValid ? 1 : 0)
         << ", redAccepted=" << (metric.redAccepted ? 1 : 0);
    cv::putText(
        visual,
        text.str(),
        cv::Point(30, 40),
        cv::FONT_HERSHEY_SIMPLEX,
        0.9,
        cv::Scalar(0, 0, 255),
        2,
        cv::LINE_AA);
    cv::imwrite(path.string(), visual);
}

std::vector<std::filesystem::path> existingImagePaths(void)
{
    const std::vector<std::filesystem::path> paths{
        "C:/Feature_detection/FeatureDetection/FeatureDetection/Desired_image/Images_0100.jpg",
        "C:/Feature_detection/FeatureDetection/FeatureDetection/Images/Images_0100.jpg",
        "C:/Feature_detection/FeatureDetection/FeatureDetection/Orthogonal/Images/Images_00.jpg",
        "C:/Feature_detection/FeatureDetection/FeatureDetection/Orthogonal/Images/Images_01.jpg",
        "C:/Feature_detection/FeatureDetection/FeatureDetection/Orthogonal/Images/Images_02.jpg"
    };

    std::vector<std::filesystem::path> existing;
    for (const std::filesystem::path& path : paths)
    {
        if (std::filesystem::exists(path))
        {
            existing.push_back(path);
        }
    }
    return existing;
}

bool writeMetricsCsv(
    const std::filesystem::path& path,
    const std::vector<ImageChainFrameMetric>& metrics)
{
    std::ofstream file(path);
    if (!file.is_open())
    {
        std::cerr << "FAIL: cannot write metrics csv: " << path.string() << '\n';
        return false;
    }

    file << "image,detection_count,top_score,roi_x,roi_y,roi_w,roi_h,"
            "roi_valid,red_accepted,red_candidates,feature_u1,feature_v1,"
            "feature_u2,feature_v2,start_u,start_v,desired_u1,desired_v1,"
            "desired_u2,desired_v2,feature_distance_px,desired_error_px\n";
    for (const ImageChainFrameMetric& metric : metrics)
    {
        file << metric.imageName << ','
             << metric.detectionCount << ','
             << metric.topScore << ','
             << metric.topRoi.x << ','
             << metric.topRoi.y << ','
             << metric.topRoi.width << ','
             << metric.topRoi.height << ','
             << (metric.roiValid ? 1 : 0) << ','
             << (metric.redAccepted ? 1 : 0) << ','
             << metric.redCandidateCount << ','
             << metric.featurePoints.first.u << ','
             << metric.featurePoints.first.v << ','
             << metric.featurePoints.second.u << ','
             << metric.featurePoints.second.v << ','
             << metric.startPoint.u << ','
             << metric.startPoint.v << ','
             << metric.desiredPoints.first.u << ','
             << metric.desiredPoints.first.v << ','
             << metric.desiredPoints.second.u << ','
             << metric.desiredPoints.second.v << ','
             << metric.featureDistancePx << ','
             << metric.desiredErrorPx << '\n';
    }
    return true;
}

} // namespace

/**
 * @brief 真实离线图像上的 YOLO ROI、红点和期望特征桥接测试入口。
 */
int main(void)
{
    const std::filesystem::path modelPath(I2R_RLC_IBVS_YOLO_MODEL_PATH);
    if (modelPath.empty() || !std::filesystem::exists(modelPath))
    {
        std::cout << "SKIP: IBVS YOLO model file does not exist: "
                  << modelPath.string() << '\n';
        return 0;
    }

    const std::vector<std::filesystem::path> imagePaths = existingImagePaths();
    if (imagePaths.empty())
    {
        std::cout << "SKIP: no IBVS real image chain inputs found\n";
        return 0;
    }

    const std::filesystem::path artifactDir =
        makeArtifactDir("i2r_rlc_ibvs_image_chain_tests");
    std::vector<ImageChainFrameMetric> metrics;
    bool hasValidRoi = false;
    bool hasAcceptedRedPair = false;

    try
    {
        IbvsYoloDetectorConfig yoloConfig;
        yoloConfig.imageSize = 640;
        yoloConfig.confidenceThreshold = 0.3f;
        yoloConfig.iouThreshold = 0.5f;
        yoloConfig.maxDetections = 5;

        IbvsRedFeatureDetectorConfig redConfig;
        redConfig.minPairDistancePx = 15.0;
        redConfig.maxPairDistancePx = 80.0;

        const IbvsYoloDetector yoloDetector(modelPath.string(), yoloConfig);
        const IbvsRedFeatureDetector redDetector(redConfig);

        for (std::size_t i = 0; i < imagePaths.size(); ++i)
        {
            const std::filesystem::path& imagePath = imagePaths[i];
            const cv::Mat image = cv::imread(imagePath.string(), cv::IMREAD_COLOR);
            if (image.empty())
            {
                std::cerr << "FAIL: cannot read image: "
                          << imagePath.string() << '\n';
                return 1;
            }

            ImageChainFrameMetric metric;
            metric.imageName = imagePath.filename().string();
            const auto detections = yoloDetector.detect(image);
            metric.detectionCount = static_cast<int>(detections.size());
            if (!detections.empty())
            {
                const IbvsYoloDetection& top = detections.front();
                metric.topScore = top.score;
                metric.topRoi = top.box;
                metric.roiValid = isValidDetection(top, image.size());
            }

            if (metric.roiValid)
            {
                hasValidRoi = true;
                const cv::Rect roi = clampToImage(metric.topRoi, image.size());
                const cv::Mat roiImage = image(roi).clone();
                const auto redResult = redDetector.detect(roiImage);
                metric.redAccepted = redResult.accepted && redResult.error.isOk();
                metric.redCandidateCount = redResult.candidates.size();

                cv::imwrite(
                    (artifactDir / ("roi_" + std::to_string(i) + ".jpg")).string(),
                    roiImage);
                cv::imwrite(
                    (artifactDir / ("red_mask_" + std::to_string(i) + ".png")).string(),
                    makeRedMaskForArtifact(roiImage));

                if (metric.redAccepted)
                {
                    hasAcceptedRedPair = true;
                    metric.featurePoints =
                        offsetFeaturePair(redResult.featurePoints, roi);
                    metric.featureDistancePx = metric.featurePoints.distancePx();
                    metric.startPoint = estimateStartPoint(metric.featurePoints);

                    const auto geometry = computeDesiredFeature(
                        metric.featurePoints,
                        metric.startPoint,
                        -1.0,
                        IbvsFeatureGeometryParameters());
                    if (!geometry.error.isOk())
                    {
                        std::cerr << "FAIL: desired feature geometry failed: "
                                  << geometry.error.message << '\n';
                        return 1;
                    }

                    metric.desiredPoints = geometry.desiredPoints;
                    metric.desiredErrorPx =
                        (metric.featurePoints.toVector()
                         - geometry.desiredFeature).norm();
                }

                drawFeatureArtifact(
                    image,
                    roi,
                    metric,
                    artifactDir / ("annotated_" + std::to_string(i) + ".jpg"));
            }
            else
            {
                drawFrameStatusArtifact(
                    image,
                    metric,
                    artifactDir / ("annotated_" + std::to_string(i) + ".jpg"));
            }

            metrics.push_back(metric);
        }
    }
    catch (const std::exception& exception)
    {
        std::cerr << "FAIL: IBVS image chain exception: "
                  << exception.what() << '\n';
        return 1;
    }

    if (!writeMetricsCsv(artifactDir / "metrics.csv", metrics))
    {
        return 1;
    }

    std::ostringstream summary;
    summary << "images=" << metrics.size() << '\n'
            << "hasValidRoi=" << (hasValidRoi ? 1 : 0) << '\n'
            << "hasAcceptedRedPair=" << (hasAcceptedRedPair ? 1 : 0) << '\n'
            << "artifactDir=" << artifactDir.string() << '\n';
    if (!writeTextFile(artifactDir / "summary.txt", summary.str()))
    {
        return 1;
    }

    if (!hasValidRoi)
    {
        std::cerr << "FAIL: no valid YOLO ROI detected on real images\n";
        return 1;
    }
    if (!hasAcceptedRedPair)
    {
        std::cerr << "FAIL: no accepted red feature pair inside YOLO ROI\n";
        return 1;
    }

    std::cout << "INFO: image chain artifacts=" << artifactDir.string() << '\n';
    std::cout << "INFO: image chain images=" << metrics.size()
              << ", hasValidRoi=" << (hasValidRoi ? 1 : 0)
              << ", hasAcceptedRedPair=" << (hasAcceptedRedPair ? 1 : 0)
              << '\n';
    std::cout << "PASS: IBVS image chain tests\n";
    return 0;
}
