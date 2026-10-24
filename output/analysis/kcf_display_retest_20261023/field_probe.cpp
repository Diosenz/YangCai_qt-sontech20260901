/**
 * @file test_online_image_measurement.cpp
 * @brief 原图测量输出对错误 CAD 提示、缺测和多方向运动的行为契约。
 */
#include <cmath>
#include <cstdio>
#include <string>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "track/LegacyOnlineKcfDetector.h"

namespace v = i2r_rlc::vision;
namespace dt = i2r_rlc::digital_twin;
namespace {
cv::Mat scene(const cv::Point2d &point, double slope)
{
    cv::Mat image(480, 640, CV_8UC1, cv::Scalar(14));
    for (int y = 0; y < image.rows; ++y) {
        for (int x = 0; x < image.cols; ++x) {
            const double dx = x - point.x, dy = y - point.y;
            const double stripe = -slope * std::abs(dx);
            image.at<unsigned char>(y, x) = cv::saturate_cast<unsigned char>(
                14 + 210 * std::exp(-std::pow(dy - stripe, 2) / 3.38));
        }
    }
    return image;
}
dt::DigitalTwinVisionLaserObservation observation(const cv::Point2d &point)
{
    dt::DigitalTwinVisionLaserObservation result;
    result.valid = true;
    result.seamIndex = 0;
    result.hasWeldPointImage_px = true;
    result.weldPointImage_px = {point.x, point.y};
    result.laserLineImagePoints_px = {{point.x - 100, point.y + 45},
        {point.x, point.y}, {point.x + 100, point.y + 45}};
    return result;
}
dt::DigitalTwinVisionLaserObservation observationAtFrame(const cv::Point2d &point,
    qint64 frameId)
{
    auto result = observation(point);
    result.frameId = frameId;
    return result;
}
v::WeldProjectionContext context()
{
    v::WeldProjectionContext result;
    result.valid = true;
    result.calibration.valid = true;
    result.calibration.focalLengthM = 0.01;
    result.calibration.sxM = result.calibration.syM = 0.00001;
    result.calibration.intrinsics.fxPixel = result.calibration.intrinsics.fyPixel = 1000;
    result.calibration.intrinsics.cxPixel = 320;
    result.calibration.intrinsics.cyPixel = 240;
    result.calibration.laserPlaneC = -1;
    return result;
}
}

int main() {
    auto options = v::LegacyOnlineKcfOptions{};
    options.enableImageMeasurementOutput = true;
    options.seamRefinement.heightPolicy = v::OnlineSeamHeightPolicy::GapCenter;
    const std::string base = "H:/YangCai_Code/YangCai_qt-sontech20260901/output/online_correction/20261023_184732_342/vision_images/";
    const auto image = cv::imread(base + "sample_213_session_1_frame_3490_source.png", cv::IMREAD_GRAYSCALE);
    const auto seed = cv::imread(base + "sample_31_session_1_frame_2713_source.png", cv::IMREAD_GRAYSCALE);
    for (const auto &hint : {cv::Point2d(507,580),cv::Point2d(439,677)}) {
        auto g=v::COnlineSeamRefiner::refine(image,hint,options.seamRefinement);
        std::printf("refine %.1f %.1f valid=%d status=%d point=%.3f %.3f ambiguity=%.3f rms=%.3f\n", hint.x,hint.y,g.valid,int(g.status),g.point.x,g.point.y,g.ambiguousColumnRatio,g.fitRmsPixel);
    }
    for (int warm=0;warm<2;++warm) {
        v::CLegacyOnlineKcfDetector detector;
        if(warm)detector.process(seed,observationAtFrame({439,677},1),context(),options);
        options.enableBoundedImageSearch=true;
        for(int frame=0;frame<160;++frame) {
            auto r=detector.process(image,observationAtFrame({439,677},frame+2),context(),options);
            if(frame<4||frame%20==0||r.valid)std::printf("warm=%d frame=%d valid=%d status=%d calls=%d pending=%d completed=%d point=%.3f %.3f\n",warm,frame,r.valid,int(r.diagnostic.rawRefinement.status),r.diagnostic.imageRefinementCallCount,r.diagnostic.imageSearchCyclePending,r.diagnostic.imageSearchCycleCompleted,r.weldPointPixel.x,r.weldPointPixel.y);
            if(r.valid)break;
        }
        options.enableBoundedImageSearch=false;
    }
}
