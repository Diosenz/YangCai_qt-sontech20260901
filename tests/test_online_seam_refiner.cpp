/**
 * @file test_online_seam_refiner.cpp
 * @brief 使用已知生成坐标验证在线接缝精定位，不把算法拟合作为真值。
 */
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "feature/OnlineSeamRefiner.h"

namespace
{
namespace vision = i2r_rlc::vision;

/** 生成独立给定的条纹中心；台阶不画连接线，避免暗中指定其焊点高度。 */
cv::Mat makeStripe(const cv::Point2d &apex, double jumpPixel,
    bool includeLeft, bool includeRight)
{
    cv::Mat image(220, 300, CV_8UC1);
    for (int y = 0; y < image.rows; ++y)
    {
        for (int x = 0; x < image.cols; ++x)
        {
            const double dx = static_cast<double>(x) - apex.x;
            const bool left = dx < 0.0;
            const bool supported = left ? includeLeft : includeRight;
            const double stripeY = apex.y + (left ? 0.28 : -0.32) * dx
                + (left ? 0.0 : jumpPixel);
            const double distance = static_cast<double>(y) - stripeY;
            const double signal = supported
                ? 210.0 * std::exp(-distance * distance / (2.0 * 1.2 * 1.2))
                : 0.0;
            const int texture = (x * 17 + y * 13) % 7;
            image.at<unsigned char>(y, x) = cv::saturate_cast<unsigned char>(
                15.0 + static_cast<double>(texture) + signal);
        }
    }
    return image;
}

bool expect(bool condition, const std::string &message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

bool expectApex(const cv::Mat &image, const cv::Point2d &coarse,
    const cv::Point2d &truth, const std::string &label)
{
    const vision::OnlineSeamRefinementOptions options;
    const vision::COnlineSeamRefiner refiner;
    const auto result = refiner.refine(image, coarse, options);
    std::cout << label << " valid=" << result.valid
              << " point=" << result.point << " confidence="
              << result.confidence << '\n';
    return expect(result.valid && result.horizontalValid, label + " accepted")
        && expect(std::abs(result.point.x - truth.x) <= 1.5,
            label + " independent X truth")
        && expect(std::abs(result.point.y - truth.y) <= 1.5,
            label + " independent Y truth");
}

bool expectRejected(const cv::Mat &image, const cv::Point2d &coarse,
    const std::string &label)
{
    const vision::OnlineSeamRefinementOptions options;
    const vision::COnlineSeamRefiner refiner;
    const auto result = refiner.refine(image, coarse, options);
    return expect(!result.valid, label + " cannot produce a projection point");
}

/** 间隙中部由用户显式定义；生成两侧高度独立给定。 */
bool checkGapCenter(double jumpPixel)
{
    vision::OnlineSeamRefinementOptions options;
    options.heightPolicy = vision::OnlineSeamHeightPolicy::GapCenter;
    const cv::Point2d truth(150.0, 110.0 + jumpPixel / 2.0);
    const auto result = vision::COnlineSeamRefiner::refine(
        makeStripe({150.0, 110.0}, jumpPixel, true, true),
        {165.0, 110.0}, options);
    return expect(result.valid && result.horizontalValid,
        "explicit gap-center policy accepts supported step")
        && expect(std::abs(result.point.x - truth.x) <= 1.5,
            "gap center independent generated X")
        && expect(std::abs(result.point.y - truth.y) <= 1.5,
            "gap center independent generated midpoint Y");
}

/** 用户随后选定同类帧的间隙中部；不把该位置宣称为独立物理测量。 */
bool checkRecordedStep()
{
    const auto path = std::filesystem::path(I2R_RLC_PROJECT_SOURCE_DIR)
        / "tests" / "fixtures" / "online_seam_refiner"
        / "frame6469_source_crop.png";
    std::ifstream input(path, std::ios::binary);
    const std::vector<unsigned char> bytes(
        (std::istreambuf_iterator<char>(input)),
        std::istreambuf_iterator<char>());
    const cv::Mat image = cv::imdecode(bytes, cv::IMREAD_GRAYSCALE);
    if (!expect(!image.empty(), "recorded fixture loaded"))
    {
        return false;
    }
    const vision::OnlineSeamRefinementOptions options;
    const auto result = vision::COnlineSeamRefiner::refine(image,
        {510.686096 - 380.0, 588.744385 - 510.0}, options);
    const double seamX = result.leftEndpoint.x + 380.0;
    std::cout << "frame6469 horizontalValid=" << result.horizontalValid
              << " valid=" << result.valid << " x=" << seamX << '\n';
    // 宽松 X 范围来自用户截图的定性位置，不把旧拟合 498.5 当作真值。
    bool passed = expect(result.horizontalValid && seamX >= 495.0 && seamX <= 503.0,
        "recorded step X near the user-confirmed joint")
        && expect(!result.valid,
            "recorded step rejected by conservative continuous-only policy");
    vision::OnlineSeamRefinementOptions gapOptions;
    gapOptions.heightPolicy = vision::OnlineSeamHeightPolicy::GapCenter;
    const auto gapResult = vision::COnlineSeamRefiner::refine(image,
        {510.686096 - 380.0, 588.744385 - 510.0}, gapOptions);
    passed = expect(gapResult.valid && gapResult.horizontalValid,
        "recorded joint accepts explicitly authorized gap center") && passed;
    passed = expect(gapResult.point.x + 380.0 >= 495.0
        && gapResult.point.x + 380.0 <= 503.0,
        "recorded gap-center X qualitative support range") && passed;
    passed = expect(gapResult.point.y + 510.0 >= 591.0
        && gapResult.point.y + 510.0 <= 593.0,
        "recorded gap-center Y consistent with authorized geometry") && passed;
    return passed;
}
}

int main()
{
    bool passed = true;
    const cv::Point2d apex(150.0, 110.0);
    const cv::Mat continuous = makeStripe(apex, 0.0, true, true);
    passed = expectApex(continuous, {166.0, 107.0}, apex,
        "right-biased coarse point") && passed;
    passed = expectApex(continuous, {132.0, 113.0}, apex,
        "left-biased coarse point") && passed;
    const cv::Point2d translated(167.0, 121.0);
    passed = expectApex(makeStripe(translated, 0.0, true, true),
        {153.0, 119.0}, translated, "translated physical seam") && passed;

    const vision::OnlineSeamRefinementOptions options;
    const vision::COnlineSeamRefiner refiner;
    const auto step = refiner.refine(makeStripe(apex, -8.0, true, true),
        {164.0, 108.0}, options);
    passed = expect(step.horizontalValid && !step.valid,
        "step has X support but no justified seam height") && passed;
    passed = expect(std::abs(step.leftEndpoint.x - apex.x) <= 2.0,
        "step X follows the known discontinuity") && passed;
    passed = checkGapCenter(-8.0) && passed;
    passed = checkGapCenter(10.0) && passed;
    passed = checkGapCenter(0.0) && passed;
    vision::OnlineSeamRefinementOptions gapOptions;
    gapOptions.heightPolicy = vision::OnlineSeamHeightPolicy::GapCenter;
    const auto excessiveGap = vision::COnlineSeamRefiner::refine(
        makeStripe(apex, -25.0, true, true), {165.0, 110.0}, gapOptions);
    passed = expect(!excessiveGap.valid,
        "explicit gap-center policy rejects excessive height gap") && passed;

    passed = expectRejected(makeStripe(apex, 0.0, true, false), apex,
        "missing right stripe") && passed;
    passed = expectRejected(makeStripe(apex, 0.0, false, true), apex,
        "missing left stripe") && passed;
    passed = expectRejected(cv::Mat(220, 300, CV_8UC1, cv::Scalar(15)), apex,
        "blank image") && passed;
    passed = expectRejected(cv::Mat(220, 300, CV_8UC1, cv::Scalar(255)), apex,
        "saturated reflection without stripe contrast") && passed;
    cv::Mat missingJoint = continuous.clone();
    missingJoint.colRange(144, 157).setTo(cv::Scalar(15));
    passed = expectRejected(missingJoint, {164.0, 110.0},
        "missing local joint cannot use extrapolated intersection") && passed;
    cv::Mat reflection = continuous.clone();
    reflection(cv::Rect(140, 75, 22, 70)).setTo(cv::Scalar(255));
    passed = expectRejected(reflection, {164.0, 110.0},
        "saturated patch obscuring the joint") && passed;
    passed = expect(!vision::COnlineSeamRefiner::refine(missingJoint,
        {164.0, 110.0}, gapOptions).valid,
        "gap-center policy cannot bridge a missing local stripe") && passed;
    passed = expect(!vision::COnlineSeamRefiner::refine(reflection,
        {164.0, 110.0}, gapOptions).valid,
        "gap-center policy rejects obscuring reflection") && passed;
    cv::Mat straight(220, 300, CV_8UC1, cv::Scalar(15));
    straight.rowRange(108, 112).setTo(cv::Scalar(220));
    passed = expectRejected(straight, apex,
        "straight stripe without a supported joint") && passed;
    const cv::Mat competingStripe = makeStripe({150.0, 124.0},
        0.0, true, true);
    cv::Mat equalStrengthLines;
    cv::max(continuous, competingStripe, equalStrengthLines);
    passed = expectRejected(equalStrengthLines, {166.0, 113.0},
        "equally strong complete V lines are ambiguous") && passed;
    const auto ambiguous = vision::COnlineSeamRefiner::refine(
        equalStrengthLines, {166.0, 113.0}, options);
    passed = expect(ambiguous.status
        == vision::OnlineSeamRefinementStatus::AmbiguousStripe
        && ambiguous.ambiguousColumnRatio > options.maxAmbiguousColumnRatio,
        "competing lines report ambiguity rather than a confident point")
        && passed;
    passed = expect(!vision::COnlineSeamRefiner::refine(equalStrengthLines,
        {166.0, 113.0}, gapOptions).valid,
        "gap-center policy cannot resolve equally strong competing lines")
        && passed;
    cv::Mat weakStripe;
    competingStripe.convertTo(weakStripe, CV_8UC1, 0.35);
    cv::Mat weakReflection;
    cv::max(continuous, weakStripe, weakReflection);
    passed = expectApex(weakReflection, {166.0, 113.0}, apex,
        "dominant true stripe with weak parallel reflection") && passed;
    const auto dominant = vision::COnlineSeamRefiner::refine(weakReflection,
        {166.0, 113.0}, gapOptions);
    passed = expect(dominant.valid
        && std::abs(dominant.point.x - apex.x) <= 1.5
        && std::abs(dominant.point.y - apex.y) <= 1.5,
        "gap-center policy preserves a clearly dominant stripe") && passed;
    passed = expectRejected(makeStripe({8.0, 110.0}, 0.0, true, true),
        {12.0, 110.0}, "insufficient support at image border") && passed;
    passed = expectRejected(cv::Mat(), apex, "empty image") && passed;
    passed = expectRejected(continuous,
        {std::numeric_limits<double>::quiet_NaN(), 110.0},
        "non-finite coarse point") && passed;
    passed = checkRecordedStep() && passed;
    return passed ? 0 : 1;
}
