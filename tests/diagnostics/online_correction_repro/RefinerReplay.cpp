/**
 * @file RefinerReplay.cpp
 * @brief 归档原图的生产精定位代码回放；像素范围是图像地标，不是物理真值。
 */
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

#include <opencv2/imgcodecs.hpp>

#include "feature/OnlineSeamRefiner.h"

/** @brief 调用原始生产算法，要求在可见折点邻域产生有效测量；失败返回 1。 */
int main(int argc, char **argv)
{
    if (argc < 4) {
        std::cerr << "usage: RefinerReplay image x y [--crop] [--ambiguity-limit n] [--half-height n]\n";
        return 2;
    }
    cv::Mat image = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        std::cerr << "fixture image unavailable\n";
        return 2;
    }
    cv::Point2d hint(std::strtod(argv[2], nullptr), std::strtod(argv[3], nullptr));
    i2r_rlc::vision::OnlineSeamRefinementOptions options;
    options.heightPolicy = i2r_rlc::vision::OnlineSeamHeightPolicy::GapCenter;
    bool crop = false;
    for (int index = 4; index < argc; ++index) {
        const std::string option(argv[index]);
        if (option == "--crop") {
            crop = true;
        } else if (option == "--ambiguity-limit" && index + 1 < argc) {
            options.maxAmbiguousColumnRatio = std::strtod(argv[++index], nullptr);
        } else if (option == "--half-height" && index + 1 < argc) {
            options.searchHalfHeightPixel = std::stoi(argv[++index]);
        } else {
            return 2;
        }
    }
    if (crop) {
        const cv::Rect bounds(0, 0, image.cols, image.rows);
        const cv::Rect roi = cv::Rect(cvRound(hint.x) - options.searchHalfWidthPixel,
            cvRound(hint.y) - options.searchHalfHeightPixel,
            2 * options.searchHalfWidthPixel + 1,
            2 * options.searchHalfHeightPixel + 1) & bounds;
        image = image(roi).clone();
        hint -= cv::Point2d(roi.x, roi.y);
    }
    const auto result = i2r_rlc::vision::COnlineSeamRefiner::refine(image, hint, options);
    const bool accepted = result.valid && cv::norm(result.point - hint) <= 12.0;
    std::cout << std::fixed << std::setprecision(6)
              << (accepted ? "GREEN" : "RED") << " visible_fold_measurement"
              << " valid=" << result.valid
              << " status=" << static_cast<int>(result.status)
              << " ambiguous_ratio=" << result.ambiguousColumnRatio
              << " limit=" << options.maxAmbiguousColumnRatio
              << " half_height=" << options.searchHalfHeightPixel
              << " input=" << image.cols << "x" << image.rows
              << " point=(" << result.point.x << "," << result.point.y << ")\n";
    return accepted ? 0 : 1;
}
