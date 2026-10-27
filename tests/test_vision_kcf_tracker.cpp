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
 * @file        test_vision_kcf_tracker.cpp
 * @brief       KCF 跟踪器单元测试
 * @details     使用合成图验证 KCF 核心迁移后的 init/update、边界和跳变保护语义。
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
#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <memory>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

#include "track/FilterKalman.h"

#define private public
#include "track/KcfTracker.h"
#undef private
#include "track/KcfFhog.h"

namespace {

cv::Mat makeFrame(const cv::Point &topLeft, bool color = false)
{
    cv::Mat gray = cv::Mat::zeros(140, 180, CV_8UC1);
    cv::rectangle(gray, cv::Rect(topLeft.x, topLeft.y, 24, 20), cv::Scalar(230), cv::FILLED);
    cv::circle(gray, cv::Point(topLeft.x + 8, topLeft.y + 7), 4, cv::Scalar(80), cv::FILLED);
    if (!color)
    {
        return gray;
    }

    cv::Mat bgr;
    cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
}

void require(bool condition, const char *message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

double centerDistance(const cv::Rect &rect, const cv::Point2d &expected)
{
    const cv::Point2d center(
        rect.x + rect.width * 0.5,
        rect.y + rect.height * 0.5);
    const double dx = center.x - expected.x;
    const double dy = center.y - expected.y;
    return std::sqrt(dx * dx + dy * dy);
}

void testGrayTracking()
{
    i2r_rlc::vision::CKcfTracker tracker(false, true, false, false);
    tracker.init(cv::Rect(38, 36, 30, 28), makeFrame(cv::Point(40, 40)));
    const cv::Size initializedTemplateSize = tracker.m_tmplSize;
    const float initializedScale = tracker.m_scale;
    const cv::Rect result = tracker.update(makeFrame(cv::Point(48, 45)));
    require(centerDistance(result, cv::Point2d(60, 55)) < 12.0,
        "KCF gray tracking did not follow translated target");
    require(tracker.m_tmplSize == initializedTemplateSize
            && std::abs(tracker.m_scale - initializedScale) < 1e-6F,
        "KCF update should not reinitialize template geometry");
}

void testColorInput()
{
    i2r_rlc::vision::CKcfTracker tracker(true, true, false, false);
    tracker.init(cv::Rect(38, 36, 30, 28), makeFrame(cv::Point(40, 40), true));
    const cv::Rect result = tracker.update(makeFrame(cv::Point(44, 42), true));
    require(result.width > 0 && result.height > 0,
        "KCF color input produced invalid rectangle");
}

void testBoundaryClamp()
{
    i2r_rlc::vision::CKcfTracker tracker(false, true, false, false);
    tracker.init(cv::Rect(-10, -8, 30, 30), makeFrame(cv::Point(2, 2)));
    const cv::Rect result = tracker.update(makeFrame(cv::Point(4, 4)));
    require(result.x > -result.width && result.y > -result.height,
        "KCF boundary clamp failed");
}

void testLargeJumpRejection()
{
    i2r_rlc::vision::CKcfTracker tracker(false, true, false, false);
    tracker.init(cv::Rect(38, 36, 30, 28), makeFrame(cv::Point(40, 40)));
    const cv::Rect result = tracker.update(makeFrame(cv::Point(130, 100)));
    require(std::abs(result.x - 38) <= 20 && std::abs(result.y - 36) <= 20,
        "KCF large jump rejection failed");
}

void testLegacyKalmanFlow()
{
    i2r_rlc::vision::CKcfTracker tracker(false, true, false, false);
    tracker.init(cv::Rect(38, 36, 30, 28), makeFrame(cv::Point(40, 40)));
    require(tracker.m_filter != nullptr, "KCF legacy Kalman filter should be allocated by initKF");
    require(!tracker.m_Kal_init, "KCF legacy Kalman state should start uninitialized after init");

    const cv::Rect first = tracker.update(makeFrame(cv::Point(42, 41)));
    require(tracker.m_Kal_init, "KCF first update should initialize legacy Kalman state");
    const auto firstDiagnostic = tracker.lastUpdateDiagnostic();
    require(firstDiagnostic.completed && firstDiagnostic.hasResponse
            && firstDiagnostic.kalmanSeeded && !firstDiagnostic.fusionApplied,
        "first frame must identify Kalman seeding without a fabricated weight");
    require(std::abs(tracker.m_filter->m_X.val[0][0] - tracker.m_roi.x) < 1e-6
            && std::abs(tracker.m_filter->m_X.val[1][0] - tracker.m_roi.y) < 1e-6,
        "KCF first update should seed X/Y without external state correction");

    tracker.m_Kalman_status_x = first.x + 1.0;
    tracker.m_Kalman_status_y = first.y + 1.0;
    const cv::Rect second = tracker.update(makeFrame(cv::Point(44, 42)));
    require(second.area() > 0, "KCF Kalman-corrected update produced invalid rectangle");
    const auto diagnostic = tracker.lastUpdateDiagnostic();
    require(diagnostic.completed && diagnostic.fusionApplied && !diagnostic.kalmanSeeded,
        "subsequent frame must report actual fusion");
    require(std::abs(diagnostic.gainX - tracker.m_filter->m_K.val[0][0]) < 1e-12
            && std::abs(diagnostic.observationNoise - tracker.m_filter->m_cR) < 1e-12
            && diagnostic.finalRoi == tracker.m_roi,
        "diagnostics must expose actual gain and emitted ROI without recomputing them");
    require(tracker.m_filter->m_cR >= tracker.m_filter->m_cRMinimum
            && tracker.m_filter->m_cR <= 1e-2,
        "KCF legacy Kalman cR should be updated from peak response");
    require(std::abs(tracker.m_roi.x - tracker.m_filter->m_X.val[0][0]) < 1e-3
            && std::abs(tracker.m_roi.y - tracker.m_filter->m_X.val[1][0]) < 1e-3,
        "KCF external Kalman state should be written back to ROI");
}

void testFusedAndMkcfPaths()
{
    i2r_rlc::vision::CKcfTracker fused(
        true,
        true,
        false,
        false,
        i2r_rlc::vision::KcfFeatureMode::FusedHogGray);
    fused.init(cv::Rect(38, 36, 30, 28), makeFrame(cv::Point(40, 40), true));
    require(fused.m_tmpl.rows == 32 && fused.m_tmpl.cols == 432,
        "KCF fused feature shape should match legacy 31 HOG + 1 gray channels");
    double fusedMin = 0.0;
    double fusedMax = 0.0;
    cv::minMaxLoc(fused.m_tmpl, &fusedMin, &fusedMax);
    require(fusedMin >= -1e-5 && fusedMax <= 1.0 + 1e-5,
        "KCF fused feature should be normalized to legacy range");
    require(fused.update(makeFrame(cv::Point(43, 42), true)).area() > 0,
        "KCF fused feature path failed");

    i2r_rlc::vision::CKcfTracker mkcf(
        true,
        true,
        false,
        false,
        i2r_rlc::vision::KcfFeatureMode::Mkcf,
        2);
    mkcf.setSimulatedTemplate(makeFrame(cv::Point(40, 40), true), cv::Rect(38, 36, 30, 28));
    require(mkcf.hasSimulatedTemplate(), "KCF simulated template setter failed");
    mkcf.init(cv::Rect(38, 36, 30, 28), makeFrame(cv::Point(40, 40), true));
    require(!mkcf.m_histTmplPrimary.empty() && !mkcf.m_histTmplSecondary.empty(),
        "KCF MKCF initialization should create primary and secondary history templates");
    require(!mkcf.m_alphaF.empty(),
        "KCF MKCF initialization should create alpha filter");
    require(mkcf.update(makeFrame(cv::Point(43, 42), true)).area() > 0,
        "KCF MKCF compatibility path failed");
    require(!mkcf.m_kernel12Primary.empty() && !mkcf.m_kernel21Primary.empty(),
        "KCF 2TMKCF should build cross-template kernel matrices");
    mkcf.clearSimulatedTemplate();
    require(!mkcf.hasSimulatedTemplate(), "KCF simulated template clear failed");
    require(mkcf.update(makeFrame(cv::Point(44, 43), true)).area() > 0,
        "KCF MKCF should fall back when simulated template is cleared");

    i2r_rlc::vision::CKcfTracker threeTemplateMkcf(
        true,
        true,
        false,
        false,
        i2r_rlc::vision::KcfFeatureMode::Mkcf,
        3);
    threeTemplateMkcf.setSimulatedTemplate(makeFrame(cv::Point(40, 40), true), cv::Rect(38, 36, 30, 28));
    threeTemplateMkcf.init(cv::Rect(38, 36, 30, 28), makeFrame(cv::Point(40, 40), true));
    require(threeTemplateMkcf.update(makeFrame(cv::Point(43, 42), true)).area() > 0,
        "KCF 3TMKCF path failed");
    require(!threeTemplateMkcf.m_kernel33Primary.empty()
            && !threeTemplateMkcf.m_kernel13Primary.empty()
            && !threeTemplateMkcf.m_kernel23Primary.empty(),
        "KCF 3TMKCF should build the third-template kernel matrices");
}

void testFhogMatchesLegacyShape()
{
    cv::Mat image = cv::Mat::zeros(32, 40, CV_8UC3);
    cv::rectangle(image, cv::Rect(8, 6, 18, 14), cv::Scalar(40, 190, 230), cv::FILLED);
    cv::line(image, cv::Point(4, 28), cv::Point(35, 3), cv::Scalar(220, 60, 80), 2);

    IplImage imageIpl;
    cvInitImageHeader(
        &imageIpl,
        cvSize(image.cols, image.rows),
        IPL_DEPTH_8U,
        image.channels());
    cvSetData(&imageIpl, image.data, static_cast<int>(image.step));
    CvLSVMFeatureMapCaskade *map = nullptr;
    require(getFeatureMaps(&imageIpl, 4, &map) == LATENT_SVM_OK && map != nullptr,
        "FHOG getFeatureMaps failed");
    require(map->sizeX == 10 && map->sizeY == 8 && map->numFeatures == 27,
        "FHOG initial feature map shape does not match legacy formula");

    require(normalizeAndTruncate(map, VAL_OF_TRUNCATE) == LATENT_SVM_OK,
        "FHOG normalizeAndTruncate failed");
    require(map->sizeX == 8 && map->sizeY == 6 && map->numFeatures == 108,
        "FHOG normalized feature map shape does not match legacy formula");

    float maxFeature = 0.0F;
    for (int i = 0; i < map->sizeX * map->sizeY * map->numFeatures; ++i)
    {
        maxFeature = std::max(maxFeature, map->map[i]);
    }
    require(maxFeature <= VAL_OF_TRUNCATE + 1e-5F,
        "FHOG normalization did not apply legacy truncate threshold");

    require(PCAFeatureMaps(map) == LATENT_SVM_OK,
        "FHOG PCAFeatureMaps failed");
    require(map->sizeX == 8 && map->sizeY == 6 && map->numFeatures == 31,
        "FHOG PCA feature map shape does not match legacy formula");
    freeFeatureMapObject(&map);
}

} // namespace

int main()
{
    testFhogMatchesLegacyShape();
    testGrayTracking();
    testColorInput();
    testBoundaryClamp();
    testLargeJumpRejection();
    testLegacyKalmanFlow();
    testFusedAndMkcfPaths();
    std::cout << "PASS: vision KCF tracker tests\n";
    return 0;
}
