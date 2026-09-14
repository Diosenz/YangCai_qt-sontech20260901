/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_point_cloud_gap_width.cpp
 * @brief       vision point cloud gap width 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <cstdio>

#include "feature/PointCloudGapWidthDetector.h"

namespace {

bool expectTrue(bool condition)
{
    return condition;
}

bool expectNear(double actual, double expected, double tolerance)
{
    return std::fabs(actual - expected) <= tolerance;
}

i2r_rlc::vision::WeldSpacePoint makePoint(
    double xMm,
    double yMm,
    double zMm)
{
    i2r_rlc::vision::WeldSpacePoint point;
    point.xMm = xMm;
    point.yMm = yMm;
    point.zMm = zMm;
    point.valid = true;
    return point;
}

i2r_rlc::vision::WeldPointCloud makeTwoClusterCloud(void)
{
    i2r_rlc::vision::WeldPointCloud cloud;
    cloud.frameId = "robot_base";
    for (int i = 0; i < 40; ++i) {
        const double s = -30.0 + 0.5 * i;
        cloud.points.push_back(makePoint(s, -5.0, 0.0));
    }
    for (int i = 0; i < 40; ++i) {
        const double s = 10.0 + 0.5 * i;
        cloud.points.push_back(makePoint(s, 5.0, 0.0));
    }
    cloud.valid = true;
    return cloud;
}

} // namespace

int main()
{
    using i2r_rlc::vision::CPointCloudGapWidthDetector;
    using i2r_rlc::vision::PointCloudGapWidthOptions;
    using i2r_rlc::vision::WeldPointCloud;

    PointCloudGapWidthOptions options;
    options.sorMeanK = 6;
    options.sorStddevMulThresh = 2.0;
    options.clusterToleranceMm = 1.2;
    options.minClusterPointCount = 8;
    options.localNeighborCount = 20;
    options.minInputPointCount = 20;

    const auto result = CPointCloudGapWidthDetector::detect(
        makeTwoClusterCloud(),
        makePoint(0.0, 0.0, 0.0),
        options);
    if (!expectTrue(result.valid)
        || !expectNear(result.gapWidthMm, 20.0, 2.0)
        || !expectTrue(result.leftClusterPointCount >= 20)
        || !expectTrue(result.rightClusterPointCount >= 20)
        || !expectTrue(result.leftBoundaryPoint.valid)
        || !expectTrue(result.rightBoundaryPoint.valid)
        || !expectTrue(result.confidence > 0.40))
    {
        std::printf(
            "gap test failed: valid=%d gap=%.3f left=%d right=%d filtered=%d confidence=%.3f message=%s\n",
            result.valid ? 1 : 0,
            result.gapWidthMm,
            result.leftClusterPointCount,
            result.rightClusterPointCount,
            result.filteredPointCount,
            result.confidence,
            result.message.c_str());
        return 1;
    }

    WeldPointCloud emptyCloud;
    const auto emptyResult = CPointCloudGapWidthDetector::detect(
        emptyCloud,
        makePoint(0.0, 0.0, 0.0),
        options);
    if (!expectTrue(!emptyResult.valid)) {
        return 2;
    }

    WeldPointCloud singleCluster;
    singleCluster.valid = true;
    for (int i = 0; i < 25; ++i) {
        singleCluster.points.push_back(makePoint(-12.0 + i, 0.0, 0.0));
    }
    const auto singleClusterResult = CPointCloudGapWidthDetector::detect(
        singleCluster,
        makePoint(0.0, 0.0, 0.0),
        options);
    if (!expectTrue(!singleClusterResult.valid)) {
        return 3;
    }

    auto invalidReference = makePoint(0.0, 0.0, 0.0);
    invalidReference.valid = false;
    const auto invalidReferenceResult = CPointCloudGapWidthDetector::detect(
        makeTwoClusterCloud(),
        invalidReference,
        options);
    if (!expectTrue(!invalidReferenceResult.valid)) {
        return 4;
    }

    std::printf("PASS: vision point cloud gap width tests\n");
    return 0;
}
