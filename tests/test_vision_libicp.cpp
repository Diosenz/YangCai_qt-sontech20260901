/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_libicp.cpp
 * @brief       vision libicp 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <cstdio>
#include <vector>

#include "geometry/IcpRegistrationMatcher.h"

namespace {

bool expectTrue(bool condition)
{
    return condition;
}

bool expectNear(double actual, double expected, double tolerance)
{
    return std::fabs(actual - expected) <= tolerance;
}

i2r_rlc::vision::WeldSpacePoint makePoint(double xMm, double yMm, double zMm)
{
    i2r_rlc::vision::WeldSpacePoint point;
    point.xMm = xMm;
    point.yMm = yMm;
    point.zMm = zMm;
    point.valid = true;
    return point;
}

std::vector<i2r_rlc::vision::WeldSpacePoint> makeSource3d(void)
{
    return {
        makePoint(-2.0, -1.0, 0.0),
        makePoint(-1.0, 1.5, 0.4),
        makePoint(0.0, 0.0, -0.5),
        makePoint(1.0, -1.5, 0.8),
        makePoint(2.0, 1.0, -0.2),
        makePoint(3.0, 0.5, 1.0),
        makePoint(-3.0, 0.2, -0.7),
        makePoint(0.5, 2.5, 0.3)
    };
}

std::vector<i2r_rlc::vision::WeldSpacePoint> transform3d(
    const std::vector<i2r_rlc::vision::WeldSpacePoint> &source)
{
    const double angle = 0.06;
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    std::vector<i2r_rlc::vision::WeldSpacePoint> target;
    target.reserve(source.size());
    for (const auto &point : source) {
        target.push_back(makePoint(
            c * point.xMm - s * point.yMm + 0.4,
            s * point.xMm + c * point.yMm - 0.25,
            point.zMm + 0.15));
    }
    return target;
}

std::vector<i2r_rlc::vision::WeldSpacePoint> makeSource2d(void)
{
    return {
        makePoint(-3.0, 0.0, 0.0),
        makePoint(-2.0, 1.0, 0.0),
        makePoint(-1.0, -0.5, 0.0),
        makePoint(0.0, 1.5, 0.0),
        makePoint(1.0, -1.0, 0.0),
        makePoint(2.0, 0.5, 0.0),
        makePoint(3.0, -0.2, 0.0)
    };
}

std::vector<i2r_rlc::vision::WeldSpacePoint> transform2d(
    const std::vector<i2r_rlc::vision::WeldSpacePoint> &source)
{
    const double angle = 0.04;
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    std::vector<i2r_rlc::vision::WeldSpacePoint> target;
    target.reserve(source.size());
    for (const auto &point : source) {
        target.push_back(makePoint(
            c * point.xMm - s * point.yMm + 0.25,
            s * point.xMm + c * point.yMm - 0.15,
            0.0));
    }
    return target;
}

i2r_rlc::vision::WeldPointCloud makeCloud(
    const std::vector<i2r_rlc::vision::WeldSpacePoint> &points)
{
    i2r_rlc::vision::WeldPointCloud cloud;
    cloud.points = points;
    cloud.frameId = "robot_base";
    cloud.valid = true;
    return cloud;
}

std::vector<i2r_rlc::vision::WeldSpacePoint> makePlaneSource(void)
{
    std::vector<i2r_rlc::vision::WeldSpacePoint> points;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            points.push_back(makePoint(
                static_cast<double>(col),
                static_cast<double>(row),
                0.2));
        }
    }
    return points;
}

std::vector<i2r_rlc::vision::WeldSpacePoint> makePlaneTarget(void)
{
    std::vector<i2r_rlc::vision::WeldSpacePoint> points;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            points.push_back(makePoint(
                static_cast<double>(col),
                static_cast<double>(row),
                0.0));
        }
    }
    return points;
}

} // namespace

int main()
{
    using i2r_rlc::vision::CIcpRegistrationMatcher;
    using i2r_rlc::vision::IcpRegistrationOptions;
    using i2r_rlc::vision::WeldPointCloud;

    IcpRegistrationOptions options;
    options.maxIterations = 60;
    options.minDelta = 1.0e-8;
    options.inlierDistanceMm = -1.0;

    const auto source3d = makeSource3d();
    const auto target3d = transform3d(source3d);
    const auto pointToPoint =
        CIcpRegistrationMatcher::alignPointToPoint(target3d, source3d, options);
    if (!expectTrue(pointToPoint.valid)
        || !expectTrue(pointToPoint.meanErrorMm < 0.20)
        || !expectTrue(pointToPoint.transform.valid)
        || !expectTrue(pointToPoint.confidence > 0.10))
    {
        std::printf(
            "point-to-point failed: valid=%d mean=%.6f message=%s\n",
            pointToPoint.valid ? 1 : 0,
            pointToPoint.meanErrorMm,
            pointToPoint.message.c_str());
        return 1;
    }

    const auto cloudResult = CIcpRegistrationMatcher::alignPointToPoint(
        makeCloud(target3d),
        makeCloud(source3d),
        options);
    if (!expectTrue(cloudResult.valid) || !expectTrue(cloudResult.meanErrorMm < 0.20)) {
        return 2;
    }

    IcpRegistrationOptions options2d = options;
    options2d.dimension = 2;
    const auto source2d = makeSource2d();
    const auto target2d = transform2d(source2d);
    const auto pointToPoint2d =
        CIcpRegistrationMatcher::alignPointToPoint(target2d, source2d, options2d);
    if (!expectTrue(pointToPoint2d.valid)
        || !expectTrue(pointToPoint2d.meanErrorMm < 0.20)
        || !expectNear(pointToPoint2d.transform.translationMm.z(), 0.0, 1.0e-9))
    {
        return 3;
    }

    const auto pointToPlane = CIcpRegistrationMatcher::alignPointToPlane(
        makePlaneTarget(),
        makePlaneSource(),
        options);
    if (!expectTrue(pointToPlane.valid) || !expectTrue(pointToPlane.meanErrorMm < 0.30)) {
        return 4;
    }

    const auto affine = CIcpRegistrationMatcher::alignAffine(target2d, source2d, options2d);
    const auto affineBdd =
        CIcpRegistrationMatcher::alignAffineBidirectionalDistance(target2d, source2d, options2d);
    const auto bdd =
        CIcpRegistrationMatcher::alignBidirectionalDistance(target2d, source2d, options2d);
    const auto scaled =
        CIcpRegistrationMatcher::alignWithScale(target3d, source3d, options);
    if (!expectTrue(affine.valid)
        || !expectTrue(affineBdd.valid)
        || !expectTrue(bdd.valid)
        || !expectTrue(scaled.valid)
        || !expectTrue(scaled.scale.x() > 0.0)
        || !expectTrue(affine.meanErrorMm < 1.0)
        || !expectTrue(affineBdd.meanErrorMm < 1.0)
        || !expectTrue(bdd.meanErrorMm < 1.0))
    {
        return 5;
    }

    WeldPointCloud invalidCloud;
    invalidCloud.points.push_back(makePoint(0.0, 0.0, 0.0));
    invalidCloud.points.front().valid = false;
    const auto invalidResult =
        CIcpRegistrationMatcher::alignPointToPoint(invalidCloud, makeCloud(source3d), options);
    if (!expectTrue(!invalidResult.valid)) {
        return 6;
    }

    std::printf("PASS: vision libicp tests\n");
    return 0;
}
