/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_pcl_registration.cpp
 * @brief       pcl registration 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

namespace {

using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
using CloudPtr = Cloud::Ptr;

void expect(bool condition, const char* message, int& failures)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

CloudPtr makeCloud()
{
    auto cloud = CloudPtr(new Cloud);
    cloud->points.reserve(25);

    for (int y = 0; y < 5; ++y) {
        for (int x = 0; x < 5; ++x) {
            cloud->push_back(Point{
                static_cast<float>(x) * 0.05F,
                static_cast<float>(y) * 0.05F,
                static_cast<float>((x + y) % 3) * 0.01F
            });
        }
    }

    return cloud;
}

} // namespace

int main()
{
    int failures = 0;

    const auto source = makeCloud();

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << 0.02F, 0.0F, 0.0F;

    auto target = CloudPtr(new Cloud);
    pcl::transformPointCloud(*source, *target, transform);

    pcl::IterativeClosestPoint<Point, Point> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaximumIterations(30);
    icp.setMaxCorrespondenceDistance(0.2);

    Cloud aligned;
    icp.align(aligned);

    expect(icp.hasConverged(), "ICP should converge on translated clouds", failures);
    expect(aligned.size() == source->size(), "ICP output point count should match source", failures);

    return failures == 0 ? 0 : 1;
}
