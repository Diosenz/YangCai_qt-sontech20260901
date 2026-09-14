/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_pcl_surface_keypoints.cpp
 * @brief       pcl surface keypoints 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <iostream>

#include <pcl/keypoints/iss_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>

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
    cloud->points.reserve(64);

    for (int y = 0; y < 8; ++y) {
        for (int x = 0; x < 8; ++x) {
            cloud->push_back(Point{
                static_cast<float>(x) * 0.04F,
                static_cast<float>(y) * 0.04F,
                static_cast<float>((x + y) % 2) * 0.005F
            });
        }
    }

    return cloud;
}

void verifySurface(const CloudPtr& cloud, int& failures)
{
    pcl::MovingLeastSquares<Point, pcl::PointNormal> mls;
    mls.setInputCloud(cloud);
    mls.setSearchMethod(pcl::search::KdTree<Point>::Ptr(new pcl::search::KdTree<Point>));
    mls.setSearchRadius(0.12);
    mls.setComputeNormals(true);

    pcl::PointCloud<pcl::PointNormal> smoothed;
    mls.process(smoothed);

    expect(!smoothed.empty(), "MLS surface processing should produce points", failures);
}

void verifyKeypoints(const CloudPtr& cloud, int& failures)
{
    pcl::ISSKeypoint3D<Point, Point> keypointDetector;
    keypointDetector.setInputCloud(cloud);
    keypointDetector.setSearchMethod(pcl::search::KdTree<Point>::Ptr(new pcl::search::KdTree<Point>));
    keypointDetector.setSalientRadius(0.12);
    keypointDetector.setNonMaxRadius(0.08);
    keypointDetector.setNormalRadius(0.08);
    keypointDetector.setBorderRadius(0.08);
    keypointDetector.setThreshold21(0.975);
    keypointDetector.setThreshold32(0.975);
    keypointDetector.setMinNeighbors(1);

    Cloud keypoints;
    keypointDetector.compute(keypoints);

    expect(keypoints.size() <= cloud->size(), "ISS keypoints should stay within input size", failures);
}

} // namespace

int main()
{
    int failures = 0;
    const auto cloud = makeCloud();

    verifySurface(cloud, failures);
    verifyKeypoints(cloud, failures);

    return failures == 0 ? 0 : 1;
}
