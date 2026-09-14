/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_pcl_features_segmentation.cpp
 * @brief       pcl features segmentation 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <iostream>

#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>

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

CloudPtr makePlaneCloud()
{
    auto cloud = CloudPtr(new Cloud);
    cloud->points.reserve(49);

    for (int y = 0; y < 7; ++y) {
        for (int x = 0; x < 7; ++x) {
            cloud->push_back(Point{
                static_cast<float>(x) * 0.05F,
                static_cast<float>(y) * 0.05F,
                0.0F
            });
        }
    }

    return cloud;
}

void verifyNormals(const CloudPtr& cloud, int& failures)
{
    pcl::NormalEstimation<Point, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud);
    normalEstimation.setSearchMethod(pcl::search::KdTree<Point>::Ptr(new pcl::search::KdTree<Point>));
    normalEstimation.setKSearch(6);

    pcl::PointCloud<pcl::Normal> normals;
    normalEstimation.compute(normals);

    expect(normals.size() == cloud->size(), "normal estimation should produce one normal per point", failures);
}

void verifySegmentation(const CloudPtr& cloud, int& failures)
{
    pcl::SACSegmentation<Point> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(0.01);
    segmentation.setInputCloud(cloud);

    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    segmentation.segment(inliers, coefficients);

    expect(!inliers.indices.empty(), "RANSAC plane segmentation should find inliers", failures);
    expect(coefficients.values.size() == 4, "plane segmentation should produce four coefficients", failures);
}

} // namespace

int main()
{
    int failures = 0;
    const auto cloud = makePlaneCloud();

    verifyNormals(cloud, failures);
    verifySegmentation(cloud, failures);

    return failures == 0 ? 0 : 1;
}
