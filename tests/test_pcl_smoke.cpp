/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_pcl_smoke.cpp
 * @brief       pcl smoke 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <filesystem>
#include <iostream>
#include <vector>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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
    cloud->points.reserve(36);

    for (int y = 0; y < 6; ++y) {
        for (int x = 0; x < 6; ++x) {
            cloud->push_back(Point{
                static_cast<float>(x) * 0.05F,
                static_cast<float>(y) * 0.05F,
                0.0F
            });
        }
    }

    return cloud;
}

void verifyPcdIo(const CloudPtr& cloud, int& failures)
{
    const auto pcdPath = std::filesystem::temp_directory_path() / "i2r_rlc_pcl_smoke_test.pcd";

    expect(pcl::io::savePCDFileASCII(pcdPath.string(), *cloud) == 0, "PCD save should succeed", failures);

    Cloud loaded;
    expect(pcl::io::loadPCDFile<Point>(pcdPath.string(), loaded) == 0, "PCD load should succeed", failures);
    expect(loaded.size() == cloud->size(), "loaded PCD point count should match", failures);

    std::error_code error;
    std::filesystem::remove(pcdPath, error);
}

void verifyKdtree(const CloudPtr& cloud, int& failures)
{
    pcl::KdTreeFLANN<Point> kdtree;
    kdtree.setInputCloud(cloud);

    std::vector<int> indices;
    std::vector<float> distances;
    expect(kdtree.nearestKSearch(cloud->front(), 3, indices, distances) == 3, "KD-tree nearest search should find three neighbors", failures);
}

CloudPtr verifyFilters(const CloudPtr& cloud, int& failures)
{
    pcl::PassThrough<Point> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.01F, 0.01F);

    auto passed = CloudPtr(new Cloud);
    pass.filter(*passed);
    expect(passed->size() == cloud->size(), "pass-through filter should keep planar points", failures);

    pcl::VoxelGrid<Point> voxel;
    voxel.setInputCloud(passed);
    voxel.setLeafSize(0.1F, 0.1F, 0.1F);

    auto filtered = CloudPtr(new Cloud);
    voxel.filter(*filtered);
    expect(!filtered->empty(), "voxel grid should produce a non-empty cloud", failures);

    return filtered;
}

} // namespace

int main()
{
    int failures = 0;

    auto cloud = makePlaneCloud();
    expect(cloud->size() == 36, "point cloud should contain generated points", failures);
    expect(!cloud->empty(), "point cloud should not be empty", failures);

    verifyPcdIo(cloud, failures);
    verifyKdtree(cloud, failures);

    auto filtered = verifyFilters(cloud, failures);
    expect(!filtered->empty(), "filtered cloud should not be empty", failures);

    cloud->clear();
    expect(cloud->empty(), "cleared point cloud should be empty", failures);

    return failures == 0 ? 0 : 1;
}
