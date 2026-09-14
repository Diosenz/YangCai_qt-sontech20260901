/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_pcl_localization_icp_compat.cpp
 * @brief       pcl localization icp compat 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <iostream>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid.h>

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

class XyYawWarp final : public pcl::registration::WarpPointRigid<Point, Point, float>
{
public:
    XyYawWarp()
        : pcl::registration::WarpPointRigid<Point, Point, float>(3)
    {
    }

    void setParam(const Eigen::VectorXf& parameter) override
    {
        const float cosYaw = std::cos(parameter[2]);
        const float sinYaw = std::sin(parameter[2]);

        this->transform_matrix_.setIdentity();
        this->transform_matrix_(0, 0) = cosYaw;
        this->transform_matrix_(0, 1) = -sinYaw;
        this->transform_matrix_(1, 0) = sinYaw;
        this->transform_matrix_(1, 1) = cosYaw;
        this->transform_matrix_(0, 3) = parameter[0];
        this->transform_matrix_(1, 3) = parameter[1];
    }
};

CloudPtr makeCloud()
{
    auto cloud = CloudPtr(new Cloud);
    cloud->points.reserve(36);

    for (int y = 0; y < 6; ++y) {
        for (int x = 0; x < 6; ++x) {
            cloud->push_back(Point{
                static_cast<float>(x) * 0.04F,
                static_cast<float>(y) * 0.04F,
                static_cast<float>((x + y) % 4) * 0.01F
            });
        }
    }

    return cloud;
}

} // namespace

int main()
{
    int failures = 0;

    using TransformationEstimation =
        pcl::registration::TransformationEstimationLM<Point, Point, float>;

    const auto source = makeCloud();
    const auto target = makeCloud();

    pcl::IterativeClosestPointNonLinear<Point, Point> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaximumIterations(2);
    icp.setMaxCorrespondenceDistance(0.2);

    auto estimation = TransformationEstimation::Ptr(new TransformationEstimation);
    auto warp = pcl::registration::WarpPointRigid<Point, Point, float>::Ptr(new XyYawWarp);

    estimation->setWarpFunction(warp);
    icp.setTransformationEstimation(estimation);

    Cloud aligned;
    icp.align(aligned);

    expect(!aligned.empty(), "nonlinear ICP should produce an output cloud", failures);
    expect(aligned.size() == source->size(), "nonlinear ICP output point count should match source", failures);

    return failures == 0 ? 0 : 1;
}
