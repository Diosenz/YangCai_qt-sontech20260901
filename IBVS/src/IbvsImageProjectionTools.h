/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsImageProjectionTools.h
 * @brief     IBVS 内部 3D 模板投影与像素采样工具。
 * @details   本文件只处理相机矩阵、位姿扰动、3D edge 投影和 2D edge 采样，
 *            不绑定具体焊接应用语义。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 8 日
 */

#ifndef I2R_RLC_IBVS_SRC_IBVS_IMAGE_PROJECTION_TOOLS_H
#define I2R_RLC_IBVS_SRC_IBVS_IMAGE_PROJECTION_TOOLS_H

#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "ibvs/IbvsTypes.h"

namespace i2r_rlc::ibvs {

/**
 * @brief 已展平的 3D 模板点及原始边界分段范围。
 */
struct IbvsProjectedTemplateModel
{
    std::vector<cv::Point3f> points3d;
    std::vector<std::pair<int, int>> edgeRanges;
};

/**
 * @brief IBVS 内部 3D 模板投影与像素转换工具。
 */
class IbvsImageProjectionTools final
{
public:
    /**
     * @brief 根据 IBVS 相机内参生成 OpenCV 相机矩阵。
     */
    static cv::Mat makeCameraMatrix(const IbvsCameraIntrinsics& intrinsics);

    /**
     * @brief 在初始位姿上叠加平移和绕 Z 轴扰动。
     */
    static void computePerturbedPose(
        const cv::Vec3d& rvec0,
        const cv::Vec3d& tvec0,
        double dxMm,
        double dyMm,
        double dpsiDeg,
        cv::Vec3d* rvecOut,
        cv::Vec3d* tvecOut);

    /**
     * @brief 将 3D 模板边投影到图像平面。
     */
    static std::vector<std::vector<cv::Point2f>> projectEdges(
        const std::vector<std::vector<cv::Point3f>>& edges3d,
        const cv::Vec3d& rvec,
        const cv::Vec3d& tvec,
        const cv::Mat& cameraMatrix);

    /**
     * @brief 将分段投影边展平为单个像素点序列。
     */
    static std::vector<cv::Point2f> flattenEdges(
        const std::vector<std::vector<cv::Point2f>>& edges);

    /**
     * @brief 将 OpenCV 像素点序列转换为 IBVS 像素点序列。
     */
    static std::vector<PixelPoint> toPixelPoints(
        const std::vector<cv::Point2f>& points);

    /**
     * @brief 将分段 3D 模板边展平为可复用投影模型。
     */
    static IbvsProjectedTemplateModel makeProjectedTemplateModel(
        const std::vector<std::vector<cv::Point3f>>& edges3d);

    /**
     * @brief 按固定弧长间隔采样已投影模板边。
     */
    static std::vector<cv::Point2f> sampleProjectedEdges(
        const std::vector<std::vector<cv::Point2f>>& projectedEdges,
        double scale,
        double sampleIntervalPx);

    /**
     * @brief 投影并采样分段 3D 模板边。
     */
    static std::vector<cv::Point2f> projectAndSampleEdges(
        const std::vector<std::vector<cv::Point3f>>& edges3d,
        const cv::Vec3d& rvec,
        const cv::Vec3d& tvec,
        const cv::Mat& cameraMatrix,
        double scale,
        double sampleIntervalPx);

    /**
     * @brief 投影并采样已展平的 3D 模板模型。
     */
    static std::vector<cv::Point2f> projectAndSampleEdges(
        const IbvsProjectedTemplateModel& model,
        const cv::Vec3d& rvec,
        const cv::Vec3d& tvec,
        const cv::Mat& cameraMatrix,
        double scale,
        double sampleIntervalPx);
};

/**
 * @brief 将 OpenCV 像素点转换为 IBVS 像素点。
 */
PixelPoint ibvsToPixelPoint(const cv::Point2f& point);

/**
 * @brief 判断 OpenCV 像素点是否落在图像范围内。
 */
bool ibvsIsInsideImage(const cv::Point2f& point, const cv::Size& imageSize);

} // namespace i2r_rlc::ibvs

#endif // I2R_RLC_IBVS_SRC_IBVS_IMAGE_PROJECTION_TOOLS_H
