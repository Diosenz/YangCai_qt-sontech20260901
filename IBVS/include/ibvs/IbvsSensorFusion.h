/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsSensorFusion.h
 * @brief     IBVS 与 LVS 观测融合的公共算法接口。
 * @details   本文件定义 IBVS 模块自有的传感观测值对象，避免公共接口依赖 Vision 类型。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 11 日
 */

#ifndef I2R_RLC_IBVS_IBVS_SENSOR_FUSION_H
#define I2R_RLC_IBVS_IBVS_SENSOR_FUSION_H

#include <optional>
#include <string>

#include <Eigen/Dense>

#include "ibvs/IbvsFeatureGeometry.h"
#include "ibvs/IbvsTypes.h"

namespace i2r_rlc::ibvs {

/**
 * @brief IBVS 红色双特征点观测。
 *
 * 该结构只表达 IBVS 速度链路需要的焊枪图像特征，不持有相机帧、
 * Vision 检测结果或设备生命周期。
 */
struct IbvsTorchObservation
{
    PixelFeaturePair featurePoints;
    double confidence = 1.0;
    int frameIndex = -1;
    double depthM = -1.0;
};

/**
 * @brief LVS 线结构光观测在 IBVS 模块内的轻量表达。
 *
 * startPoint 优先表示焊缝起始点；weldPoint 可作为焊缝中心线或
 * 起始点不可用时的替代先验。Vision 层结果应在适配层转换为该结构。
 */
struct IbvsLvsObservation
{
    PixelPoint startPoint;
    PixelPoint weldPoint;
    bool hasStartPoint = false;
    bool hasWeldPoint = false;
    double confidence = 0.0;
    double centerlineQuality = 0.0;
    int frameIndex = -1;
};

/**
 * @brief IBVS/LVS 融合参数。
 */
struct IbvsFusionConfig
{
    double minTorchConfidence = 0.05;
    double minLvsConfidence = 0.10;
    double minLvsCenterlineQuality = 0.05;
    int maxFrameDelta = 2;
    bool preferLvs = true;
    double lvsChannelSigmaScale = 0.75;
    double fallbackChannelSigmaScale = 1.25;
    Eigen::Vector4d baseChannelSigmasPx =
        Eigen::Vector4d(0.55, 0.38, 0.55, 0.38);
    IbvsFeatureGeometryParameters geometryParameters;
};

/**
 * @brief IBVS/LVS 融合输出。
 *
 * desiredFeature 可直接进入现有 4 维 IBVS 速度求解链路；channelSigmasPx
 * 用于启用 WLS/KF 的通道权重；fallbackReason 记录降级原因，便于离线诊断。
 */
struct IbvsFusionResult
{
    IbvsError error;
    IbvsFeatureVector desiredFeature = IbvsFeatureVector::Zero();
    PixelFeaturePair desiredPoints;
    Eigen::Vector4d channelSigmasPx =
        Eigen::Vector4d(0.55, 0.38, 0.55, 0.38);
    bool usedLvs = false;
    bool usedFallback = false;
    std::string fallbackReason;
};

/**
 * @brief 将红色焊枪特征与可选 LVS 观测融合为 IBVS 期望特征。
 *
 * 本类是纯算法类，不接真实相机线程、RobotComm 或在线闭环；调用方负责把
 * Vision/DigitalTwin 等外部结果转换成本模块自有观测结构。
 */
class IbvsSensorFusion
{
public:
    /**
     * @brief 根据当前红点观测、可选 LVS 观测和 fallback 期望特征生成融合结果。
     */
    static IbvsFusionResult fuseDesiredFeature(
        const IbvsTorchObservation& torchObservation,
        const std::optional<IbvsLvsObservation>& lvsObservation,
        const IbvsFeatureVector& fallbackDesiredFeature,
        const IbvsFusionConfig& config);
};

} // namespace i2r_rlc::ibvs

#endif // I2R_RLC_IBVS_IBVS_SENSOR_FUSION_H
