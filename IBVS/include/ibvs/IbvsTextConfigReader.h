/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsTextConfigReader.h
 * @brief     IBVS 文本配置读取入口。
 * @details   本文件定义 IBVS 运行参数和工件 profile 的 txt 读取接口。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 15 日
 */

#ifndef I2R_RLC_IBVS_IBVS_TEXT_CONFIG_READER_H
#define I2R_RLC_IBVS_IBVS_TEXT_CONFIG_READER_H

#include <string>
#include <vector>

#include "ibvs/IbvsFeatureGeometry.h"
#include "ibvs/IbvsSensorFusion.h"
#include "ibvs/IbvsUnifiedVelocitySolver.h"

namespace i2r_rlc::ibvs {

/**
 * @brief IBVS 运行时 txt 配置聚合结果。
 *
 * 该结构只保存算法值对象，不持有文件路径或设备状态。读取失败时调用方
 * 可以继续使用各结构体自身的默认值作为保守 fallback。
 */
struct IbvsRuntimeTextConfig
{
    IbvsFeatureGeometryParameters featureGeometry;
    IbvsUnifiedVelocityConfig unifiedVelocity;
    IbvsLvsConstraint lvsConstraint;
    IbvsFusionConfig fusion;
    IbvsControlParameters control;
    IbvsKalmanConfig kalman;
};

/**
 * @brief 单个工件 profile 中与 IBVS 相关的配置。
 */
struct IbvsWorkpieceProfile
{
    std::string id;
    std::string displayName;
    std::string legacyName;
    std::string shape;
    std::string robotSimModelId;
    std::string robotSimModelPath;
    std::string calibrationFile;
    std::vector<int> legacyTypeAliases;
    int localizationRoiExpandPx = 250;
    double localizationRoiTopScale = 0.6;
    double localizationRoiBottomScale = 0.1;
    int localizationCenterlineThreshold = 64;
    int localizationPointCloudStride = 5;
    int localizationSlidingWindowFrames = 20;
    int localizationMaxPointCount = 0;
    std::string localizationIcpDof;
    bool hasFeatureGeometry = false;
    IbvsFeatureGeometryParameters featureGeometry;
    bool hasLvsFusionThresholds = false;
    double minLvsConfidence = 0.0;
    double minLvsCenterlineQuality = 0.0;
    bool hasPositionConstraint = false;
    IbvsPositionConstraint positionConstraint;
};

/**
 * @brief IBVS 运行时配置读取结果。
 */
struct IbvsRuntimeTextConfigResult
{
    IbvsError error;
    IbvsRuntimeTextConfig config;
};

/**
 * @brief 工件 profile 配置读取结果。
 */
struct IbvsWorkpieceProfilesResult
{
    IbvsError error;
    std::string defaultWorkpieceId;
    std::vector<IbvsWorkpieceProfile> profiles;
};

/**
 * @brief 读取 IBVS txt 配置并转换为算法值对象。
 *
 * 本类是配置适配层，不属于速度求解器或传感融合算法本身。算法入口仍然
 * 只接收结构体参数，避免在纯计算路径中引入文件 IO。
 */
class IbvsTextConfigReader
{
public:
    /**
     * @brief 读取 IBVS runtime defaults txt。
     */
    static IbvsRuntimeTextConfigResult readRuntimeDefaults(
        const std::string& path);

    /**
     * @brief 优先读取 primary，失败时读取 fallback。
     */
    static IbvsRuntimeTextConfigResult readRuntimeDefaultsWithFallback(
        const std::string& primaryPath,
        const std::string& fallbackPath);

    /**
     * @brief 读取 IBVS workpiece profiles txt。
     */
    static IbvsWorkpieceProfilesResult readWorkpieceProfiles(
        const std::string& path);

    /**
     * @brief 优先读取 primary，失败时读取 fallback。
     */
    static IbvsWorkpieceProfilesResult readWorkpieceProfilesWithFallback(
        const std::string& primaryPath,
        const std::string& fallbackPath);

    /**
     * @brief 按兼容 workpieceType 查找对应 profile。
     */
    static const IbvsWorkpieceProfile* findProfileForLegacyType(
        const std::vector<IbvsWorkpieceProfile>& profiles,
        int legacyType);

    /**
     * @brief 按可配置工件 id 查找 profile。
     */
    static const IbvsWorkpieceProfile* findProfileById(
        const std::vector<IbvsWorkpieceProfile>& profiles,
        const std::string& workpieceId);
};

} // namespace i2r_rlc::ibvs

#endif // I2R_RLC_IBVS_IBVS_TEXT_CONFIG_READER_H
