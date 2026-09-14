/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsConfig.h
 * @brief     IBVS 模块公共默认配置常量。
 * @details   本文件集中定义公共参数对象使用的默认值，供配置文件读取和界面组装使用。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 8 日
 */

#ifndef I2R_RLC_IBVS_IBVS_CONFIG_H
#define I2R_RLC_IBVS_IBVS_CONFIG_H

namespace i2r_rlc::ibvs {

inline constexpr double FEATURE_GEOMETRY_H1_MM_DEFAULT = 21.0;
inline constexpr double FEATURE_GEOMETRY_H2_MM_DEFAULT = 24.0;
inline constexpr double FEATURE_GEOMETRY_FOCAL_LENGTH_PX_DEFAULT = 2318.8;
inline constexpr double FEATURE_GEOMETRY_TORCH_TILT_SCALE_DEFAULT = 1.0;
inline constexpr bool FEATURE_GEOMETRY_USE_DEPTH_BASED_H2_DEFAULT = false;
inline constexpr double FEATURE_GEOMETRY_MIN_FEATURE_DISTANCE_PX_DEFAULT = 1e-6;
inline constexpr double FEATURE_GEOMETRY_LEGACY_H2_PX_DEFAULT = 38.6409;
inline constexpr double FEATURE_GEOMETRY_LEGACY_UNIT_U_DEFAULT = -0.7513;
inline constexpr double FEATURE_GEOMETRY_LEGACY_UNIT_V_DEFAULT = 0.6599;

} // namespace i2r_rlc::ibvs

#endif // I2R_RLC_IBVS_IBVS_CONFIG_H
