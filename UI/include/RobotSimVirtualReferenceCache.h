/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      RobotSimVirtualReferenceCache.h
 * @brief     UI 层 RobotSim 虚拟参考缓存。
 * @details   本文件定义 RobotSim 查询结果到 Localization/IBVS 输入的轻量适配边界。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 29 日
 */

#ifndef I2R_RLC_UI_ROBOT_SIM_VIRTUAL_REFERENCE_CACHE_H
#define I2R_RLC_UI_ROBOT_SIM_VIRTUAL_REFERENCE_CACHE_H

#include <optional>

#include <QString>

#include "ibvs/IbvsLaserLineObservation.h"
#include "IDigitalTwinService.h"
#include "LocalizationTypes.h"

namespace i2r_rlc::ui {

/**
 * @brief UI 最近一次 RobotSim 虚拟参考查询缓存。
 *
 * 该结构只保存跨模块可消费的数据快照，不持有 RobotSim 连接、socket 或后台线程。
 */
struct RobotSimVirtualReferenceCache {
    bool available = false;
    int sensorId = 1013;
    qint64 frameId = -1;
    QString message;
    i2r_rlc::localization::LocalizationVirtualReferenceSnapshot localizationSnapshot;
    bool hasIbvsLaserPrior = false;
    i2r_rlc::ibvs::IbvsLaserPrior ibvsLaserPrior;
    std::optional<i2r_rlc::ibvs::IbvsLvsObservation> ibvsLvsObservation;
};

/**
 * @brief 将 DigitalTwin 虚拟参考结果转换为 UI 可缓存的模块输入快照。
 */
RobotSimVirtualReferenceCache makeRobotSimVirtualReferenceCache(
    const i2r_rlc::digital_twin::DigitalTwinVirtualReferenceResult& result,
    qint64 frameId,
    int sensorId);

/**
 * @brief 从 IBVS 激光先验生成 dry-run 可直接消费的 LVS 观测。
 */
std::optional<i2r_rlc::ibvs::IbvsLvsObservation> makeIbvsLvsObservation(
    const i2r_rlc::ibvs::IbvsLaserPrior& prior);

} // namespace i2r_rlc::ui

#endif // I2R_RLC_UI_ROBOT_SIM_VIRTUAL_REFERENCE_CACHE_H
