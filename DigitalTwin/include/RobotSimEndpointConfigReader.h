/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      RobotSimEndpointConfigReader.h
 * @brief     RobotSim endpoint 配置读取器。
 * @details   本文件定义 DigitalTwin 使用的轻量 key-value endpoint 配置读取入口，
 *            调用方负责提供已经解析好的文件路径。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 17 日
 */

#ifndef I2R_RLC_DIGITAL_TWIN_ROBOT_SIM_ENDPOINT_CONFIG_READER_H
#define I2R_RLC_DIGITAL_TWIN_ROBOT_SIM_ENDPOINT_CONFIG_READER_H

#include <QString>

#include "IDigitalTwinService.h"

namespace i2r_rlc::digital_twin {

/**
 * @brief RobotSim endpoint 配置加载结果。
 */
struct RobotSimEndpointConfigLoadResult {
    bool loaded = false;
    RobotSimEndpointConfig config;
    QString message;
};

/**
 * @brief 读取 RobotSim endpoint 的轻量 key-value 配置。
 *
 * 支持 `host`、`port`、`connect_timeout_ms`、`request_timeout_ms` 和可选的
 * `reference_object_id`。本类不负责查找项目根目录；调用方传入已经解析好的
 * 文件路径。
 */
class RobotSimEndpointConfigReader final {
public:
    /**
     * @brief 从指定文件读取 RobotSim endpoint 配置。
     */
    RobotSimEndpointConfigLoadResult readFile(const QString& filePath) const;
};

} // namespace i2r_rlc::digital_twin

#endif // I2R_RLC_DIGITAL_TWIN_ROBOT_SIM_ENDPOINT_CONFIG_READER_H
