/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      RobotSimSceneConfigReader.h
 * @brief     RobotSim 场景对象 ID 配置读取器。
 */

#ifndef I2R_RLC_DIGITAL_TWIN_ROBOT_SIM_SCENE_CONFIG_READER_H
#define I2R_RLC_DIGITAL_TWIN_ROBOT_SIM_SCENE_CONFIG_READER_H

#include <QString>

namespace i2r_rlc::digital_twin {

/**
 * @brief 一个 RobotSim 场景中与实时仿真有关的对象 ID。
 */
struct RobotSimSceneConfig {
    QString sceneName;
    int robotId = -1;
    int positionerId = -1;
    int sensorId = -1;
    /// @brief RobotSim 中需要由全局定位结果修正的工件对象 ID。
    int workpieceId = -1;
    /// @brief 工件直接父级对象 ID；全局定位矩阵在该坐标系中左乘。
    int workpieceParentId = -1;
};

/**
 * @brief RobotSim 场景配置加载结果。
 */
struct RobotSimSceneConfigLoadResult {
    bool succeeded = false;
    RobotSimSceneConfig config;
    QString message;
};

/**
 * @brief 读取 key-value 格式的 RobotSim 场景对象 ID。
 */
class CRobotSimSceneConfigReader final {
public:
    /**
     * @brief 从指定文件读取 RobotSim 场景对象 ID。
     *
     * `workpiece_id` 和 `workpiece_parent_id` 仅供定位结果写回使用；旧配置
     * 可不包含它们，但此时写回操作会明确失败。
     */
    static RobotSimSceneConfigLoadResult load(const QString &filePath);
};

} // namespace i2r_rlc::digital_twin

#endif // I2R_RLC_DIGITAL_TWIN_ROBOT_SIM_SCENE_CONFIG_READER_H
