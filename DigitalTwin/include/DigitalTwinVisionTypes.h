/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      DigitalTwinVisionTypes.h
 * @brief     DigitalTwin 提供给 Vision 的轻量仿真观测类型。
 * @details   本头文件只定义跨模块按值传递的数据结构和转换函数，不包含
 *            RobotSim transport/socket/json 通信实现。Vision、Runtime 和测试
 *            只能消费这些 DTO，不直接访问 RobotSim。
 */

#ifndef I2R_RLC_DIGITAL_TWIN_VISION_TYPES_H
#define I2R_RLC_DIGITAL_TWIN_VISION_TYPES_H

#include <optional>
#include <vector>

#include <QMetaType>
#include <QString>

#include "IDigitalTwinService.h"

namespace i2r_rlc::digital_twin {

/**
 * @brief Vision 在线检测可消费的一帧数字孪生激光观测。
 *
 * 所有字段按值保存，跨线程投递时不得传递外部容器引用或裸指针。二维点单位为
 * pixel，三维焊缝点和切换点单位为 mm。`seamIndex` 当前 RobotSim 尚未提供，
 * 默认 -1。
 */
struct DigitalTwinVisionLaserObservation {
    /// @brief true 表示本帧具备至少两个仿真激光线图像点。
    bool valid = false;

    /// @brief 相机帧号；无法严格绑定时可为 -1。
    qint64 frameId = -1;

    /// @brief 仿真激光线二维图像点列，单位 pixel。
    std::vector<DigitalTwinImagePoint2D_px> laserLineImagePoints_px;

    /// @brief 与当前帧位姿对应的仿真激光线三维点列，单位 mm。
    std::vector<DigitalTwinPoint3D_mm> laserLinePoints_mm;

    /// @brief 三维激光线和焊缝点参考对象 ID；0 表示 RobotSim 世界坐标。
    int referenceObjectId = 0;

    /// @brief true 表示 `weldPointImage_px` 可用。
    bool hasWeldPointImage_px = false;

    /// @brief 仿真焊缝点二维图像坐标，单位 pixel。
    DigitalTwinImagePoint2D_px weldPointImage_px;

    /// @brief true 表示 `weldPoint_mm` 包含 RobotSim 返回的三维坐标。
    bool hasWeldPoint_mm = false;

    /// @brief RobotSim 焊缝点；坐标系由 referenceObjectId 标识，单位 mm。
    DigitalTwinPoint3D_mm weldPoint_mm;

    /// @brief 当前仿真焊缝段编号；未知时为 -1。
    int seamIndex = -1;

    /// @brief 虚拟激光线所属 STARTPOSSEG 定位面；未知为 -1，预热整模为 0。
    int positioningSegmentIndex = -1;

    /// @brief 旧 `sim_scan_line_change_point` 等价切换点，单位 mm。
    std::optional<DigitalTwinPoint3D_mm> scanLineChangePoint_mm;

    /// @brief 转换或诊断说明。
    QString message;
};

/**
 * @brief 构造无效 Vision 仿真观测。
 *
 * @param frameId 帧号，单位 frame；未知时为 -1。
 * @param message 无效原因。
 * @return 无效观测 DTO。
 */
inline DigitalTwinVisionLaserObservation makeInvalidVisionLaserObservation(
    qint64 frameId,
    const QString& message)
{
    DigitalTwinVisionLaserObservation observation;
    observation.valid = false;
    observation.frameId = frameId;
    observation.message = message;
    return observation;
}

/**
 * @brief 将 RobotSim 虚拟参考结果收敛为 Vision 激光观测 DTO。
 *
 * 转换成功要求 RobotSim 结果可用、二维激光线至少 2 点且包含焊缝图像点。
 * 本函数不伪造焊缝点、seamIndex 或 scanLineChangePoint；失败时优先保留
 * RobotSim 服务给出的具体错误，便于上层按原始原因展示和计数。
 */
inline DigitalTwinVisionLaserObservation makeVisionLaserObservation(
    const DigitalTwinVirtualReferenceResult& reference)
{
    DigitalTwinVisionLaserObservation observation;
    observation.frameId = reference.frameId;
    observation.laserLineImagePoints_px = reference.laserLineImagePoints_px;
    observation.laserLinePoints_mm = reference.laserLinePoints_mm;
    observation.referenceObjectId = reference.referenceObjectId;
    observation.hasWeldPointImage_px = reference.hasSeamPointImage_px;
    if (reference.hasSeamPointImage_px) {
        observation.weldPointImage_px = reference.seamPointImage_px;
    }
    observation.hasWeldPoint_mm = reference.hasSeamPoint_mm;
    if (reference.hasSeamPoint_mm) {
        observation.weldPoint_mm = reference.seamPoint_mm;
    }

    if (!reference.available) {
        observation.message = reference.message.isEmpty()
            ? QStringLiteral("DigitalTwin virtual reference is unavailable")
            : reference.message;
        return observation;
    }
    if (reference.laserLineImagePoints_px.size() < 2U) {
        observation.message = reference.message.isEmpty()
            ? QStringLiteral(
                "DigitalTwin vision observation requires at least 2 laser points")
            : reference.message;
        return observation;
    }
    if (!reference.hasSeamPointImage_px) {
        observation.message = reference.message.isEmpty()
            ? QStringLiteral(
                "DigitalTwin vision observation requires a weld point")
            : reference.message;
        return observation;
    }
    observation.valid = true;
    observation.message = reference.message.isEmpty()
        ? QStringLiteral("DigitalTwin vision observation available")
        : reference.message;
    return observation;
}

} // namespace i2r_rlc::digital_twin

Q_DECLARE_METATYPE(i2r_rlc::digital_twin::DigitalTwinVisionLaserObservation)

#endif // I2R_RLC_DIGITAL_TWIN_VISION_TYPES_H
