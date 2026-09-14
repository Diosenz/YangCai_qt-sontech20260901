/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IDigitalTwinService.h
 * @brief     数字孪生服务公共接口。
 * @details   本文件定义主项目访问 RobotSim 的状态、命令和虚拟参考数据边界；
 *            定位、视觉伺服和工艺策略由各自模块负责。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 17 日
 */

#ifndef I2R_RLC_DIGITAL_TWIN_I_DIGITAL_TWIN_SERVICE_H
#define I2R_RLC_DIGITAL_TWIN_I_DIGITAL_TWIN_SERVICE_H

#include <vector>

#include <QMetaType>
#include <QString>
#include <QtGlobal>

namespace i2r_rlc::digital_twin {

/**
 * @brief 数字孪生服务状态。
 */
enum class DigitalTwinServiceState {
    Unavailable,
    Idle,
    Running,
    Error
};

/**
 * @brief RobotSim 外部进程通信端点配置。
 *
 * DigitalTwin 连接已经启动的 RobotSim 进程，不负责启动、关闭或嵌入
 * RobotSim，也不把 RobotSim 的 OCCT/VTK 依赖带入主工程。
 */
struct RobotSimEndpointConfig {
    QString host = QStringLiteral("127.0.0.1");
    quint16 port = 0;
    int connectTimeoutMs = 1000;
    int requestTimeoutMs = 1000;
    /// @brief 三维传感结果的参考对象 ID；小于等于 0 时省略并使用世界坐标。
    int referenceObjectId = 0;
};

/**
 * @brief 六自由度位姿，平移单位为 mm，转角单位为 deg。
 */
struct DigitalTwinPose6D {
    double x_mm = 0.0;
    double y_mm = 0.0;
    double z_mm = 0.0;
    double rx_deg = 0.0;
    double ry_deg = 0.0;
    double rz_deg = 0.0;
};

/**
 * @brief 数字孪生服务启动结果。
 */
struct DigitalTwinStartResult {
    DigitalTwinServiceState state = DigitalTwinServiceState::Unavailable;
    QString message;
};

/**
 * @brief 数字孪生服务命令结果。
 */
struct DigitalTwinCommandResult {
    bool succeeded = false;
    QString message;
};

/**
 * @brief 数字孪生位姿查询结果。
 */
struct DigitalTwinPoseResult {
    bool available = false;
    DigitalTwinPose6D pose;
    QString message;
};

/**
 * @brief 数字孪生中的二维图像点，单位 pixel。
 */
struct DigitalTwinImagePoint2D_px {
    double x_px = 0.0;
    double y_px = 0.0;
};

/**
 * @brief 数字孪生中的三维点，单位 mm。
 */
struct DigitalTwinPoint3D_mm {
    double x_mm = 0.0;
    double y_mm = 0.0;
    double z_mm = 0.0;
};

/**
 * @brief 主项目选择 RobotSim 当前工件模型的最小输入。
 */
struct DigitalTwinWorkpieceSelection {
    QString workpieceId;
    QString robotSimModelId;
    QString robotSimModelPath;
};

/**
 * @brief 虚拟激光/点云参考请求。
 */
struct DigitalTwinVirtualReferenceRequest {
    qint64 frameId = -1;
    QString workpieceId;
    int sensorId = 1013;
    int robotId = -1;
    int positionerId = -1;
    /// @brief GET_LSL_PHOTO/GET_WELD_SEAM_PHOTO 的可选参考对象 ID。
    int referenceObjectId = 0;
    std::vector<double> robotJointValuesRad;
    std::vector<double> positionerJointValuesRad;
    DigitalTwinPose6D workpiecePose;
    bool needLaserLineImagePoints = true;
    bool needLaserLinePoints = true;
    bool needLocalPointCloud = true;
    /// @brief 是否在批量请求中追加 GET_WELD_SEAM_PHOTO。
    bool needWeldSeamPoint = true;
};

/**
 * @brief 虚拟激光/点云参考查询结果。
 */
struct DigitalTwinVirtualReferenceResult {
    bool available = false;
    bool supported = false;
    qint64 frameId = -1;
    QString workpieceId;
    DigitalTwinPose6D referencePose;
    /// @brief 三维点参考对象 ID；0 表示 RobotSim 世界坐标。
    int referenceObjectId = 0;
    /// @brief true 表示 `seamPoint_mm` 包含 RobotSim 返回的三维坐标。
    bool hasSeamPoint_mm = false;
    /// @brief 仿真焊缝点坐标；坐标系由请求的 referenceObjectId 决定，单位 mm。
    DigitalTwinPoint3D_mm seamPoint_mm;
    /// @brief true 表示 `seamPointImage_px` 包含仿真相机图像坐标。
    bool hasSeamPointImage_px = false;
    /// @brief RobotSim 返回的仿真焊缝点图像坐标，单位 pixel。
    DigitalTwinImagePoint2D_px seamPointImage_px;
    std::vector<DigitalTwinImagePoint2D_px> laserLineImagePoints_px;
    /// @brief 仿真激光三维点；坐标系由 referenceObjectId 标识，单位 mm。
    std::vector<DigitalTwinPoint3D_mm> laserLinePoints_mm;
    std::vector<DigitalTwinPoint3D_mm> localPointCloud_mm;
    QString message;
};

/**
 * @brief 数字孪生服务状态快照。
 */
struct DigitalTwinStatusSnapshot {
    DigitalTwinServiceState state = DigitalTwinServiceState::Unavailable;
    RobotSimEndpointConfig endpoint;
    QString lastError;
    qint64 lastResponseTimeMs = -1;
};

/**
 * @brief 数字孪生服务接口。
 *
 * 本接口作为 RobotSim 对接 facade 的边界。虚拟点云、虚拟激光线和仿真结果由
 * RobotSim 提供；真实/虚拟点云配准和定位求解属于 localization 模块。
 */
class IDigitalTwinService {
public:
    virtual ~IDigitalTwinService() = default;

    /**
     * @brief 启动或声明服务状态。
     */
    virtual DigitalTwinStartResult start() = 0;

    /**
     * @brief 按指定 RobotSim 端点启动服务连接。
     */
    virtual DigitalTwinStartResult start(const RobotSimEndpointConfig& endpoint) = 0;

    /**
     * @brief 停止服务连接并释放通信资源。
     */
    virtual void stop() = 0;

    /**
     * @brief 返回最近一次服务状态。
     */
    virtual DigitalTwinStatusSnapshot status() const = 0;

    /**
     * @brief 设置 RobotSim 中机器人关节值。
     */
    virtual DigitalTwinCommandResult setRobotJoints(
        int robotId,
        const std::vector<double>& jointValues) = 0;

    /**
     * @brief 设置 RobotSim 中机器人 TCP 位姿。
     */
    virtual DigitalTwinCommandResult setRobotTcpPose(
        int robotId,
        const DigitalTwinPose6D& pose) = 0;

    /**
     * @brief 查询 RobotSim 中对象绝对位姿。
     */
    virtual DigitalTwinPoseResult getObjectPose(int objectId) = 0;

    /**
     * @brief 设置 RobotSim 中对象位姿。
     */
    virtual DigitalTwinCommandResult setObjectPose(
        int objectId,
        const DigitalTwinPose6D& pose,
        bool isRelative,
        bool updateChildren) = 0;

    /**
     * @brief 选择或同步 RobotSim 当前工件模型。
     */
    virtual DigitalTwinCommandResult selectWorkpiece(
        const DigitalTwinWorkpieceSelection& selection) = 0;

    /**
     * @brief 请求当前工件下的虚拟激光线和局部点云参考。
     */
    virtual DigitalTwinVirtualReferenceResult requestVirtualReference(
        const DigitalTwinVirtualReferenceRequest& request) = 0;
};

} // namespace i2r_rlc::digital_twin

Q_DECLARE_METATYPE(i2r_rlc::digital_twin::DigitalTwinVirtualReferenceResult)

#endif // I2R_RLC_DIGITAL_TWIN_I_DIGITAL_TWIN_SERVICE_H
