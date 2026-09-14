# DigitalTwin RobotSim facade v1

## 结论

`DigitalTwin` 第一阶段只作为主项目到外部 `RobotSim` 进程的 facade，不嵌入
`RobotSim/` 源码，也不把 OpenCascade、VTK 或 RobotSim 内部模型对象引入主工程。

当前 v1 目标是先建立稳定边界：

- 连接 RobotSim ZeroMQ ROUTER/DEALER JSON 服务；
- 发送 RobotSim 当前已经注册的对象/机器人命令；
- 为 Localization 提供显式的虚拟参考失败边界；
- 保持定位求解、点云配准和视觉算法仍归属于各自模块。

## 当前支持

`RobotSimDigitalTwinService` 当前只使用 RobotSim 已注册的通信命令：

- `ROBOT_VOID / SET_Q_VEC`：同步机器人关节值；
- `ROBOT_VOID / SET_POSE_FRAME_TCP`：同步机器人 TCP 位姿；
- `BASE_OBJECT_VALUE_RET / GET_ABS_POSE`：查询对象绝对位姿；
- `BASE_OBJECT_VOID / SET_POSE`：设置对象位姿。

通信端点通过 `config/digital_twin/robotsim_endpoint.txt` 配置。由于 RobotSim 目前绑定
随机端口并只在控制台打印端口，联调时需要先启动 RobotSim，再把控制台端口写入配置。

## 仍缺少的 RobotSim 能力

当前不能直接完成虚拟点云或视觉仿真数据闭环，原因在 RobotSim 对外 API 尚未暴露：

- 固定端口、端口文件或握手机制；
- `TOOL_LSL_SENSOR_*` 命令注册；
- `CToolLSLSensor::getLSLPhoto()` 或等价虚拟激光线结果的通信导出；
- 虚拟工件点云或世界坐标激光线点集导出。

在这些 API 补齐前，`DigitalTwinLocalizationVirtualReferenceProvider` 会返回
`available=false`，并把缺口写入 message，避免定位模块误以为已经拿到有效虚拟参考。
