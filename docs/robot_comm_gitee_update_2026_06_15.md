# RobotComm Gitee 更新记录（2026-06-15）

## 来源

- 外部仓库：`https://gitee.com/irr_scut_edu_cn/CommRobot.git`
- 本次参考提交：`6ca4f01`，提交说明为“移除 Noitf 及其所有派生类，消息队列中直接存储 string”。

## 集成边界

本仓库继续保留 `RobotComm/` 作为当前 Qt 项目的生产模块，CMake target 仍为
`i2r_rlc_robot_comm`，命名空间仍为 `i2r_rlc::robot_comm`。

未导入外部仓库的 `example/`、根 `CMakeLists.txt` 和 `vcpkg.json`。这些文件属于外部独立工程，
不应覆盖当前项目的模块化 CMake 边界。

## 接口变化

- `Client` 从 `ZMQ_REQ` 改为 `ZMQ_STREAM` 通信模式。
- `Client` 连接后会读取 ZeroMQ stream peer id，发送和接收均按“身份帧 + 数据帧”处理。
- `ClientQueue` 不再接收 `NotifInterface` 派生对象，消息队列直接保存 `std::string`。
- `ClientQueue::enqueueNotif(...)` 被移除，替换为 `ClientQueue::enqueueMessage(...)`。
- `MessageSerialize`、`Notif`、`NotifConcept`、`Ciallo` 已随外部新版移除。
- `RobotState` 合并到 `RobotStateSnap::RobotState`，状态快照仍由 `RobotStateSnap` 统一保存。
- `config::TIMEOUT` 改名为 `config::CLIENT_QUEUE_TIMEOUT`。

## 注意事项

真实机器人通信行为需要在控制柜或仿真端确认。当前更新只同步外部通信模块的接口和传输方式，
不改变机器人默认 IP、端口和当前项目的模块依赖边界。
