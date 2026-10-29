Status: ready-for-agent
Category: bug

# 正式启动后的连续采集丢失实际检测输入

入口：CRuntimeCoordinator::startOnlineCorrectionJob。现有 worker/recorder 直连测试通过，但现场连续采集全部缺输入。

先通过完整启动、启动事务清理、Vision 再配置、处理曝光、停止排空和记录器审计复现。
修复后必须有同曝光灰度图、仿真条纹、参数、时间、版本和生命周期事件；UI 不携带归档大对象。
关闭采集时保持关闭；后续会话不能沿用旧策略。不得修改检测算法以令采集回归通过。

## Comments

2026-10-28：已保留修改前源码快照，新增公开启动入口回归，准备 MSVC 验证红灯。

2026-10-28：完整启动回归 red：4 个处理曝光，4 份上下文均 incomplete、图像尺寸 0x0、无仿真帧号。
固定会话诊断策略后，同一回归 green，4 帧完整性审计通过，生产 DetectorReplay 的像素/有效性/控制资格逐帧一致。
显式关闭采集、关闭诊断、runtime 全量、记录器全量回归均通过；主程序已在既有 release 目录构建并完成禁用设备的 8 秒启动检查。
根因：clearOnlineCorrectionStart 清空临时配置，而 onlineTrackingOptions 继续从临时配置读取诊断选项；现场重配 Vision 后退回默认策略。
修复只改变会话诊断策略生命周期，未改变生产检测位置或参数。现场跳变完整回放仍待新采集。
