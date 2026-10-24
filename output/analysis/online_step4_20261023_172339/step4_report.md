# 步骤 4 实施与反证审查（2026-10-23）

本轮修复位于已定位的根因边界：慢帧导致事务误撤销、有界原图搜索、缺测控制参考，以及目标快照的 UI 生命周期。反向扫描选起点算法保留。遵循已有 Notion 软件工作流快照（提供方版本 2026-10-18），没有声称已读取更新版本。

## 权威数据与完整链路

1. 曝光原图＋冻结机器人/外轴/标定 → Vision 原图精定位或质量验收 KCF → Runtime 测量 DTO；仿真只提案搜索。独立原图点与 estimated KCF 身份区分，不把显示重投影或 Control 预测变成测量。
2. 同帧原图/母材参考 → Runtime 局部配准 → 确认 SET/GET 写回 → 后验 → 验证/原有保留结案 → 新窗口。旧实现将冻结帧延迟与当前通信失鲜混同，从这里开始错误撤销事务并 fault。现在旧 session/epoch 先隔离；当前通信失鲜或停止/Hold/模式变化仍撤销，只有旧曝光跳过且打断连续好帧，原五秒时限不重启。
3. 初扫已校正有序路径＋最新机器人反馈＋已通过释放的实测偏差 → Control 同次跟随求解 → 受限 P 增量与 controlTarget 快照 → Worker 盖会话 → Coordinator 过滤 → UI 当前曝光投影/数值显示。缺测预测仅生成 Control 参考；不新增 DetectionSample、独立点或 ICP 输入。
4. controlTarget 的位置是同次求解的 futureFoot。它与 P 使用同一参考线和当前位姿，但 P 是由当前 foot 误差经原 PID/限幅得到的增量；futureFoot 不是 P 协议的绝对终点。未改原增量控制公式。

## 修改必要性与最小反证

| 边界/模块 | 必须改这里的原因；为什么不改其它位置 | 删除修改将失败的测试/行为 |
| --- | --- | --- |
| UI RuntimeLocalRegistration 与两种后验状态 | 错误撤销发生在事务入口；不应放宽定位器质量门或 RobotSim 协议 | online_local_registration_frame_lifecycle：慢帧期间 pending 保持、原期限结案、新窗口再次 SET；安全失联仍撤销 |
| Vision LegacyOnlineKcfDetector | 原图扩大搜索在单帧同步执行大量 refiner，是结果延迟的算法边界；UI 限刷新不能消除该成本 | online_image_measurement：正式在线每帧总 refiner ≤2；当前图复验、竞争检查、3新帧确认；初扫旧入口不变 |
| Control Controller/PathFollower/Types | 检测失测时无新图像点是正确行为；续行参考必须在 Control 由真实反馈重算 | online_correction：无图续行、直/反/曲/回折路径、恢复接管、ICP隔离、静止回包去重、非STARTCOR无P；coordinate_contract：无效/NaN/Inf不触发新P |
| UI Worker/Coordinator/UiController DTO | 旧显示只依赖当前有效检测，不能代表 Control 当前参考；必须沿发布到展示链处理会话和生命周期 | online_control_target_presentation：当前投影、无投影数值、invalid防复活、退出阶段、清会话、迟到结果、下一段反扫原图显示 |

原始四组 RED 全部失败，见 red_tests.log；原始四组 GREEN 见 green_tests.log。独立复核后补真实失败：review_red_tests.log（失效/无投影）、lifecycle_red_tests.log（退出与清会话）、feedback_window_red_tests.log（新增反馈入口窗口门）、reverse_display_red_tests.log（下一段反扫显示）。未使用只检查源码文本的测试替代实际行为。

## 回归失败处置

- 坐标合同原先要求单测量永不发 P；用户允许缺测预测后改为核对 PredictedReference 身份和测量队列不被伪造，所有坐标偏置/方向/零误差断言保留。回归发现无效输入触发预测 P，保留原拒绝合同并修实现；final_review_tests.log 记录真实 RED。
- 跟踪集成测试取消固定首/第三帧已完成完整搜索的假设，以有限新帧等待建立真实身份，保留原图坐标、三连续新曝光、缺图打断、异常冷却和每帧预算断言。两次计数夹具错误及原失败日志均保留。
- Offline frame-sync 夹具缺少启动后的停止预检，把 WaitingForDetection 误当 Vision armed。该预检在本轮前备份已存在。仅补正确的停止预检→真实 Vision 启动→扫描时序；Online 无状态测试保留；Offline 经公开入口预检后无状态不可达，明示不适用。没有放宽生产 Offline 门。boundary_green_tests.log 已确认5/5。
- vision_penzui_station_projection 的 data/replay/penzui 回放数据本地缺失；首次运行挂起后仅停止本轮该测试进程，未改测试或制造数据。该项不计为通过，最终选择清单明示排除。原始24项回归失败日志为 regression_tests.log。

## 验证范围与交付限制

最终构建使用已授权 VS2022 x64 Developer 环境与既有 qt-debug；主程序未启动，设备未启动。最终选择与原始日志分别为 final_test_selection.json、final_validation_build.log、final_tests.log。精确本轮文件及 SHA256 为 changed_files.json，基线为 .codex_backups/online_step4_20261023_172339/manifest.json；新增和额外回归测试均另有备份/清单。

本轮未修改 RobotSim、oldproject、Localization、RobotComm 生产文件，未使用 Computer Use/鼠标，未删除文件。没有圆心/半径/圆焊缝路径类型特判。子 agent 分别负责有界搜索集成夹具/通用性只读审查及同步生命周期调查/最终只读复核，主 agent 统一实现和构建。

有界的是 refiner 调用次数，不是绝对墙钟时间；角点生成与单次 refiner 仍有成本，远重获可能增加跨帧等待。预测依赖初始校正路径与最后可信偏差，无法观测遮挡期间的新偏差。自动化覆盖根因边界和相关回归，不能据此声称现场卡顿、预焊点漂移或所有更新停顿已经消除；仍需连续原图/机器人/工件更新记录的现场闭环复测。

最终结果：主程序及选定目标构建退出码 0；27/27 CTest GREEN，88.79 秒。包括原有合同别名项；不是全仓全量测试。喷嘴投影缺数据项未验证。最终 EXE：out/build/qt-debug/UI/i2r_rlc.exe。
