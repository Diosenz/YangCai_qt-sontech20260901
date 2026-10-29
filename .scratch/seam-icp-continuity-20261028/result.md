Status: ready-for-human
Category: bug

# 焊缝识别跳变与工件在线 ICP：软件交付记录

日期：2026-10-28。软件修复、相关回归、现场归档回放、Windows/MSVC构建和独立复审完成；等待现场复测。

## 结论

之前异步日志修复的交付变化没有直接改动Vision检测算法，且同类切换在更早归档中已经存在；不能据此排除闭环处理节奏的间接影响。已分别复现并修复：

1. 扩大搜索状态残留阻断普通跟踪的局部几何校核，清晰恢复无法及时重建母材模型。现在在既有每曝光最多2次精定位预算内交替保留局部校核机会；完整可靠几何结束旧搜索周期。
2. 虚拟参考版本刷新错误废弃真实像素母材模型，同图只改版本即可产生10.7766像素帧间位移。现在仿真相关偏差和融合先验仍失效；真实模型保留但每曝光必须获得当前双侧像素支撑。
3. 逐帧小幅预焊形变递推训练身份和接缝偏置，合成固定焊缝最大漂移14.8909像素。现在保留原始可靠身份及接缝到外围线的法向偏差，并同时核对候选位置及当前母材约束位置；估计不训练模板、不刷新独立几何、不晋升独立实测。
4. 估计点附近原±24像素ICP排除带仍会纳入中央未测区域。现在由非诊断生产证据驱动，估计期间真实表面排除左100、右130像素，完整观测保持±24；采样和写后验证使用同一策略。

未延长12秒种子TTL，未新增通用限速，未改变PID、轨迹ICP、配准求解或写回门禁。

## 两次归档顺序回放

使用生产检测器完整 `process`，源码哈希与最终Vision文件一致。统计相同输入顺序的问题窗口内相邻可用输出的欧氏像素位移；这些数字衡量连续性，不是检测精度。

| 会话 | 曝光窗口 | 原归档最大位移 / px | 修复后 / px |
|---|---|---:|---:|
| 20261028_195020_065 | 10440–10846 | 36.2615 | 5.6626 |
| 20261028_195020_065 | 13625–13695 | 23.3209 | 1.2625 |
| 20261028_194620_530 | 3000–3300 | 17.5106 | 2.4350 |
| 20261028_194620_530 | 5700–6200 | 2.2792 | 2.9533 |

最新归档1803个输入均有保存原图，顺序回放完成。前一归档1661个输入只有1639幅原图，22次缺图显式按unavailable/gap处理；不是原现场历史的精确完整重现。前一归档末段最大相邻位移略增，不能宣称所有窗口都更平滑。截图对应6107帧旧输出约(485.345,596.426)，新输出约(456.5,595.388)，与原图可见折点参考(456.5,594.75)约差0.64像素；单图地标不是物理真值。

证据：`field-final-latest-report.json`、`field-final-previous-report.json`及对应rows/log，汇总比较见 `final-audit.json`。多数预焊输出现在是当前外围母材约束估计；平滑不能代替独立真值。

## ICP开关、刷新与取点

工件在线ICP与Control轨迹ICP是不同开关。首场归档有15次工件SET，版本从1到16；第二场关闭工件ICP后继承版本16场景，因此原两次现场运行不是相同初始位姿的开关对照。

永久lifecycle测试通过生产运行时与内存场景GET/SET验证：初始关闭保持20；开启写21并确认；关闭仍21；再开启基于当前位姿写22；后验期间关闭仍22，无隐式回滚。

8个选择曝光（最新10466、10469、10578、10846、13680、13684，前次6107、6150）分别对比同幅原图的旧/新排除中心。独立红绿证明估计时规定中央区域不再进入真实母材云，外围原图支撑仍满足原有数量门禁；完整观测仍走窄排除带。

冻结矩阵在诊断工具中重建为等价sensor/robot/positioner输入，机器人欧拉角有原协议量化；只据此核对取点像素，不声称精确三维重放。取点工具不求解、不写场景，不把估计点当独立点。掩膜也不能保证所有外围像素语义正确。

证据：`red-surface-mask.log`、`green-surface-mask.log`、最终重验 `surface-mask-final.log`、`surface-mask-results.jsonl`、`surface-mask-summary.json`。

## 构建与测试

全程命令行，先加载 `D:/vs2022/Common7/Tools/VsDevCmd.bat -arch=x64 -host_arch=x64`，复用已有build tree，无Computer Use。

- 原生Release检测器构建通过：`build-native-final.log`。
- 检测新旧回归16/16通过：`regressions-final.log`，113.364秒。
- 运行时专项合同18/18通过：`runtime-contract-results.json`，覆盖版本/失效/重复曝光、正常统计、反光、预焊、恢复、失测和停止；该组在最终UI掩膜变化前运行，Vision之后未再改变。
- 最终Debug构建成功：`build-runtime-final.log`。6项CTest全部通过：`ctest-runtime-final.log`，117.00秒；覆盖recorder、runtime、normal session、seam refinement、registration、frame lifecycle。
- 复审修正测试诊断的损坏变换输入处理后，`build-runtime-review-fix.log`构建成功，registration再次1/1通过：`ctest-registration-review-fix.log`；公开诊断合法/损坏输入红绿：`red-surface-replay-input.log`、`green-surface-replay-input.log`。原图取点对照再次通过。
- Release主程序构建成功：`build-release-mask.log`；最终核对 `build-release-final.log`为 `ninja: no work to do`。
- 禁用自动设备启动、Qt offscreen启动3秒存活，然后只终止本工具启动的进程：`startup-smoke.json`；该进程最终exit1来自主动终止，不是正常退出验收。

交付程序：`H:/YangCai_Code/YangCai_qt-sontech20260901/out/build/release/UI/i2r_rlc.exe`

SHA256：`3e5d65aa4ae11cfe58a69d4bacdedbffde95f0e31b0ec879bff2862a8b04e5f0`。

## 文件范围与审查

生产5文件：`Vision/include/track/LegacyOnlineKcfDetector.h`、`Vision/src/track/LegacyOnlineKcfDetector.cpp`、`UI/include/OnlineLocalRegistrationWorker.h`、`UI/src/OnlineLocalRegistrationWorker.cpp`、`UI/src/RuntimeLocalRegistration.cpp`。

测试3文件：`tests/diagnostics/online_detector_replay/test_preweld_reference_continuity.py`、`tests/test_online_local_registration.cpp`、`tests/test_online_local_registration_frame_lifecycle.cpp`。同步两个现有文档 `docs/online_correction_state_machine.md`、`docs/online_correction_session_replay.md`。

最终Standards及Spec独立复审无未解决发现，历史两项已关闭，见 `review.md`。105个基线文件无意外变化或缺失；修改的7个C++文件均UTF-8 BOM+CRLF，无临时DEBUG探针；`git diff --check`通过。未删除文件，未触碰oldproject/RobotSim/Control，无提交、推送或设备运动。

本目录内probe、minimize、add/fix等脚本是保留的debug-only诊断材料，不属于生产入口，不应直接对现场设备执行。

## 现场复测建议与未完成验收

1. 保存并记录当前工件位姿和版本；开关对照前人工统一初始位姿，关闭ICP不会自动回滚。
2. 先低速、不带弧复测连续原图；分别记录工件ICP关/开，保持相同速度、参考和起始条件，不同时改控制参数。
3. 核对预焊前后清晰恢复、双侧支撑、结果来源/控制资格、工件版本/写回和表面取点；保留缺测曝光而非仅挑成功帧。
4. 用独立焊缝参考核对三维检测与实际TCP误差。此次只完成软件边界验收，未完成真实闭环、真实形状变化、所有反光语义或现场2/5mm精度验收。
