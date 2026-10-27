# 新Release人工采集与验收准备

2026-10-27。软件侧已完成性能诊断、实时主图和三模板等价优化。设备由用户操作；本次未自动启动相机、机器人或正式程序。

## 已构建程序与具体变化

程序：`H:/YangCai_Code/YangCai_qt-sontech20260901/out/build/qt-release/UI/i2r_rlc.exe`。请在仓库根目录作为工作目录启动，沿用既有配置/标定/工件设置。对应代码提交 `2b7397f`（性能诊断）、`df77d22`（UI）、`a93a37d`（三模板FFT复用）。

本次程序SHA256：`10378003b0cc99fedb8162279069092665f29e9e06f59fe158ff3f83894e75b2`，完整构建产物记录在 `output/analysis/online_correction_optimization/release-artifacts-verified.json`。后续重新构建后应重新记录，不能沿用旧哈希。

主图775×578显示最新原图，小窗300×224保留检测同曝光及标记，各自显示帧号/年龄。算法三模板训练减少重复FFT；ROI、训练频率、可信判据、PID、标定及80/60μm保护未调整。具体行为、收益及限制见01/02/03结果文档。

## 首轮采集

1. 保留当前程序与历史会话，记录这轮程序哈希、Release配置、相机设置、实际作业速度、标定版本、工件摆放及轨迹/工件ICP开关。前后对照条件必须一致。
2. 在检测成功率未知时，按原图/实际作业位置预先标注normal、reflection、preweld、loss区间；记录半开 `[start_ms,end_ms)`，时间为曝光使用的本机epoch毫秒。不能事后只挑检测成功帧。起步、稳定和预焊业务段也需单独标注。
3. 用户按已有流程进行完整一轮，包含正常、预焊和反光及恢复。保留整个 `output/online_correction/<本次目录>/`，尤其unified_session、vision_diagnostics、actual_robot_path、online_follow_targets和loss_window原图。默认在线视觉诊断已启用；确认新观察包含performance及runtime_received_ms，缺失仍按未知处理。
4. 核对大图帧号是否持续前进，检测小窗能否保持原曝光，观察按钮和窗口交互。当前生产窗口已发出liveFramePainted信号，但尚无现场持久化消费者；离线受控30Hz报告不能替代现场25Hz/150ms的量化验收，屏幕录像只能辅助观察。

离线分析入口（在仓库根目录，Python使用本机已可用解释器）：

```powershell
python tests/diagnostics/online_detector_replay/performance_report.py output/online_correction/<本次目录> --intervals <预标注区间.json> --output output/analysis/<新性能报告.json>
python tests/diagnostics/online_detector_replay/tcp_trace_audit.py output/online_correction/<本次目录>/unified_session.jsonl --output output/analysis/<新TCP关联报告.json>
```

区间JSON格式：`[{"label":"normal","start_ms":曝光区间起点,"end_ms":曝光区间终点}]`，把说明文字替换为真实整数后才是可用JSON。正式正常段至少包含完整10秒窗；尾窗单列。查看每窗有效新曝光Hz、曝光到runtime接收P95、空间间隔和最长缺口，目标10Hz（争取15Hz）及200ms；预测、插值、未知支持和无效样本保留。

## 固定参考基线

先明确参考建立方法：优先独立测得的焊缝；若暂用一次离线生成的参考折线，应冻结其数据/版本，不允许用本轮检测点重新拟合。记录参考坐标系、长度单位mm、工件摆放及变位机角度对应关系，保存标定和必要变换。在线检测点、变位机末节系futureFoot和机器人基座系TCP不能直接相减；必须先转换到同一固定坐标系。

同一冻结参考、相同摆放/速度/完整业务窗口下记录基线与候选；检测/算法优化后的首轮先作为新基线，不与旧0.923mm混比。按反馈的时间权重评价平均/P95/最大误差，沿参考等距离空间复核，并分别保留起步、稳定、预焊、反光与失测区间。关联缺失和无效区间要报告覆盖率，不按检测成功筛选TCP样本，也不跨长缺口插值假造轨迹。

归因顺序：检测线是否正确 → 首点/方向点与futureFoot是否正确 → 实际命令发送/合并与反馈是否可关联 → TCP残差。通信回执为unknown_execution，不能当作机器人执行确认；精确通信时延需补独立发送/接收时刻后再评价。只有归因支持时才实施一项跟随候选，列出旧/新值及独立前后对照。

TCP相对参考目标：平均下降至少20%，P95及最大不恶化；独立真实焊缝平均2mm/最大5mm另验。当前两者均未验收。任务04/05仍待固定参考及现场新数据，任务06尚不具备完整联合结项条件。
