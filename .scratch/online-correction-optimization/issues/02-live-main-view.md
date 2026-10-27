# 02：实时原图主画面与同曝光检测小窗

Category: enhancement
Status: ready-for-agent
Execution: implemented-offline; field evidence pending

**What to build:** 主画面连续显示最新相机原图，独立小窗展示检测对应曝光与标记，分别标注帧号和年龄。

**Blocked by:** None (can start immediately).

- [x] 从既有公开运行时输入到展示验证新布局，实时主图不继承旧快照检测标记。
- [x] 检测小窗保留同曝光检测与虚拟参考归属，新结果到达即更新来源文字。
- [x] 重复曝光绘制不增加新帧计数，记录曝光到绘制完成的延迟和交互响应。
- [ ] Release 目标至少 25 个不同曝光帧/s、显示延迟 P95 不超过 150 ms；离线呈现能力和现场限制分开报告。
- [x] 会话停止/切换、无结果与缺参考显示行为回归通过。

## Comments

2026-10-27：用户确认拆分，任务已发布；可独立于算法热点调查实施。

2026-10-27：实现和离线验证见 `../02-live-main-view-result.md`；Debug/Release UI全套PASS，受控Qt绘制约30.2～30.3/s。真实连续曝光、现场交互和曝光到绘制延迟仍留待人工设备验收。
