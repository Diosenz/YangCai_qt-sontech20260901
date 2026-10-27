# 02 实时主图与检测快照

2026-10-27，起点 `2b7397f`。

## 具体调整

- `MainWidget::showLivePreview`：旧行为为新相机帧只刷新220×164小窗；现在刷新775×578实时主图，不叠旧检测点。检测时UI主图刷新不等待算法结果。
- `MainWidget::showTrackingFrame/updateDisplayedImage`：旧检测原图及全部标记占主图；现在保留在300×224检测小窗，标记始终依据该曝光原图缩放。实时相机到来不改小窗，检测结果也不把大图倒退到旧曝光。
- 主图/小窗各自显示帧号与曝光年龄；缺时间显示未知，逆序时钟显示异常。年龄每100ms刷新，图像仅随新帧/检测结果改变。
- 小窗标题保留同帧参考缺失、来源、非独立实测和控制资格限定，完整说明也保留tooltip。小窗高度及标题几何固定，长文字不推动周边面板。
- `CMeasuredImageLabel` 在主图 `paintEvent` 返回后发出 `liveFramePainted(frameId,exposureMs,paintedAtMs)`；同帧重绘不增加计数。它是Qt绘制完成标记，不代表显示器物理呈现。时间未知不伪造有效延迟。
- `UiController`开始在线等待首个检测时仍可显示已有最新原图。停止恢复纯相机显示，清理检测叠加。

风险：检测小窗尺寸比原图小，标记仍保留像素相对位置，完整状态文字需视实际屏幕检查；真实相机线程、设备负载和窗口交互仍需现场验证。本项不改采图频率、检测模型、训练或控制参数。

## 验证

先调整原有 `--live-preview-contract` 对外断言，旧实现RED（新大图未更新/旧快照归属错误），新实现GREEN。补参考缺失与无控制资格文字断言曾RED，修复后GREEN。重复同帧repaint不增加新帧计数，原生全套main widget panel tests在Debug和Release均PASS。

10秒受控30Hz注入，以重复实拍图和人工帧号测试Qt绘制能力，实际统计paint完成：

| 构建 | 新帧绘制频率 | 注入到绘制P95 | 事件队列focus响应P95 |
|---|---|---|---|
| Debug | 30.215/s | 11ms | 0.244ms |
| Release | 30.294/s | 3ms | 0.150ms |

报告在 `output/analysis/online_correction_optimization/ui-debug-paint.json` 与 `ui-release-paint.json`。实拍截图 `ui-live-main-final.png` 已检查，中文字体、主图与小窗及绿色标记可见。截图工具为offscreen Qt，未调用鼠标或设备。

规范审查无问题；规格审查发现小窗截短状态文字，已修正并补RED/GREEN测试。此处验证离线绘制及队列响应，人工点击/移动窗口的现场流畅、真实曝光到绘制P95和物理显示均未验收。
