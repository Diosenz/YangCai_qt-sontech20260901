Category: bug
Status: ready-for-human

# 连续采集与回放闭环

规格：../spec.md。基线7ef1a9f。用户已批准全部阶段；本阶段不改检测算法。
记录器公开入口RED已确认（recorder-red.log），目前GREEN。完整runtime正常28曝光连续
采集审计通过，生产DetectorReplay逐帧像素一致。待完成成熟统计阶段切换、审查及Release构建。

## Comments
2026-10-27 用户追加禁止Computer Use、允许命令行，构建已使用VS2022 Developer环境。

2026-10-27 第一阶段完成：最终112帧普通/成熟/彩色输入严格审计和生产回放一致，Release及11项验证通过，审查问题已修正。等待用户现场复测完整会话，详见../result.md。算法修复尚未开始。

2026-10-28 用户授权使用历史图像推进后，已完成第二阶段修复并交付新版Release，见02-image-recovery-delivery.md及../repair-result.md。当前仅现场完整会话与三维精度验收仍待人工复测；不再以等待新复测为由停止软件修复。
