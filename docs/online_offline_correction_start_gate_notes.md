# 在线/离线纠偏启动门禁记录

## 旧工程依据

旧工程只读对比路径：

- `E:\SontechWorks-new\SontechWorks_Weld_Merge\SWv1\robotleadingDlg.cpp`
- `E:\SontechWorks-new\SontechWorks_Weld_Merge\SWv1\RobotCtrl.cpp`

`CRobotleadingDlg::OnBnClickedOnebuttonweld()` 是“一键全自动”入口，随后进入
`OnChangeVirtualStatus()`。在线启动分支在发送 JBI 前使用：

```cpp
while (sStart != 0 || sMode != 2 || sServoPower != 1) {
    MessageBox(_T("准备启动程序，请：\n1、停止机器人当前动作\n2、切换至PLAY模式\n3、打开伺服"), _T("提示!"), MB_OK);
}
```

随后才设置 `jobname_dt` 和 `jobstart = 1`。实际 J 命令发送位于
`RobotCtrl::RevRobotData()`，仍要求 `sStart == 0 && jobstart == 1`。

离线纠偏会通过 `Offcorstart_OnChangeVirtualStatus(VS_ONLINE_REC_MOD)` 复用
`OnChangeVirtualStatus()`。旧源码中 `m_Offline != NULL` 分支的同款 while 当前被注释，
但本次按现场目标在新工程中对在线/离线统一启用同一门禁。

## 新工程行为

Qt 工程新增 `CCorrectionRobotStartGate`，发送在线/离线纠偏 JBI 前必须满足：

- 已收到有效 D 回包；
- `RobotState::sStart == 0`；
- `RobotState::sMode == 2`；
- `RobotState::sServoPower == 1`。

不额外检查 `sHold`、`sIsAlarm`、`sRemote`，以避免扩大旧工程提示中的门禁范围。

UI 不使用阻塞式 `while`。当状态不满足时，关闭提示框后通过 Qt event loop 重新检查；
满足门禁后只发送一次 JBI，并启动对应在线/离线检测流程。
