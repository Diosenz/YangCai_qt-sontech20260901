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

## 在线启动初始化与中断重启

在线作业的 RobotComm 预检在发送 J 前先发送普通 N，确认当前 D 中程序停止且
没有未完成 O 批次（`offline != 4`），再发送一次第九字段为 1 的 N 初始化请求。
任一事务失败都不能报告在线就绪。普通轮询、自动重连和离线激光预检继续发送
第九字段为 0 的 N，不反复初始化。

控制柜存档 `TcpSvr.c::ReceiveDATA` 将 N 第九字段解释为 `OnlineCor_initflag`，
值为 1 时清除 B005；它与 P 第九字段的在线结束标志语义不同。JBI 在接近 P001
之后才调用 ENDDET，提前停止可能跳过该清理，因此每次在线启动都重新请求初始化。
该请求不重置 O 点计数，未完成批次仍禁止直接重启。

D 没有 B005 或运动目标读回。初始化事务成功只能证明请求已发送并收到有效 D，
控制柜实际清除行为仍需确认安装的 MotoPlus 版本；P001/EX001 写入、MOVL 取值和
工具/坐标/构型属性，以及 P001 到固定 C 点的运动连续性必须另行实机验证。
