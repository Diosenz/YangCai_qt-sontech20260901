# 在线纠偏作业保持与反馈边界

## 本轮范围

2026-10-26，用户明确要求继续核对类似 SontechWorks 项目的 WithinRobot C 代码；未发现起弧/收弧接口时，留待后续讨论。本轮软件范围因此收敛为运动 HOLD 请求、反馈确认、停止后命令隔离及不自动恢复。弧状态保持“未验证”，其缺失不阻塞有源码依据的 HOLD 通道实施，也不能被包装为已经停弧。

本文记录已接通的软件契约及离线证据。模拟回包验证不表示真实设备已经停止，HOLD 也不证明停弧或机械零速。

## 下位机源码证据

只读核对了本机 SontechWorks、SontechWorks_Weld_Merge、SontechWorks_IBVS 三份工程各自的 WithinRobot 和 motoplus 存档，以及桌面 SontechWorks 拷贝的 WithinRobot。相关目录均未修改。以下行号对应 SontechWorks 的非存档版本：

- `WithinRobot/TcpSvr.c:141` 接收公共八个整数字段，第七个为 `setHold`；`:142-147` 中 tracker 关闭只清检测/纠偏窗口，不调用保持 API。
- `WithinRobot/mpMain.c:185-195` 的任务持续将 `setHold` 赋给 `MP_HOLD_SEND_DATA.sHold`，调用 `mpHold`。这是有实现证据的运动保持入口。
- `WithinRobot/TcpSvr.c:252-253,274-280` 通过 `mpGetPlayStatus` 读取控制器 `sStart` 和 `sHold` 并放入 D 回包；不是请求值的直接回显。
- `WithinRobot/mpMain.c:148-172` 的三个输出分别是 stop_robot、open_laser、open_filter。激光器关闭不能当作焊接电弧关闭；stop_robot 对应的现场接线和梯形图未核实，不能用它替代本轮 HOLD。
- `WithinRobot/TcpSvr.c:206-237` 的其他命令包括 J 启动作业、R 到点、T 建作业、V 视觉伺服，没有独立起弧/收弧命令。`:437-452` 的 StartJob 调用 `mpStartJob`，不能据此假设可以对运行中的焊接主程序异步执行停弧子程序。
- 对上述七份 WithinRobot 源码目录检索 ARCON、ARCOF、weld、arc-on/off、起弧、收弧、停弧及相关 API 关键词，没有找到可接通的独立停弧协议或弧状态反馈。

四份非存档 `mpMain.c`（三个 E 盘工程与桌面拷贝）的 SHA256 均为 `32422BD1D4DD33BD7A3501AC763D158A432CE97E9893685F266A178397FB5AC2`。因此它们不是四项独立现场验证，只是相同源码的多份保存。SDK 头文件中仅出现通用 API include，也不等于已实现或配置了焊机控制。

## JBI 和确认语义

已读的焊接 JBI/JBR 以顺序 ARCON／SMOVC／ENDDETPRG／ARCOF 完成正常焊接；关闭上位机 tracker 或停止发送 P，不会自动让程序立即执行到 ARCOF。STARTCOR 与 ENDDETPRG 子程序只发送检测/纠偏窗口消息。

HOLD 请求的 N/D 事务成功只证明有响应；应等待请求之后由成功事务得到的新 D.sHold 才确认控制器处于 HOLD。旧缓存、直接注入状态、旧会话结果或一次 receipt.completed 不能替代这个确认。HOLD 也不是实测机械零速反馈，D 回包中不存在独立弧状态。

## 软件契约

- 作业停止与普通检测会话清理分开，现有停止按钮进入作业停止，不增加测试专用按钮。
- 一旦请求，协议 HOLD 字段锁存；普通轮询、检测恢复和重连不自动释放。
- 清除待发实时 P，并抑制停止之后的 P/O/J 等运动请求；已经开始的同步传输无法事后撤回，停止请求必须在通信线程接续处理。
- 公开状态区分待确认、已确认 HOLD 与失败/通信不可确认；所有状态文字保留“停弧未验证”。
- 保持不自动解除，不因重新检测到焊缝恢复运动；后续人工恢复流程另行明确。
- 真实控制柜部署版本、HOLD 接受条件、焊机在 HOLD 时的行为、停弧接口及现场确认继续保留为外部验证条件。

## 已有离线证据

使用现有 runtime 会话入口和真实 worker／codec，替身仅提供外部机器人 transport 反馈。2026-10-26 在根工作区既有 `qt-debug` build tree 通过 MSVC 编译并运行 `--controlled-stop-contract`，四轮 RED／GREEN 覆盖：

- 明确作业停止发送现有 N 请求第七个公共整数 HOLD=1；普通检测停止不等价于设备保持。
- N 事务成功但新 D.sHold=0 时仍待确认，直接发布旧缓存 HOLD=1 不确认；后续真实 transport 事务的新 D.sHold=1 仅确认 HOLD，arc_stop_verified 始终 false。
- 停止清除待发 P，拒绝停止后的 P/O/J/C，检测请求及断开重连后的 N 继续携带 HOLD=1。
- 超时、非法 D、断开会撤销旧 HOLD 确认；保留请求锁存，后续新鲜有效 D 才可重新确认。

对应第四轮 GREEN：MSVC 构建 `20261026_201846` 成功，`20261026_202026_911056` controlled-stop-contract 退出码 0。日志在实施事项的本地 build-records 中保留，不删除失败证据。

后续第五至第八轮离线验证补足了同会话记录、停止入口和恢复隔离：

- 保留原始会话编号，依次记录 pending、实际 N/D 请求回执、hold_confirmed，之后才结束报告。D 原文以生产 codec 解码核实 sHold=1；普通收面板清理不能提前关闭待确认报告。
- HOLD 锁存后，检测启动和完整作业启动均拒绝并明确要求人工处理，不自动解除保持。
- 现有 UI Stop 仅对仍活跃的在线会话发起作业 HOLD；立即显示待确认，区分 HOLD 反馈、停弧未验证、机械零速未验证及人工恢复。迟到的普通 Idle 和收面板不覆盖锁存状态；反馈失败撤销旧确认。
- 第七轮 runtime GREEN：MSVC `20261026_210119`，controlled-stop-contract `20261026_210325_358093` 均成功。第八轮 UI GREEN：MSVC `20261026_213032`，operation-stop-ui `20261026_213114_653533` 均成功。

第九轮进一步覆盖了在途同步 O 批次：完整 runtime 的连续原图经生产检测、投影、Control 生成初始 O，外部 transport 明确阻塞首笔事务。主线程先请求作业停止，再释放首笔回包；第一笔仍如实保留 offline=4，后续 O 均未发送，并由机器人线程接续实际 N HOLD 和新反馈。MSVC `20261026_213448` 成功，controlled-stop-contract `20261026_213609_120862` exit 0。此前同一用例有效 RED 只失败于仍发送后续 O，设置和反馈前提已通过。

实现仅跨线程发布原子停止意图，协议字段、回执、普通状态及 transport 仍由机器人线程独占。机器人在线程事务边界消费意图并锁存 HOLD；已排队运动入口同样检查。已经开始的单笔同步事务不可抢占或撤回，代码没有给出现场总停止时延上限，不能把离线用例当作实时停止性能或设备机械停止证明。现场控制柜部署、HOLD 接受条件、保持时焊机行为以及独立弧状态和机械速度反馈均未验证。本程序没有自动解除 HOLD 的入口，重新检测、重连或再发送运动请求不构成操作者完成现场恢复；恢复策略需后续明确。
UI 最终补充回归：MSVC `20261026_213617` 成功，operation-stop-ui `20261026_213646_700503` exit 0，完整 UI panel 目标 `20261026_213652_068338` exit 0。四例分别验证 Completed、Error、Cleared、StoppedAfterFailures 后残留非零 sessionId 再按 Stop 不会误发 HOLD；实际活跃会话停止及保持状态持续显示仍通过。
