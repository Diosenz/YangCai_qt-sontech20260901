# TCP 控制线程记录阻塞：本轮交付

已完成首阶段修复，主程序为 `out/build/release/UI/i2r_rlc.exe`。本轮只隔离记录与控制时效，不改变PID、跟随参数、80/60um限值或检测策略。未修改Vision/Control；保留任务开始前已有的未提交改动，完整对照见baseline/和change.diff。

## 修改

- 新增COnlineCorrectionTextWriter：每会话独占可回收后台线程，队列上限8 MiB/4096块，含在途块。文件回调不占队列锁。
- recorder运行期文本追加全部非阻塞提交，时间与来源在调用线程冻结。正常结束排空已接纳数据，后台失败/队列满锁存错误，拒绝后续数据；摘要保存完整性、拒绝块数和错误。
- 文件追加与表头检查QFile::flush；终态文件系统异常转换为返回失败。最终摘要在排空后由所有者线程追加，注入回调不得依赖固定线程。
- 公共测试边界为真实在线worker输入到P信号，以及RobotComm公开命令入口到模拟transport。测试使用文件系统可控慢写；未连接真实设备。
- 更新docs/online_correction_session_replay.md的线程、完整性及崩溃丢失边界。

## 失败到通过的证据

生产入口：`out/build/qt-debug/tests/i2r_rlc_runtime_tracking_flow_tests.exe --slow-recording-control-contract`，使用Qt/OpenCV/vcpkg运行库PATH。

修复前：`FAIL: P must be published while the filesystem is still blocked, before releasing the write gate`，见red-control.log。修复后同一入口通过，另增加慢写期间20mm→30mm新反馈对应的新P，避免只证明首条命令先发出。

终态失败入口：`out/build/qt-debug/tests/i2r_rlc_online_correction_run_recorder_tests.exe --text-writer-contract`。先发现文件系统异常逃出finishSession，见red-finalization.log；修正后通过。最终同时覆盖独立字节/条数上限、在途容量、顺序、幂等stop、刷新失败报告、摘要不宣称完整、新会话恢复。刷新失败由外部文件边界报告，未真实制造Windows磁盘满或Qt内部flush故障。

## 最终验证

- 加载D:/vs2022/Common7/Tools/VsDevCmd.bat，复用既有out/build/qt-debug和out/build/release；未新建完整build tree。
- 最终CTest：8/8通过，78.61秒，见ctest-final.log。涵盖完整runtime flow、normal session replay、calibration、fusion controls、recorder、新增慢写/背压/文本合同。
- 最终额外3项：session-lifecycle、controlled-stop、startup-replay-capture，全部通过，见lifecycle-final-results.json。
- Release目标i2r_rlc构建成功，见build-final-release.log。
- 主程序关闭自动设备连接、Qt offscreen启动3秒存活，再只终止本次启动的进程；见startup-smoke-final.json。
- Standards与Spec各发现1项，均修正后复核关闭，最终0/0，详见review.md。
- 无本轮新增DEBUG埋点；既有KCF诊断标记未删除。无文件删除、无鼠标操作；用户要求命令行后未再使用Computer Use。

## 实测边界与下一步

历史数据确认的13.590mm TCP偏移不会因代码变化而在归档中消失。软件回归证明慢记录已不再阻塞P发布和新反馈处理，尚不证明现场偏差已消除。原回执的生成至记录跨度也不能直接等同网络或控制柜执行时延。

请在同一工件摆放、相同速度和参考口径下复测，结束后正常停止/复位并等待记录排空。重点比较中段TCP峰值、起步/预焊段，以及新会话最终text_recording_complete是否为1。取得新会话后再评估剩余目标线残差与实际发送节奏，并决定是否需要调整控制律或参数。正常结束可能等待慢盘；进程崩溃/强杀仍可能丢失有界未落盘队列。

主程序SHA256：`1509a5170f347e438b95469b1a2c2a4dbc04c0b1e43c7e78c6c25f829b7667c4`。
