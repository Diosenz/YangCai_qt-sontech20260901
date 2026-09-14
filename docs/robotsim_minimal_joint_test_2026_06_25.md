# RobotSim 最小联调记录 2026-06-25

## 结论

本次已确认 `RobotSimWork-Johnston-win64` 是外部 RobotSim 运行包，
`TestZeroMQ` 是独立 ZeroMQ 测试客户端。主项目不需要链接或复制该运行包中的
Qt、OpenCascade、ZeroMQ 或模型依赖。

2026-06-27 全量更新后，外部包的场景和测试协议已经变化：新版 `scene.xml`
中的 `SENSOR_LSL` ID 为 `1013`，新版 `TestZeroMQ.cpp` 只查询
`GET_LSL_PHOTO` 和 `GET_WELD_SEAM_PHOTO` 两类数据，不再默认查询
`GET_LSL_POINT`。新版 `GET_LSL_PHOTO` 实际同时携带三维世界点和二维图像点；
主项目 `DigitalTwin` 已同步到这个新版最小协议。

2026-06-25 的历史记录中，通信服务可以启动并接受 `TestZeroMQ` 请求，但三类传感器
查询均返回 `null`。该历史结论保留作为旧包对照；新版包应以 2026-06-27 记录为准。

## 外部包用途

- `C:\Code\tongxinceshi\RobotSimWork-Johnston-win64\bin\RobotSim.exe`
  是 RobotSim 可执行程序。
- `C:\Code\tongxinceshi\RobotSimWork-Johnston-win64\data\scene.xml`
  是本次联调场景。
- 2026-06-27 新版 `scene.xml` 中线结构光传感器 ID 为 `1013`，工件对象 ID 为
  `1022`，焊缝编号为 `697`。
- `C:\Code\tongxinceshi\TestZeroMQ\TestZeroMQ.exe`
  是对 RobotSim ZeroMQ 服务发请求的独立客户端。

## 2026-06-27 全量更新后的协议差异

新版 `TestZeroMQ.cpp` 的单条消息为：

```json
["SENSOR_LSL_VALUE_RET", "GET_LSL_PHOTO", [1013]]
```

新版批量消息只包含两条查询：

```json
[
  ["SENSOR_LSL_VALUE_RET", "GET_LSL_PHOTO", [1013]],
  ["SENSOR_LSL_VALUE_RET", "GET_WELD_SEAM_PHOTO", [1013]]
]
```

这意味着当前最小闭环应先拿到线结构光图像数据：

- `GET_LSL_PHOTO`：每个离散激光点包含世界坐标 `[x_mm, y_mm, z_mm]`
  和图像坐标 `[x_px, y_px]`；
- `GET_WELD_SEAM_PHOTO`：图像上的焊缝点。

`GET_LSL_POINT` 在新版测试客户端中已不再发送。主项目通过 `GET_LSL_PHOTO`
填充 `laserLinePoints_mm` 和 `laserLineImagePoints_px`。

## 2026-06-27 新版包实际启动与 probe 结果

用户将外部包目录从中文路径 `C:\Code\通信测试` 改为英文路径
`C:\Code\tongxinceshi` 后，RobotSim 可以正常打开新版场景。截图和模型树已确认
当前场景中存在 `Sensor LSL` 和 `260626-FengJiKeChuKou.step`。

按新版运行包启动：

```powershell
Start-Process `
  -FilePath 'C:\Code\tongxinceshi\RobotSimWork-Johnston-win64\bin\RobotSim.exe' `
  -WorkingDirectory 'C:\Code\tongxinceshi\RobotSimWork-Johnston-win64\bin'
```

21:04 复测结果：

- RobotSim 进程可启动，进程响应正常；
- 主窗口和控制台窗口均出现；
- 控制台打印 `COMMServer Port: 49643`；
- 使用本项目 `i2r_rlc_robotsim_probe.exe` 可连接该端口并收到回包；
- 使用新版 `TestZeroMQ.exe` 输入端口 `49643`、传感器 `1013`，单条查询 `1`
  和批量查询 `2` 均可收到非空回包；
- 单条 `GET_LSL_PHOTO` 返回 1 条激光线，共 29 个离散点；
- 批量 `GET_LSL_PHOTO + GET_WELD_SEAM_PHOTO` 返回 29 个激光点和 1 个焊缝图像点。

本次 probe 请求与结果如下：

```text
已连接 RobotSim：tcp://127.0.0.1:49643
RobotSim 响应成功。
[[[[143.30108362940433,6.74008888513505,95.08617650660791],[812,904]], ... ]]
```

```text
已连接 RobotSim：tcp://127.0.0.1:49643
RobotSim 响应成功。
[[[[[143.30108362940433,6.74008888513505,95.08617650660791],[812,904]], ... ]],[[1034,374]]]
```

新版 `TestZeroMQ.exe` 复测单条和批量查询同样成功。单条查询：

```text
发送请求: ["SENSOR_LSL_VALUE_RET","GET_LSL_PHOTO",[1013]]
收到响应: [[[[143.30108362940433,6.74008888513505,95.08617650660791],[812,904]], ... ]]
```

批量查询：

```text
发送请求: [["SENSOR_LSL_VALUE_RET","GET_LSL_PHOTO",[1013]],["SENSOR_LSL_VALUE_RET","GET_WELD_SEAM_PHOTO",[1013]]]
收到响应: [[[[[143.30108362940433,6.74008888513505,95.08617650660791],[812,904]], ... ]],[[1034,374]]]
```

因此当前结论是：通信服务已启动，数组协议可达，场景加载成功后 `SensorLSL ID=1013`
可以返回有效仿真数据。此前 `[null,null]` 的主要原因不是协议错误，而是中文路径下
场景没有实际加载进 RobotSim 运行态。

## 已验证的启动和端口

直接通过自动化启动 exe 时，RobotSim 会因为工作目录不对而退出。可用启动方式是以
`bin` 为工作目录启动。当前建议使用英文路径，避免中文路径导致场景打开失败：

```powershell
Start-Process `
  -FilePath 'C:\Code\tongxinceshi\RobotSimWork-Johnston-win64\bin\RobotSim.exe' `
  -WorkingDirectory 'C:\Code\tongxinceshi\RobotSimWork-Johnston-win64\bin'
```

启动后控制台会打印端口，例如本次启动得到过：

```text
COMMServer Port: 49934
COMMServer Port: 49965
COMMServer Port: 65464
COMMServer Port: 49443
COMMServer Port: 49643
```

尝试用场景路径作为命令行参数启动：

```powershell
RobotSim.exe C:\Code\tongxinceshi\RobotSimWork-Johnston-win64\data\scene.xml
```

仍未让通信查询返回有效传感器数据，因此当前不能认为 RobotSim 支持命令行自动加载场景。

## 2026-06-25 TestZeroMQ 历史协议

`TestZeroMQ.cpp` 使用 ZeroMQ `DEALER` socket 连接 RobotSim 的 `ROUTER` 服务。
发送帧格式为：

1. 空分隔帧；
2. JSON 数据帧。

2026-06-25 晚间更新后，单条消息不再使用 object 字段，而是使用三元数组：

```json
["SENSOR_LSL_VALUE_RET", "GET_LSL_POINT", [1001]]
```

一次性批量查询三类数据时，消息根节点是三元数组的数组：

```json
[
  ["SENSOR_LSL_VALUE_RET", "GET_LSL_POINT", [1001]],
  ["SENSOR_LSL_VALUE_RET", "GET_LSL_PHOTO", [1001]],
  ["SENSOR_LSL_VALUE_RET", "GET_WELD_SEAM_PHOTO", [1001]]
]
```

主项目 `DigitalTwin` 已改为发送上述批量消息，并按返回数组的 0/1/2 槽位解析三类数据。

更新前，Codex 使用端口 `49934` 和 `49965` 分别测试旧三类 object 请求，返回均为：

```text
收到响应: null
```

更新后，Codex 使用端口 `65464` 测试新版批量请求，返回为：

```text
发送请求: [["SENSOR_LSL_VALUE_RET","GET_LSL_POINT",[1001]],["SENSOR_LSL_VALUE_RET","GET_LSL_PHOTO",[1001]],["SENSOR_LSL_VALUE_RET","GET_WELD_SEAM_PHOTO",[1001]]]
收到响应: [null,null,null]
```

为避免 `TestZeroMQ` 只能发送固定查询，本仓库新增了手工联调用探针：

```text
out\build\qt-debug\tests\i2r_rlc_robotsim_probe.exe
```

该探针不是自动测试，只用于对正在运行的外部 RobotSim 发送任意 JSON。使用端口
`49443` 复测三类传感器批量查询，结果仍为：

```text
已连接 RobotSim：tcp://127.0.0.1:49443
RobotSim 响应成功。
[null,null,null]
```

## 场景加载接口探测

仓库内 RobotSim 参考源码显示 OP 层存在场景通知：

- `SCENE_VOID / SCENE_LOAD_FROM_FILE`
- `SCENE_VALUE_RET / GET_SCENE_FILE_PATH`

但当前外部更新 exe 的 ZeroMQ 层没有验证出可用的场景查询回包。使用探针发送：

```json
["SCENE_VALUE_RET", "GET_SCENE_FILE_PATH", []]
```

结果为等待响应超时。继续发送无返回的清空并加载场景命令：

```json
[
  ["SCENE_VOID", "SCENE_CLEAR", []],
  [
    "SCENE_VOID",
    "SCENE_LOAD_FROM_FILE",
    ["C:\\Code\\通信测试\\RobotSimWork-Johnston-win64\\data\\scene.xml"]
  ]
]
```

探针只能确认请求已发送；等待 2 秒后再次查询三类传感器，仍返回：

```text
[null,null,null]
```

因此当前不能认为外部 RobotSim 已经通过 ZeroMQ 暴露了可用的场景加载接口。继续推进时，
应优先由 RobotSim 主窗口稳定完成“打开场景”，或请 RobotSim 开发者明确提供可自动化
加载场景的通信命令。

## 主项目侧适配状态

`DigitalTwin` 已按 2026-06-27 新版批量协议适配最小虚拟参考查询：

- `GET_LSL_PHOTO` 同时转为世界系激光点 `laserLinePoints_mm`
  和图像系激光线点 `laserLineImagePoints_px`；
- `GET_WELD_SEAM_PHOTO` 转为可选焊缝图像点 `seamPointImage_px`；
- `DigitalTwinVirtualReferenceRequest::sensorId` 默认值为 `1013`；
- RobotSim 常规命令已从旧 `{op_mode, op_type, param}` 对象格式改为
  新版 `[mode, type, params]` 数组格式；
- RobotSim 返回 `null` 时，主项目会报告不可用，但保留“通信成功、数据为空”的诊断信息。

本次没有接 UI，也没有接完整 Localization/IBVS 在线流程。

## 后续人工联调检查点

下一次需要由 RobotSim 主窗口明确执行“打开场景”，选择：

```text
C:\Code\tongxinceshi\RobotSimWork-Johnston-win64\data\scene.xml
```

加载成功后再使用 `TestZeroMQ` 输入：

```text
Port: <RobotSim 控制台端口>
SensorLSL ID: 1013
查询类型: 1 / 2
```

若场景确实加载成功，但批量请求仍返回 `null`、`[null,null]` 或空数组，需要 RobotSim 开发者确认：

- 是否已经支持通过 ZeroMQ 加载场景；若支持，准确的 op_mode、op_type、参数和是否回包是什么；
- 通信服务对象表是否同步了场景中的传感器 `1013`；
- `SENSOR_LSL_VALUE_RET` 两个图像命令是否只在特定仿真状态下返回数据；
- 线结构光传感器是否需要先触发一次渲染、扫描或刷新；
- 批量返回是否固定按 `GET_LSL_PHOTO`、`GET_WELD_SEAM_PHOTO` 的请求顺序组织；
- 两个查询的成功响应具体 JSON 形态；
- 后续是否还会恢复 `GET_LSL_POINT` 三维激光线点接口。

## 2026-06-29 实连复测与模块接入进展

本轮在 `Twin` 分支合并冲突解决后，使用新版路径：

```text
C:\Code\tongxinceshi\RobotSimWork-Johnston-win64\bin\RobotSim.exe
```

RobotSim 进程仍在运行，进程 PID 为 `11972`，通过 `netstat` 查到当前监听端口：

```text
127.0.0.1:51900
```

使用主项目探针发送单条线结构光图像查询：

```powershell
$json='["SENSOR_LSL_VALUE_RET","GET_LSL_PHOTO",[1013]]'
.\out\build\qt-debug\tests\i2r_rlc_robotsim_probe.exe 51900 $json
```

结果：

- 连接成功；
- RobotSim 响应成功；
- 返回 `29` 个激光点；
- 每个点格式为 `[[x_mm, y_mm, z_mm], [u_px, v_px]]`；
- 第一个点示例为 `[[143.301..., 6.740..., 95.086...], [812, 904]]`。

继续发送批量查询：

```powershell
$json='[["SENSOR_LSL_VALUE_RET","GET_LSL_PHOTO",[1013]],["SENSOR_LSL_VALUE_RET","GET_WELD_SEAM_PHOTO",[1013]]]'
.\out\build\qt-debug\tests\i2r_rlc_robotsim_probe.exe 51900 $json
```

结果：

- 连接成功；
- RobotSim 响应成功；
- 第一项返回 `29` 个激光点；
- 第二项返回焊缝图像点 `[[1034, 374]]`。

这说明当前不是 `[null,null]`，也不是空数组，而是 RobotSim 场景和传感器已经能返回可用数据。
截至本轮，主项目侧进一步完成了以下最小业务接入：

- UI 的 `仿真联调` 成功后会缓存最近一次 RobotSim 虚拟参考结果；
- 缓存结果会转换为 `LocalizationVirtualReferenceSnapshot`，其中三维激光点进入
  `laserLinePoints_mm`；
- 全局定位 dry-run 若已有 RobotSim 三维点，会优先使用该虚拟参考，而不是继续走
  `NullLocalizationVirtualReferenceProvider`；
- 缓存结果会转换为 `ibvs::IbvsLaserPrior`，其中二维激光线进入 `virtualLaserLine`，
  焊缝图像点进入 `virtualWeldPoint`；
- IBVS dry-run/单步若已有 RobotSim 二维先验，会优先使用该先验生成 LVS 观测；
- 当前仍不接真实机器人运动，不把结果自动下发到 `Control` 或 `RobotComm`。

待 VS2022 重新生成后，需要在主程序 UI 中补做一次人工验收：

1. 点击 `仿真联调`，输入端口 `51900` 或 RobotSim 控制台当前端口。
2. 确认状态区显示激光线三维点和二维点数量。
3. 点击全局定位，确认日志出现“使用 RobotSim 虚拟参考”。
4. 启动并单步执行 IBVS dry-run，确认日志出现“使用 RobotSim LVS 先验”。
