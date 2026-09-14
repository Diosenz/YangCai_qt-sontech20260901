# RobotSim 数字孪生对接 API 需求

## 背景和结论

主项目 `i2r_rlc` 已经实现 `DigitalTwin` facade，用于连接外部 `RobotSim` 进程。
当前不会把 `RobotSim/` 源码、OpenCascade、VTK 或内部模型对象直接编进主项目。

现在主项目能按 RobotSim 现有 ZeroMQ + JSON 协议发送基础对象/机器人命令，但要真正支撑
定位、局部定位和视觉仿真，还需要 RobotSim 开发者补齐对外 API。

## 当前 RobotSim 已有能力

从当前 RobotSim 源码看，通信服务位于：

- `RobotSim/RobotSimCOMM/include/COMMServer.h`
- `RobotSim/RobotSimCOMM/src/COMMServer.cpp`
- `RobotSim/RobotSimCOMM/src/COMMNotifFactory.cpp`

当前通信形态：

- ZeroMQ `ROUTER` server；
- client 侧按 `DEALER` 发送 multipart；
- JSON 字段为 `op_mode`、`op_type`、`param`；
- `_VALUE_RET` 类通知会通过 `whenReady()` 回传 JSON；
- server 目前绑定 `tcp://127.0.0.1:0`，端口随机，只打印 `COMMServer Port: <port>`。

当前工厂已注册的主要命令包括：

- `BASE_OBJECT_VALUE_RET / GET_NAME`
- `BASE_OBJECT_VALUE_RET / GET_ABS_POSE`
- `BASE_OBJECT_VALUE_RET / GET_TYPE_BASE_OBJECT`
- `BASE_OBJECT_VOID / SET_NAME`
- `BASE_OBJECT_VOID / SET_PARENT`
- `BASE_OBJECT_VOID / SET_POSE`
- `ROBOT_VOID / SET_Q_VEC`
- `ROBOT_VOID / SET_POSE_FRAME_TCP`
- `COLLISION_DETECTOR_VALUE_RET / GET_DETECT_PAIR`

RobotSim 内部也已有线结构光传感器能力：

- `RobotSim/RobotSimMD/include/CToolLSLSensor.h`
- `RobotSim/RobotSimMD/src/CToolLSLSensor.cpp`
- `CToolLSLSensor::getLSLPhoto()` 可生成仿真激光线成像像素点；
- `getLaserPlaneParaList()` 和 `getCameraIntrPara()` 已有对应 operator 类型，但未在通信工厂注册。

## 主项目当前已经按以下协议适配

主项目当前发送的 JSON 形态：

```json
{
  "op_mode": "BASE_OBJECT_VALUE_RET",
  "op_type": "GET_ABS_POSE",
  "param": [1004]
}
```

位姿数组采用：

```json
[x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
```

机器人关节数组采用：

```json
[q1, q2, q3, q4, q5, q6]
```

当前主项目可直接消费的响应：

- `GET_ABS_POSE` 返回 6 元数组；
- 无返回值命令只要求 RobotSim 成功接收，不需要响应。

## 必须补齐的 RobotSim API

### 1. 端口发现或固定端口

当前随机端口只能从控制台复制，不适合程序自动连接。建议二选一：

- 支持固定端口配置，例如 `tcp://127.0.0.1:5557`；
- 或启动后写出端口文件，例如 `output/robotsim_comm_endpoint.txt`。

建议端口文件格式：

```text
host=127.0.0.1
port=5557
protocol=zeromq-router-dealer-json
```

### 2. 注册线结构光传感器通信命令

当前已有 `OPNotifToolLSLSensorValueRet.h` / `OPNotifToolLSLSensorVoid.h`，但
`COMMNotifFactory` 未注册。建议至少注册：

- `TOOL_LSL_SENSOR_VALUE_RET / GET_CAMERA_INTR_PARA`
- `TOOL_LSL_SENSOR_VALUE_RET / GET_LASER_PLANE_PARA_LIST`
- `TOOL_LSL_SENSOR_VOID / SET_CAMERA_INTR_PARA`

请求示例：

```json
{
  "op_mode": "TOOL_LSL_SENSOR_VALUE_RET",
  "op_type": "GET_CAMERA_INTR_PARA",
  "param": [1001]
}
```

### 3. 暴露虚拟激光线成像结果

需要把 `CToolLSLSensor::getLSLPhoto()` 暴露为通信命令。

建议命令：

- `TOOL_LSL_SENSOR_VALUE_RET / GET_LSL_PHOTO`

请求：

```json
{
  "op_mode": "TOOL_LSL_SENSOR_VALUE_RET",
  "op_type": "GET_LSL_PHOTO",
  "param": [1001]
}
```

建议响应：

```json
{
  "sensor_id": 1001,
  "frame_id": 123,
  "lines": [
    {
      "line_index": 0,
      "pixels": [[u0, v0], [u1, v1]]
    }
  ]
}
```

如果暂时无法提供对象响应，也可以先返回当前内部数据的数组形式：

```json
[
  [[u0, v0], [u1, v1]],
  [[u0, v0], [u1, v1]]
]
```

### 4. 暴露世界坐标下的虚拟激光线点集

定位模块最终更需要 3D 点，而不是纯像素点。建议新增命令返回世界坐标或传感器坐标下的
激光线点集。

建议命令：

- `TOOL_LSL_SENSOR_VALUE_RET / GET_LSL_POINT_LIST`

建议响应：

```json
{
  "sensor_id": 1001,
  "workpiece_id": 1004,
  "coordinate_frame": "world",
  "unit": "mm",
  "lines": [
    {
      "line_index": 0,
      "points_mm": [[x0, y0, z0], [x1, y1, z1]]
    }
  ]
}
```

### 5. 暴露虚拟工件点云或采样点

全局定位需要工件虚拟参考点云。建议 RobotSim 提供从 `CShape` 或工件模型采样的点集导出。

建议命令：

- `SHAPE_VALUE_RET / GET_SAMPLED_POINT_CLOUD`

请求：

```json
{
  "op_mode": "SHAPE_VALUE_RET",
  "op_type": "GET_SAMPLED_POINT_CLOUD",
  "param": [1004, 2.0]
}
```

其中第二个参数可表示采样间隔或精度，单位 mm。

建议响应：

```json
{
  "shape_id": 1004,
  "coordinate_frame": "world",
  "unit": "mm",
  "points_mm": [[x0, y0, z0], [x1, y1, z1]]
}
```

## 错误响应建议

建议所有有返回值命令在失败时返回结构化错误，而不是 `null`：

```json
{
  "ok": false,
  "error_code": "OBJECT_NOT_FOUND",
  "message": "LSL sensor 1001 not found"
}
```

成功时建议统一返回：

```json
{
  "ok": true,
  "data": {}
}
```

如果短期不想改现有返回格式，也请至少保证：

- 对象不存在时返回 `null`；
- JSON 语法始终合法；
- 大数组响应不要混入控制台日志文本。

## 优先级

1. 固定端口或端口文件。
2. 注册已有 `TOOL_LSL_SENSOR_*` 配置查询命令。
3. 暴露 `GET_LSL_PHOTO`。
4. 暴露 `GET_LSL_POINT_LIST`，返回世界坐标 mm 点集。
5. 暴露工件模型采样点云。

完成 1-3 后，主项目可以做基础通信联调和虚拟视觉回放；完成 4-5 后，主项目的
Localization 才能真正消费 RobotSim 虚拟参考数据。
