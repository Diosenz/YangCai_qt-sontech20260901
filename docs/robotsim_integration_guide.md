# RobotSim 首次联调最小需求

本文只写给 RobotSim 开发者，用于第一次和主项目通信联调。

本轮目标很小：先确认 RobotSim 和主项目能正常通信，并且 RobotSim 能返回 `chufengkouxiao` 工件的一帧仿真焊缝点和激光条纹点列。通信跑通后，再讨论局部点云、扫描面、face index、完整虚拟参考等后续能力。

## 本轮只支持的工件

首次联调只需要支持：

```text
workpiece_id=chufengkouxiao
robotsim_model_id=chufengkouxiao
robotsim_model_path=models/weldment/chufengkouxiao
```

其他工件暂时不用准备。

## RobotSim 需要先提供什么

请先保证以下最小能力：

1. RobotSim 启动后，主项目能知道通信端口。
2. 主项目能通过当前 ZeroMQ + JSON 通信方式连接 RobotSim。
3. RobotSim 能确认当前工件是 `chufengkouxiao`。
4. RobotSim 能返回一帧仿真焊缝点。
5. RobotSim 能返回一帧仿真激光条纹点列。
6. 返回数据要说明坐标系和单位。

如果暂时无法提供焊缝段编号，也没关系，但请明确返回“暂无焊缝段编号”，不要返回随机编号。

## 通信端口

当前主项目通过配置文件连接 RobotSim：

```text
config/digital_twin/robotsim_endpoint.txt
```

这个文件目前的 `port=0` 只是占位。联调时需要 RobotSim 提供真实端口。

推荐方式：

```text
host=127.0.0.1
port=5557
```

如果 RobotSim 仍然使用随机端口，也可以先在控制台打印：

```text
COMMServer Port: 5557
```

只要主项目联调时能拿到这个端口即可。

## 最小返回数据

RobotSim 可以使用已有接口，也可以临时新增接口。字段名不必完全照抄下面示例，但需要提供等价含义。

建议返回内容：

```json
{
  "available": true,
  "workpiece_id": "chufengkouxiao",
  "frame_id": 12,
  "coordinate_frame_3d": "world",
  "unit_3d": "mm",
  "unit_2d": "pixel",
  "seam_point_2d": [612.8, 420.6],
  "seam_point_3d": [905.2, -18.7, -253.6],
  "laser_stripe_2d": [[612.3, 420.1], [613.0, 421.0]],
  "laser_stripe_3d": [[905.1, -18.8, -253.6], [905.3, -18.6, -253.5]],
  "seam_edge_index": 3,
  "message": "ok"
}
```

字段含义：

| 字段 | 含义 |
| --- | --- |
| `available` | 本帧数据是否可用。 |
| `workpiece_id` | 本轮固定为 `chufengkouxiao`。 |
| `frame_id` | 当前帧号；没有帧号时可先返回 `0`。 |
| `coordinate_frame_3d` | 3D 点所在坐标系，建议先用 `world`。 |
| `unit_3d` | 3D 点单位，建议固定为 `mm`。 |
| `unit_2d` | 2D 点单位，固定为 `pixel`。 |
| `seam_point_2d` | 仿真焊缝点的图像坐标 `[u, v]`。 |
| `seam_point_3d` | 仿真焊缝点的 3D 坐标 `[x, y, z]`。 |
| `laser_stripe_2d` | 仿真激光条纹图像点列。 |
| `laser_stripe_3d` | 仿真激光条纹 3D 点列；如果暂时没有 3D 点，可先不返回，但请说明。 |
| `seam_edge_index` | 可选，当前焊缝段编号。 |
| `message` | 成功说明或失败原因。 |

第一次联调时，最关键的是：

- `seam_point_2d`
- `seam_point_3d`
- `laser_stripe_2d`
- `laser_stripe_3d`

主项目先用这些数据确认通信和基础数据链路正常。

## 焊缝段编号

如果 RobotSim 能返回焊缝段编号，请用 `seam_edge_index` 表示。

语义对齐旧项目 `SwLSLSensor::getSeamIndex()`：

- 正常值：当前扫描到的焊缝业务编号。
- `-1`：当前没有扫描到焊缝。
- `-2`：RobotSim 内部焊缝索引异常。

如果第一次联调暂时做不到焊缝段编号，可以这样返回：

```json
{
  "available": true,
  "workpiece_id": "chufengkouxiao",
  "seam_edge_index": -1,
  "message": "laser stripe available, seam edge index is not available yet"
}
```

不要在无法确定焊缝段时返回随机编号。

## 暂时不用做的内容

第一次联调先不用提供：

- 多工件支持。
- 局部点云。
- 扫描面 face index。
- 激光条纹逐点 face index。
- 工艺参数。
- 定位结果。
- IBVS 参数。
- 真实机器人控制。
- 缓存、历史帧或协议版本协商。

这些可以等通信功能确认正常后再继续讨论。

## 首次联调步骤

建议按这个顺序联调：

1. 启动 RobotSim。
2. 告诉主项目当前通信端口。
3. 主项目连接 RobotSim。
4. RobotSim 确认当前工件为 `chufengkouxiao`。
5. RobotSim 返回一帧仿真焊缝点和激光条纹点列。
6. 主项目检查 JSON 是否能解析。
7. 主项目检查点列是否非空。
8. 双方确认 2D 点单位为 pixel，3D 点单位为 mm。
9. 通信和数据格式确认正常后，再讨论下一步扩展。

## 最低验收标准

第一次联调只看这些结果：

- 主项目能连接 RobotSim。
- 主项目能收到合法 JSON。
- `workpiece_id` 是 `chufengkouxiao`。
- 仿真焊缝点存在。
- 仿真激光条纹点列存在且非空。
- 2D / 3D 点的单位和坐标系说得清楚。
- 如果没有焊缝段编号，RobotSim 明确说明暂时没有。

达到以上标准，就认为首次 RobotSim 通信联调通过。
