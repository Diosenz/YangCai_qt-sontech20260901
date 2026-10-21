# RobotSim 仿真信息在主项目中的使用说明

本文面向当前 Qt 主项目所有模块开发者，说明如何连接 RobotSim、能拿到什么仿真数据、各模块如何通过统一接口消费仿真信息，以及如何向 RobotSim 发送写命令。

当前已实连验证的场景使用 `SensorLSL ID = 1013`。RobotSim 由外部进程启动，主项目只连接它的 ZeroMQ 端口，不负责启动 RobotSim、不加载 RobotSim 源码、不引入 OpenCascade/VTK 等仿真依赖。

## 1. 总体数据流

```text
RobotSim.exe
  -> ZeroMQ JSON
  -> DigitalTwin/RobotSimDigitalTwinService
  -> DigitalTwinVirtualReferenceResult
  -> 各模块自己的输入 DTO 或适配器
      -> LocalizationVirtualReferenceSnapshot
      -> ibvs::IbvsLaserPrior / ibvs::IbvsLvsObservation
      -> Vision / Control / UI / 测试代码自定义输入结构
```

模块职责：

- `DigitalTwin`：负责连接 RobotSim、发送 JSON、解析返回值。
- `Localization`：已落地消费三维虚拟参考点。
- `IBVS`：已落地消费二维激光线和焊缝图像点。
- `Vision`、`Control`、`Camera`、`RobotComm`、测试代码等其他模块：如果需要仿真信息，应优先消费 `DigitalTwinVirtualReferenceResult` 或由上层转换后的本模块 DTO，不直接依赖 RobotSim 内部源码。
- `UI`：当前负责把一次 RobotSim 查询结果缓存下来，并演示分发给定位和 IBVS dry-run；其他模块不应为了拿仿真数据反向依赖 UI。

### MH12 TCP 同步的工具定义（2026-10-21）

当前 MH12 解析逆解以 RobotSim `test31` 的 TCP1011 为准，不再使用迁移时的
旧工具标定。工具在软件法兰中的位置为 `(-125.8, 4.7, 490.2) mm`，固定轴
`Rz * Ry * Rx` 角为 `(-174.8, -145.2, 169.2) deg`；Link6 到软件法兰的
100 mm 偏置仍独立保留。

同步链为 `真实 TCP * inverse(场景工具) -> 软件法兰 -> MH12 关节`。
RobotSim 按同一场景工具正解后，应恢复输入 TCP；该约束影响所有复用
`CMotomanMh12InverseKinematics` 的全局定位、在线/离线纠偏及实时虚拟参考。
不通过移动工件或修改真实相机标定来补偿工具定义差异。

`i2r_rlc_mh12_inverse_kinematics_tests` 使用独立场景 MDH 正运动学，检查多姿态
TCP 位置/旋转闭环、无效输入恢复和分支复位。测试不连接机器人或 RobotSim。
此适配针对当前场景工具，不会自动读取外部 XML；以后更换工具定义时，必须同步
核对逆解参数及正向闭环。历史采样仍按当时工具链解释，不能据此自动重写旧归档。

## 2. 所有模块统一接入原则

任何模块想用 RobotSim 仿真信息，都优先遵守下面三条：

1. 需要原始仿真数据时，依赖 `DigitalTwin` 的公共接口和值对象：

```cpp
#include "IDigitalTwinService.h"
```

2. 模块内部算法不要直接创建或持有 RobotSim socket。推荐由 UI、应用层或流程编排层查询 RobotSim，再把结果转换成本模块自己的输入结构。

3. 如果某个模块确实是流程编排层，需要主动查询 RobotSim，可以依赖实现类：

```cpp
#include "RobotSimDigitalTwinService.h"
```

推荐的模块函数形态：

```cpp
void consumeSimulationReference(
    const i2r_rlc::digital_twin::DigitalTwinVirtualReferenceResult& reference);
```

或者先转换成本模块自己的 DTO：

```cpp
MyModuleSimulationInput input =
    makeMyModuleSimulationInput(digitalTwinResult);
myModule.run(input);
```

不要在普通算法函数里这样做：

```cpp
// 不推荐：算法内部直接连接 RobotSim，会把通信、超时和测试都耦合进去。
i2r_rlc::digital_twin::RobotSimDigitalTwinService service;
service.start(endpoint);
```

## 3. DigitalTwin：直接连接 RobotSim

需要包含：

```cpp
#include "RobotSimDigitalTwinService.h"
```

如果只依赖接口和值对象，包含：

```cpp
#include "IDigitalTwinService.h"
```

连接 RobotSim：

```cpp
i2r_rlc::digital_twin::RobotSimEndpointConfig endpoint;
endpoint.host = QStringLiteral("127.0.0.1");
endpoint.port = 51900; // RobotSim 控制台 COMMServer Port
endpoint.connectTimeoutMs = 1000;
endpoint.requestTimeoutMs = 1000;

i2r_rlc::digital_twin::RobotSimDigitalTwinService service;
const auto start = service.start(endpoint);
if (start.state != i2r_rlc::digital_twin::DigitalTwinServiceState::Idle) {
    // 连接失败，查看 start.message
}
```

请求仿真参考数据：

```cpp
i2r_rlc::digital_twin::DigitalTwinVirtualReferenceRequest request;
request.frameId = frameId;
request.sensorId = 1013;

const auto result = service.requestVirtualReference(request);
if (!result.available) {
    // 无可用数据，查看 result.message
}
```

返回结构：

```cpp
result.laserLinePoints_mm;        // 三维激光点，单位 mm
result.laserLineImagePoints_px;   // 图像激光点，单位 pixel
result.hasSeamPointImage_px;      // 是否有焊缝图像点
result.seamPointImage_px;         // 焊缝图像点，单位 pixel
```

当前实测数据：

- `GET_LSL_PHOTO` 返回 29 个激光点；
- 每个点格式为 `[[x_mm, y_mm, z_mm], [u_px, v_px]]`；
- `GET_WELD_SEAM_PHOTO` 返回焊缝图像点，例如 `(1034, 374)`。

## 4. 各模块能直接拿到的数据结构

`DigitalTwinVirtualReferenceResult` 是当前所有模块共享的仿真参考结果：

```cpp
struct DigitalTwinVirtualReferenceResult {
    bool available;
    bool supported;
    qint64 frameId;
    QString workpieceId;
    DigitalTwinPose6D referencePose;
    bool hasSeamPointImage_px;
    DigitalTwinImagePoint2D_px seamPointImage_px;
    std::vector<DigitalTwinImagePoint2D_px> laserLineImagePoints_px;
    std::vector<DigitalTwinPoint3D_mm> laserLinePoints_mm;
    std::vector<DigitalTwinPoint3D_mm> localPointCloud_mm;
    QString message;
};
```

字段含义：

| 字段 | 含义 | 当前状态 |
| --- | --- | --- |
| `laserLinePoints_mm` | 线结构光三维点，单位 mm | 已实连验证，当前场景 29 点 |
| `laserLineImagePoints_px` | 线结构光图像点，单位 pixel | 已实连验证，当前场景 29 点 |
| `seamPointImage_px` | 焊缝图像点，单位 pixel | 已实连验证，当前场景 `(1034, 374)` |
| `localPointCloud_mm` | 局部虚拟点云，单位 mm | 公共字段已预留，当前 RobotSim 查询未返回 |
| `referencePose` | 参考位姿 | 公共字段已预留，当前线结构光查询不依赖它 |
| `message` | 诊断信息 | 用于日志和失败原因 |

不同模块可以按需使用：

- `Localization`：优先使用 `laserLinePoints_mm` 或未来的 `localPointCloud_mm`；
- `IBVS`：使用 `laserLineImagePoints_px` 和 `seamPointImage_px`；
- `Vision`：可使用二维激光线和焊缝点作为仿真先验、模板初始化或检测对照；
- `Control`：可使用仿真参考结果做 dry-run 流程判断，但不要直接把它当真实机器人反馈；
- `UI`：可显示点数、焊缝点、连接状态和错误信息；
- 测试代码：可构造该结果作为 fake 输入，不需要启动 RobotSim。

## 5. Localization：消费三维虚拟参考示例

定位模块自身不应直接创建 RobotSim 连接。推荐由 UI 或流程编排层先查询 RobotSim，再把快照传给定位。

需要包含：

```cpp
#include "LocalizationApplicationService.h"
#include "LocalizationTypes.h"
```

使用已经查询到的 RobotSim 三维点构造快照：

```cpp
i2r_rlc::localization::LocalizationVirtualReferenceSnapshot snapshot;
snapshot.available = true;
snapshot.referenceId = QStringLiteral("robotsim-sensor-1013");
snapshot.message = QStringLiteral("RobotSim virtual reference");
snapshot.laserLinePoints_mm = laserLinePointsFromRobotSim;
```

运行全局定位 dry-run：

```cpp
i2r_rlc::localization::LocalizationGlobalDryRunRequest request;
request.frameId = frameId;

i2r_rlc::localization::LocalizationApplicationService localizationService;
const auto result = localizationService.runGlobalIcpDryRun(request, snapshot);
```

注意：

- `LocalizationGlobalIcpPipeline` 会优先使用 `snapshot.workpiecePointCloud_mm`；
- 如果没有整件点云，也可以使用 `snapshot.laserLinePoints_mm`；
- 当前 RobotSim 已能提供三维激光线点，所以可以形成最小定位闭环；
- 如果没有 RobotSim 数据，仍可调用 `runGlobalIcpDryRun(request)`，它会走空 provider 并返回明确的缺少虚拟参考状态。

如果需要直接使用 DigitalTwin 到 Localization 的适配器，可以包含：

```cpp
#include "DigitalTwinLocalizationVirtualReferenceProvider.h"
```

示例：

```cpp
i2r_rlc::digital_twin::DigitalTwinLocalizationVirtualReferenceProvider provider(service);

i2r_rlc::localization::LocalizationVirtualReferenceRequest request;
request.frameId = frameId;
request.referenceId = QStringLiteral("robotsim-sensor-1013");
request.needLaserLinePoints = true;

const auto snapshot = provider.requestSnapshot(request);
```

## 6. IBVS：消费二维激光线和焊缝点示例

IBVS 模块不直接依赖 RobotSim。它消费模块自己的先验结构：

```cpp
#include "ibvs/IbvsLaserLineObservation.h"
#include "ibvs/IbvsSensorFusion.h"
```

核心结构：

```cpp
i2r_rlc::ibvs::IbvsLaserPrior prior;
prior.frameIndex = static_cast<int>(frameId);
prior.virtualLaserLine = laserLineImagePointsFromRobotSim;
prior.hasVirtualWeldPoint = true;
prior.virtualWeldPoint = seamPointFromRobotSim;
```

其中：

- `virtualLaserLine` 来自 `DigitalTwinVirtualReferenceResult::laserLineImagePoints_px`；
- `virtualWeldPoint` 来自 `DigitalTwinVirtualReferenceResult::seamPointImage_px`；
- 单位均为 pixel。

UI 层当前已经提供转换工具：

```cpp
#include "RobotSimVirtualReferenceCache.h"
```

示例：

```cpp
const auto cache = i2r_rlc::ui::makeRobotSimVirtualReferenceCache(
    digitalTwinResult,
    frameId,
    1013);

if (cache.hasIbvsLaserPrior) {
    const auto& prior = cache.ibvsLaserPrior;
}

if (cache.ibvsLvsObservation.has_value()) {
    input.lvsObservation = cache.ibvsLvsObservation;
}
```

边界建议：

- UI/流程编排层可以包含 `RobotSimVirtualReferenceCache.h`；
- `IBVS` 模块内部不要反向包含 UI 头文件；
- 如果后续需要在非 UI 层复用转换逻辑，应把转换器下沉到独立适配层，而不是让 IBVS 依赖 UI。

## 7. 其他模块如何使用

### 7.1 Vision 使用二维仿真先验

如果视觉模块需要仿真激光线或焊缝点，不建议直接连接 RobotSim。推荐上层传入：

```cpp
const auto& imageLine = reference.laserLineImagePoints_px;
if (reference.hasSeamPointImage_px) {
    const auto seamPoint = reference.seamPointImage_px;
}
```

Vision 内部应转换成自己的图像点结构，例如：

```cpp
std::vector<MyVisionPoint> points;
for (const auto& point : reference.laserLineImagePoints_px) {
    points.push_back({point.x_px, point.y_px});
}
```

适合用途：

- KCF/STC/模板跟踪初始化；
- Gabor 或中心线算法的方向先验；
- 离线回放和仿真对照；
- 检测算法调试可视化。

### 7.2 Control 使用仿真信息

`Control` 如果只是判断流程状态，应消费上层已经转换好的业务状态，例如“仿真参考是否可用、点数多少、是否有焊缝点”。不建议 `Control` 直接解析 RobotSim JSON。

示例：

```cpp
struct SimulationReferenceSummary {
    bool available = false;
    int laserLine3dPointCount = 0;
    int laserLine2dPointCount = 0;
    bool hasSeamPoint = false;
};
```

注意：

- 当前 RobotSim 信息适合 dry-run、演示和算法输入；
- 不要把仿真结果直接当作真实机器人安全反馈；
- 真正下发机器人运动仍应走 `RobotComm` 和安全门控。

### 7.3 Camera 使用仿真信息

`Camera` 模块通常不需要 RobotSim 数据。若后续做“虚拟相机帧”或“仿真图像源”，建议新增明确的相机设备实现或离线设备实现，而不是让真实相机 SDK 类依赖 DigitalTwin。

推荐边界：

```text
RobotSim/DigitalTwin -> 虚拟图像或点数据 -> Camera-like adapter -> 上层流程
```

### 7.4 RobotComm 使用仿真信息

`RobotComm` 负责真实机器人通信边界。RobotSim 写命令目前属于 `DigitalTwin` facade，不建议混进真实机器人协议类。

如果需要仿真机器人状态，应先明确是：

- 真实机器人状态的模拟输入；
- RobotSim 场景中的机器人对象操作；
- 还是对真实控制柜的命令替代。

这三种语义不能混用。

### 7.5 测试代码使用 fake 数据

单元测试不需要启动 RobotSim，可以直接构造：

```cpp
i2r_rlc::digital_twin::DigitalTwinVirtualReferenceResult result;
result.available = true;
result.laserLinePoints_mm = {
    {143.0, 6.0, 95.0},
    {144.0, 7.0, 96.0}
};
result.laserLineImagePoints_px = {
    {812.0, 904.0},
    {823.0, 874.0}
};
result.hasSeamPointImage_px = true;
result.seamPointImage_px = {1034.0, 374.0};
```

当前已有测试：

```text
i2r_rlc_digital_twin_tests.exe
i2r_rlc_robot_sim_virtual_reference_cache_tests.exe
i2r_rlc_localization_tests.exe
i2r_rlc_ibvs_online_service_tests.exe
```

## 8. UI 当前用法

当前主界面已有 `仿真联调`入口：

1. 启动 RobotSim；
2. 加载场景；
3. 记录控制台 `COMMServer Port`；
4. 在主程序中点击 `仿真联调`；
5. 输入端口；
6. 点击查询。

查询成功后，UI 会缓存最近一次结果：

```cpp
RobotSimVirtualReferenceCache m_lastRobotSimReference;
bool m_hasRobotSimReference;
```

后续行为：

- 点击全局定位时，如果缓存中有三维点，就调用 `runGlobalIcpDryRun(request, snapshot)`；
- IBVS dry-run/单步时，如果缓存中有二维先验，就把 `ibvsLvsObservation` 放入 `IbvsOnlineFrameInput`；
- 点击停止会清空缓存，避免换场景或换端口后误用旧数据。

## 9. 向 RobotSim 发送写命令

RobotSim 通信文档已支持写命令，主项目 `RobotSimDigitalTwinService` 也已有 facade。

### 6.1 修改对象位姿

接口：

```cpp
DigitalTwinCommandResult setObjectPose(
    int objectId,
    const DigitalTwinPose6D& pose,
    bool isRelative,
    bool updateChildren);
```

对应 RobotSim JSON：

```json
["BASE_OBJECT_VOID","SET_POSE",[objectId,[x,y,z,rx,ry,rz],isRel,updateChildren]]
```

示例：

```cpp
i2r_rlc::digital_twin::DigitalTwinPose6D pose;
pose.x_mm = 100.0;
pose.y_mm = 100.0;
pose.z_mm = 100.0;
pose.rx_deg = 0.0;
pose.ry_deg = 45.0;
pose.rz_deg = 60.0;

const auto result = service.setObjectPose(1001, pose, false, false);
```

### 6.2 修改机器人关节值

接口：

```cpp
DigitalTwinCommandResult setRobotJoints(
    int robotId,
    const std::vector<double>& jointValues);
```

对应 RobotSim JSON：

```json
["ROBOT_VOID","SET_Q_VEC",[robotId,[q0,q1,q2,q3,q4,q5]]]
```

示例：

```cpp
const auto result = service.setRobotJoints(
    1002,
    {0.1, 0.2, 0.3, 0.4, 0.5, 0.6});
```

### 6.3 查询对象位姿

接口：

```cpp
DigitalTwinPoseResult getObjectPose(int objectId);
```

对应 RobotSim JSON：

```json
["BASE_OBJECT_VALUE_RET","GET_ABS_POSE",[objectId]]
```

### 6.4 写命令当前验证状态

- JSON 构造和发送路径已有自动测试覆盖；
- `setObjectPose()`、`setRobotJoints()` 是无响应命令，发送时不等待 RobotSim 回包；
- 目前真实 RobotSim 上已经验证读取类命令可用；
- 修改位姿/关节会改变场景状态，尚未做默认实连破坏性测试；
- 实测前必须确认对象 ID、相对/绝对位姿语义和 `updateChildren` 参数。

手工 probe 示例，最后的 `0` 表示不等待响应：

```powershell
$json='["BASE_OBJECT_VOID","SET_POSE",[1001,[100,100,100,0,45,60],false,false]]'
.\out\build\qt-debug\tests\i2r_rlc_robotsim_probe.exe 51900 $json 0
```

## 10. CMake 依赖建议

如果模块只需要 RobotSim 仿真数据的公共接口：

```cmake
target_link_libraries(your_target
    PRIVATE
        i2r_rlc_digital_twin
)
```

如果模块只消费定位值对象或定位服务，不主动连接 RobotSim：

```cmake
target_link_libraries(your_target
    PRIVATE
        i2r_rlc_localization
)
```

如果模块只消费 IBVS 先验，不主动连接 RobotSim：

```cmake
target_link_libraries(your_target
    PRIVATE
        i2r_rlc_ibvs
)
```

如果在 UI/流程编排层需要同时做 RobotSim 查询和跨模块转换：

```cmake
target_link_libraries(your_target
    PRIVATE
        i2r_rlc_ui
)
```

不要为了使用 RobotSim 数据引入 `RobotSim/` 源码目录，也不要在业务模块中直接依赖 RobotSim 的 OCCT、VTK 或内部场景对象。

## 11. 当前注意事项

- 当前默认传感器 ID 是 `1013`；
- RobotSim 端口是运行时随机端口，以控制台 `COMMServer Port` 为准；
- 三维点在 RobotSim 文档中称为绝对位置，单位 mm；具体是世界系、工件系还是传感器相关坐标系，仍建议和 RobotSim 开发者确认；
- `GET_WELD_SEAM_PHOTO` 可能返回多个焊缝点，当前公共结果先使用第一个作为 `seamPointImage_px`；
- 主项目不会自动启动 RobotSim，也不会自动加载场景；
- 当前接入范围是仿真信息读取和 dry-run 消费，不自动发送真实机器人运动命令。
