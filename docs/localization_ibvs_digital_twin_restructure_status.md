# 定位、视觉伺服、数字孪生重构状态

## 结论

截至 2026-06-17，`Localization`、`IBVS`、`DigitalTwin` 三个模块已经从“只有基础接口/局部 dry-run”的状态推进到可配置、可测试、可等待 RobotSim 数据接入的阶段。

当前已经完成：

- 工件 profile 改为运行时 txt 配置，默认工件为 `chufengkouxiao`，支持现有 4 个 IBVS 工件。
- `Localization` 已具备模块内激光线中心线提取、图像点到点云投影适配、局部点云滑窗能力。
- `IBVS` 已具备模块内激光线中心线提取和 `IbvsLvsObservation` 适配能力。
- `DigitalTwin` 已扩展为可选择工件、请求虚拟 2D 激光线、3D 激光线和局部点云的 facade。
- RobotSim 不支持相关命令时，主项目会返回清晰 unavailable，不阻塞 dry-run。
- 相关定向测试已在 `qt-debug` build tree 通过。

当前还没有完成真实在线闭环。剩余工作主要集中在运行时组装、真实数据接入、RobotSim 虚拟传感器 API 对接、真实工件/工艺参数校准和端到端验证。

## 范围边界

本状态只覆盖用户负责的三个模块：

- `Localization/`
- `IBVS/`
- `DigitalTwin/`

以下模块不作为本轮定位/视觉伺服/数字孪生缺口归属：

- `Vision/`
- `Control/`

`Vision/` 和 `Control/` 由其他开发者负责，本轮只在 `AGENTS.md` 中补充约束，避免后续误把它们当成公共模块或改动目标。

`RobotSim/` 和 `oldproject/` 均保持只读。

## 已完成任务

### 1. 工件与工艺 profile 配置

新增统一配置文件：

```text
config/workpieces/workpiece_profiles.txt
```

已支持 4 个工件：

| workpieceId | legacy type | 说明 |
| --- | ---: | --- |
| `lingjian2` | 0 | 直线类工件 |
| `penzuicone` | 2, 3 | 曲线类工件 |
| `chufengkouxiao` | 4 | 当前默认工件 |
| `workpiece3_simM` | 1, 6 | 直线类仿真工件 |

配置中已包含：

- `id`
- `display_name`
- `legacy_type_aliases`
- `shape`
- `robotsim_model_id`
- `robotsim_model_path`
- `calibration_file`
- 定位 ROI、中心线阈值、点云 stride、滑窗帧数、最大点数、ICP DOF 参数
- IBVS feature geometry、position constraint、LVS 融合阈值

兼容性：

- `IBVS` 仍可读取旧 `config/IBVS/workpiece_profiles.txt`。
- 新逻辑优先读 `config/workpieces/workpiece_profiles.txt`。
- `workpieceType=4` 仍可映射到 `chufengkouxiao`。
- `UiController` 暂不加下拉框，只读取默认 `workpieceId`。

### 2. Localization 模块

新增文件：

```text
Localization/include/LocalizationLaserLineObservation.h
Localization/src/LocalizationLaserLineObservation.cpp
```

已完成能力：

- 定义定位模块自有二维激光点、虚拟激光先验、中心线提取结果、点云窗口结构。
- 根据虚拟焊缝点或虚拟激光线生成 ROI。
- 根据虚拟激光线方向估计 Gabor 方向。
- 执行 Gabor 预滤波。
- 执行 Hessian/Steger 风格中心线提取。
- 将 ROI 内坐标还原到整图坐标。
- 复用 `LocalizationPointProjector` 把中心线图像点批量投影为空间点云。
- 支持真实/虚拟点云滑窗累积。
- 滑窗达到帧数上限后丢弃最旧帧。
- OpenCV 相关提取能力只在 `I2R_RLC_ENABLE_OPENCV=ON` 时启用。

当前尚未完成：

- 在线相机帧到 `LocalizationLaserLineExtractor` 的运行时调用。
- 当前机器人/工具位姿、标定、RobotSim 虚拟参考的实时组装。
- 从配置 profile 自动生成完整 `LocalizationPointCloudWindowOptions` 和 ICP 参数的应用层入口。
- 局部点云窗口自动进入 `LocalizationLocalCorrectionPipeline` 的生产流程。
- 使用真实工业图像、真实标定和真实 RobotSim 数据的端到端验证。

### 3. IBVS 模块

新增文件：

```text
IBVS/include/ibvs/IbvsLaserLineObservation.h
IBVS/src/IbvsLaserLineObservation.cpp
```

已完成能力：

- 定义 IBVS 模块自有虚拟激光先验、提取参数和结果。
- 独立实现与 `Localization` 一致的 ROI、Gabor、Hessian/Steger 中心线提取流程。
- 输出真实中心线点列。
- 输出 `IbvsLvsObservation`。
- `startPoint` 当前取中心线首点。
- `weldPoint` 当前取中心线中点。
- 输出 `centerlineQuality` 和失败原因。
- `IbvsOnlineService` 支持 `workpieceId` 优先配置。
- 仍保留 legacy `workpieceType` fallback。
- 工件 profile 中的 feature geometry、position constraint、LVS 阈值可覆盖运行默认值。

当前尚未完成：

- 真实相机帧到 `IbvsLaserLineExtractor` 的在线调用。
- 从 RobotSim 虚拟线或虚拟焊缝点实时构造 `IbvsLaserPrior`。
- LVS 观测与红色特征观测的完整在线同步策略。
- 真实机器人闭环输出仍未开启，只保留 dry-run 和速度草案。
- 真实工件上 `startPoint` / `weldPoint` 选择策略仍需按工艺校准。

### 4. DigitalTwin 模块

扩展文件：

```text
DigitalTwin/include/IDigitalTwinService.h
DigitalTwin/include/RobotSimDigitalTwinService.h
DigitalTwin/src/RobotSimDigitalTwinService.cpp
DigitalTwin/include/NullDigitalTwinService.h
DigitalTwin/src/NullDigitalTwinService.cpp
DigitalTwin/include/DigitalTwinLocalizationVirtualReferenceProvider.h
DigitalTwin/src/DigitalTwinLocalizationVirtualReferenceProvider.cpp
```

已完成能力：

- 增加工件选择输入 `DigitalTwinWorkpieceSelection`。
- 增加虚拟参考请求 `DigitalTwinVirtualReferenceRequest`。
- 增加虚拟参考结果 `DigitalTwinVirtualReferenceResult`。
- 支持请求：
  - 虚拟激光线 2D 图像点；
  - 虚拟激光线 3D 点；
  - 虚拟局部点云。
- `RobotSimDigitalTwinService` 可发送：
  - `DIGITAL_TWIN_VOID / SET_WORKPIECE_MODEL`
  - `DIGITAL_TWIN_VALUE_RET / GET_VIRTUAL_REFERENCE`
- `DigitalTwinLocalizationVirtualReferenceProvider` 已从固定失败改为实际调用 service 并转换成定位快照。
- `NullDigitalTwinService` 返回明确 unavailable。

当前尚未完成：

- RobotSim 端是否已实现上述新命令仍待对方确认。
- 真实 RobotSim endpoint、模型 id/path、虚拟传感器状态还没有端到端联调。
- 端口发现、握手、能力查询、版本协商仍未完成。
- 虚拟参考缓存、重试和时序对齐策略未实现。

## 测试状态

已执行命令：

```powershell
ctest --test-dir out\build\qt-debug -C Debug -R "i2r_rlc_(ibvs_text_config_reader|ibvs_online_service|digital_twin|localization_laser_line_observation|ibvs_laser_line_observation)_tests" --output-on-failure
```

通过测试：

| 测试 | 结果 |
| --- | --- |
| `i2r_rlc_ibvs_online_service_tests` | Passed |
| `i2r_rlc_ibvs_text_config_reader_tests` | Passed |
| `i2r_rlc_localization_laser_line_observation_tests` | Passed |
| `i2r_rlc_ibvs_laser_line_observation_tests` | Passed |
| `i2r_rlc_digital_twin_tests` | Passed |

结论：

```text
100% tests passed, 0 tests failed out of 5
```

其他检查：

- `git diff --check` 通过，仅有 Git LF/CRLF 提示。
- 未修改 `Vision/`、`Control/`、`RobotSim/`、`oldproject/`。
- `qt-debug-no-opencv` 当前 build tree 未注册本轮相关测试，因此无可执行条目，不视为失败。

## RobotSim 仍需提供的能力

主项目不要求 RobotSim 承担工件语义、工艺选择、定位策略或 IBVS 策略。RobotSim 只需提供仿真与模型相关的原始能力：

1. 工件模型选择或同步。
2. 当前工件模型/OCCT 对象可被虚拟传感器使用。
3. 虚拟激光线 2D 图像点。
4. 虚拟激光线 3D 点。
5. 虚拟局部点云。
6. 明确的 unsupported/unavailable 返回。
7. 稳定端点、端口发现或握手机制。

当前主项目侧已经为这些能力预留 facade 和测试 fake transport。RobotSim 实现后，重点是联调协议字段和坐标系。

## 还差的整体功能

### 必须补齐

1. 在线运行时组装：
   - 相机帧；
   - 当前工具位姿；
   - 标定；
   - 当前 `workpieceId`；
   - RobotSim 虚拟参考；
   - Localization/IBVS 激光线提取入口。

2. Localization 在线局部定位流程：
   - 图像中心线提取；
   - 点云重建；
   - 滑窗点云累计；
   - 进入 `LocalizationLocalCorrectionPipeline`；
   - 输出位姿修正和诊断。

3. IBVS 在线 LVS 输入适配：
   - 虚拟激光先验到 `IbvsLaserPrior`；
   - 真实图像到 `IbvsLvsObservation`；
   - 与 torch/red feature 观测融合；
   - dry-run 输出验证。

4. RobotSim 真实联调：
   - 端口/连接；
   - 工件选择；
   - 虚拟线/点云请求；
   - 坐标系和单位验证。

5. 工件/工艺参数校准：
   - 4 个工件的 ROI、阈值、stride、窗口帧数、ICP DOF；
   - IBVS feature geometry；
   - LVS 质量阈值；
   - 工件模型 id/path。

### 可以后置

1. UI 工件下拉框。
2. DigitalTwin 虚拟参考缓存。
3. RobotSim 能力查询和协议版本管理。
4. 多工件新增/删除的配置管理界面。
5. 全量真实数据 replay 报告。
6. 性能优化和多线程节流。

## 建议下一步

| 优先级 | 工作 | 目标 |
| --- | --- | --- |
| P0 | 与 RobotSim 开发者确认虚拟激光线/点云 API | 打通 DigitalTwin 数据源 |
| P0 | 写运行时适配层，把相机帧和 RobotSim 虚拟线送入 Localization/IBVS | 形成真实数据入口 |
| P1 | 局部定位端到端 dry-run | 验证中心线、点云、ICP 是否连通 |
| P1 | IBVS LVS 输入 dry-run | 验证 LVS 观测能影响融合逻辑 |
| P1 | 校准 4 个工件配置参数 | 减少硬编码和重新编译 |
| P2 | UI 下拉框和配置刷新 | 提升使用便利性 |
| P2 | RobotSim 联调报告和真实数据 replay | 固化验收证据 |

