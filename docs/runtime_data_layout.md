# 运行时数据布局

## 目标

新的 Qt 应用不能依赖当前工作目录。运行时路径应通过 `AppPaths` 解析，并由 `ConfigRepository` 加载和校验。

该布局将以下内容分离：

- 输入配置；
- 作业和工件定义；
- 模型资源；
- legacy 兼容数据；
- replay/test 数据；
- 日志；
- 生成输出。

## 推荐根目录

应用应支持可配置的运行时数据根目录，例如：

```text
<runtime_data_root>/
```

`AppPaths` 应从显式配置、命令行选项、安装器提供的位置或用户选择的项目目录中发现该根目录。它不能假定进程工作目录就是仓库根目录。

## 目录树

```text
<runtime_data_root>/
  config/
    app/
    camera/
    robot/
    calibration/
    workpieces/
    workflows/
  jobs/
  models/
    weldment/
    ibvs/
  data/
    legacy/
    replay/
  logs/
  Data/
    CALID/
      4 TWO_STEP/
  output/
    images/
    point_clouds/
    traces/
    reports/
```

## 目录职责

| 目录 | 内容 | 类别 |
| --- | --- | --- |
| `config/app/` | 应用级设置：UI 默认值、feature flag、dry-run 默认值、运行时根目录元数据。 | 输入配置。 |
| `config/camera/` | Vimba 焊缝跟踪、Basler IBVS、离线 replay 的相机 profile，以及曝光/帧率默认值和标定 profile 引用。 | 输入配置。 |
| `config/robot/` | DX200 endpoint、mock/dry-run、协议和安全门控设置。真实命令启用状态必须默认为 false。 | 输入配置。 |
| `config/calibration/` | 规范化标定文件，包括新格式传感器标定、工位/机器人标定和自动标定模板位姿 `CalibPose.txt`。 | 输入配置。 |
| `config/workpieces/` | 工件注册表、焊缝定义、模型引用和工件专用参数。 | 输入配置。 |
| `config/workflows/` | 焊缝跟踪 workflow profile、离线 replay 设置和算法参数集。 | 输入配置。 |
| `jobs/` | 作业定义和机器人作业元数据，后续如需要可包含 JBI 名称或映射。 | 输入配置。 |
| `models/weldment/` | 用于焊缝跟踪和未来数字孪生的 `.weldment` 或转换后的工件几何/模型文件。 | 输入资源。 |
| `models/ibvs/` | IBVS 专用模型，例如 ONNX 文件、期望图像和特征模板。首版骨架不要求。 | 输入资源。 |
| `data/legacy/` | 只读复制的 legacy 文件，用于兼容、参考或迁移 fixture。 | 兼容输入。 |
| `data/replay/` | 离线图像、记录帧、机器人状态 trace 和开发/测试 fixture。 | 测试/replay 输入。 |
| `logs/` | 运行日志、诊断文本、协议 trace、worker 生命周期日志。 | 运行时输出。 |
| `Data/CALID/` | 自动标定运行时输出根目录，包含 two-step 采集子目录。 | 运行时输出。 |
| `Data/CALID/4 TWO_STEP/` | 标定数据采集输出，兼容旧 `CALID` 命名，包含 `CalibPose_new.txt`、`CalibOriginData.txt`、按 `image_01.png` 到 `image_19.png` 顺序记录 TCP 六数组的 `CalibResult.txt`，以及与后 4 张板图同编号配对的 `image_16L.png` 到 `image_19L.png`。 | 运行时输出。 |
| `output/images/` | 保存的调试图像、overlay 和处理后帧。 | 运行时输出。 |
| `output/point_clouds/` | 生成的点云或导出的数字孪生数据。 | 运行时输出。 |
| `output/traces/` | 跟踪 trace、机器人 dry-run 命令 trace 和纠偏 trace。 | 运行时输出。 |
| `output/reports/` | 生成报告、校验摘要、迁移/调试报告。 | 运行时输出。 |

## SENSORPARA.txt 放置位置

`SENSORPARA.txt` 是 legacy 传感器标定文件，应作为输入兼容文件支持。

推荐的新位置：

```text
<runtime_data_root>/config/calibration/SENSORPARA.txt
```

推荐的 legacy 兼容镜像位置：

```text
<runtime_data_root>/data/legacy/Robotleadingdata/BASIC/SENSORPARA.txt
```

`ConfigRepository` 应能按配置从任一位置加载。推荐的规范化路径是 `config/calibration/`。`data/legacy/` 位置用于迁移或 replay 期间保留旧目录结构。

## 自动标定位姿文件

自动标定模板位姿使用旧工程 `CALID/CalibPose.txt` 格式，作为输入配置保存到：

```text
<runtime_data_root>/config/calibration/CalibPose.txt
```

点击“自动标定”进入红框预览时，该文件会被解析为旧 `O` 示教点更新命令。点击
“确认”后，应用使用最近一次有效 D 回包中的当前机器人位姿对齐首点，并把修正结果写到：

```text
<runtime_data_root>/Data/CALID/4 TWO_STEP/CalibPose_new.txt
```

`CalibPose_new.txt` 是运行时输出，不应回写到 `config/`。

## Legacy 路径兼容

旧项目会引用类似路径：

```text
data/Robotleadingdata/BASIC/SENSORPARA.txt
```

新应用不应硬编码该相对路径，而应：

1. `AppPaths` resolves `<runtime_data_root>`.
2. `ConfigRepository` checks the configured calibration path first.
3. If legacy compatibility is enabled, `ConfigRepository` may search:

```text
<runtime_data_root>/data/legacy/Robotleadingdata/BASIC/SENSORPARA.txt
```

4. loader 报告实际使用的路径。
5. 文件缺失或格式错误时，产生结构化配置错误。

## 输入与输出规则

## penzui 离线 replay 数据

`data/replay/penzui/` 是当前焊接视觉迁移使用的真实离线 fixture。该目录用于
验证 OpenCV-only vision 算法，不代表真实相机、DX200 或 TCP 已接入。

| 路径 | 内容 | 当前用途 |
| --- | --- | --- |
| `data/replay/penzui/imagecatch/` | 采集图片，命名为 `image_0.png`、`image_1.png` 等 | 离线读取为 `CV_8UC1 cv::Mat`，用于 replay 测试 |
| `data/replay/penzui/robotPos.txt` | 每帧机器人和变位机位姿 | 后续 geometry / correction replay 输入；当前 ImageProShare replay 不使用 |
| `data/replay/penzui/SENSORPARA.txt` | 标定数据 | 后续图像点到空间点重建输入；当前 ImageProShare replay 不使用 |
| `data/replay/penzui/simulatedLaser.txt` | 每帧仿真激光线 2D 点对 | 当前用于 ImageProShare replay 的 ROI、控制点和诊断输入 |
| `data/replay/penzui/simulatedWeldpt.txt` | 每帧仿真焊缝点 2D 点对 | 当前用于 ImageProShare replay 的控制点和拖拽起点输入 |

当前 `i2r_rlc_vision_image_pro_share_replay_tests` 会抽样扫描该目录，
生成 `output/reports/image_pro_share_replay_report.csv`。如果该目录或必要文本
不存在，测试输出 `SKIP` 并返回 0。

只作为输入的目录：

- `config/`
- `jobs/`
- `models/`
- `data/legacy/`
- `data/replay/`

运行时输出目录：

- `logs/`
- `output/`

应用不应将生成图像、trace 或日志写入 `config/`、`jobs/`、`models/` 或 `data/legacy/`。

## AppPaths 职责

`AppPaths` 应暴露带语义的路径访问器，而不是让调用方拼接原始字符串：

| 路径访问器 | 示例目标 |
| --- | --- |
| `runtimeRoot()` | `<runtime_data_root>/` |
| `appConfigDir()` | `<runtime_data_root>/config/app/` |
| `cameraConfigDir()` | `<runtime_data_root>/config/camera/` |
| `robotConfigDir()` | `<runtime_data_root>/config/robot/` |
| `calibrationConfigDir()` | `<runtime_data_root>/config/calibration/` |
| `calibPoseTemplateFilePath()` | `<runtime_data_root>/config/calibration/CalibPose.txt` |
| `calibPoseCorrectedFilePath()` | `<runtime_data_root>/Data/CALID/4 TWO_STEP/CalibPose_new.txt` |
| `jobsDir()` | `<runtime_data_root>/jobs/` |
| `weldmentModelsDir()` | `<runtime_data_root>/models/weldment/` |
| `ibvsModelsDir()` | `<runtime_data_root>/models/ibvs/` |
| `legacyDataDir()` | `<runtime_data_root>/data/legacy/` |
| `replayDataDir()` | `<runtime_data_root>/data/replay/` |
| `logsDir()` | `<runtime_data_root>/logs/` |
| `outputImagesDir()` | `<runtime_data_root>/output/images/` |
| `outputPointCloudsDir()` | `<runtime_data_root>/output/point_clouds/` |
| `outputTracesDir()` | `<runtime_data_root>/output/traces/` |
| `outputReportsDir()` | `<runtime_data_root>/output/reports/` |

## ConfigRepository 职责

`ConfigRepository` 应负责：

- 通过 `AppPaths` 加载和校验配置；
- 在 workflow 启动前报告缺失的必需文件；
- 显式保留 legacy 路径兼容逻辑；
- 向 `UiController` 暴露类型化配置对象；
- 避免直接依赖 `QWidget`；
- 避免静默 fallback 到当前工作目录。
