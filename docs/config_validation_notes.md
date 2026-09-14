# 配置校验说明

## 测试目标

第四阶段收尾新增轻量级测试目标：

```text
i2r_rlc_config_tests
```

该目标不引入 GTest 或第三方测试框架，只依赖 Qt Core 和必要的配置/标定解析模块，不依赖 Qt Widgets，不访问旧工程 `SonTech`。

## 覆盖范围

`i2r_rlc_config_tests` 覆盖以下配置与标定解析场景：

| 场景 | 覆盖内容 |
| --- | --- |
| 完整 `SENSORPARA.txt` | 正常解析全部关键字段。 |
| 单位字段映射 | 验证 `focalLengthM`、`sxM`、`syM`、`cxPixel`、`cyPixel`、`poseCinTXM`、`poseCinTRxDeg` 等字段映射。 |
| legacy 附加行 | 忽略 `More Info` 行和日期行。 |
| 未知行 | 未知行不会导致崩溃。 |
| 缺失关键字段 | 缺失字段时返回失败和结构化错误。 |
| 非法数值 | 数值格式错误时返回失败和结构化错误。 |
| 主路径优先 | 优先查找 `config/calibration/SENSORPARA.txt`。 |
| legacy fallback | 主路径不存在时查找 `data/legacy/Robotleadingdata/BASIC/SENSORPARA.txt`。 |
| 两者缺失 | 返回结构化错误，不弹窗。 |

## 在 VS2022 中构建和运行

在 Visual Studio 2022 中打开 `i2r_rlc` 文件夹后：

1. 选择已确认可用的 Qt/CMake 配置，例如 `Qt-Debug` 或 `qt-debug`。
2. 执行 CMake Configure / Generate。
3. 在目标列表中选择并构建 `i2r_rlc_config_tests`。
4. 运行 `i2r_rlc_config_tests`，成功时控制台输出：

```text
PASS: sensor calibration parser and config repository tests
```

也可以使用 CTest：

```powershell
ctest --test-dir out/build/qt-debug -C Debug --output-on-failure
```

## 命令行运行

普通 PowerShell 可以使用标准构建脚本：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\build_vs2022_qt.ps1 -QtPrefixPath "C:\Qt\6.7.3\msvc2022_64" -BuildDir "out/build/codex-debug" -Configuration "Debug"
```

构建完成后运行：

```powershell
.\out\build\codex-debug\i2r_rlc_config_tests.exe
```

如果使用多配置生成器，测试程序可能位于：

```powershell
.\out\build\codex-debug\Debug\i2r_rlc_config_tests.exe
```

## 当前限制

当前测试只验证第四阶段的数据与配置支撑能力。仍未接入 OpenCV、Vimba、Basler、DX200、HALCON、OpenCascade、PCL、VTK、YOLO/ONNX，也不会发送真实机器人命令。

如果 Codex shell 中 CMake configure 或 build 超时，优先以 VS2022 内部构建结果为准，并在用户本机 PowerShell 或 Developer PowerShell for VS2022 中运行上述命令复验。
