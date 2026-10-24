# Windows / VS2022 离线 KCF 验证

本分支提供六个原生 JSON/JSONL 回放程序和既有 Python 策略、报告入口。
原生程序通过 CMake/MSVC 生成；Python 只运行它们，不在 Windows 自动调用
clang、pkg-config、CMake、Ninja 或设备。应用中的生产 KCF 行为尚未替换。

## 1. 拉取验证分支

在现有 Git 仓库中先运行 `git status --short`，保留未提交工作，不强制覆盖。
工作区干净后：

```powershell
git fetch origin
git switch --track origin/codex/kcf-evidence-integration
git log -1 --oneline
```

若本地已存在同名分支：

```powershell
git switch codex/kcf-evidence-integration
git pull --ff-only origin codex/kcf-evidence-integration
```

若当前文件夹不是 Git clone 而是下载的 ZIP，在新的、不存在的目录克隆该分支，
保留原目录。不要复制 Mac 编译缓存或把旧 build tree 搬到 Windows。

## 2. 在 Visual Studio 2022 构建

使用既有仓库文件夹与依赖：Qt、Eigen、OpenCV 3.4.x（现有配置为 3.4.3）。
Python 3.10+ 与 Git 需要在 Windows 可用。建议仓库和回放临时目录使用英文路径；
OpenCV 3.4 的窄字符图像文件入口尚未完成中文路径验收。

1. VS“打开文件夹”选择仓库根目录，读取工具栏当前 CMake 配置及输出窗口。
2. 选择已有的 `Qt Debug (OpenCV + Allied)` 配置（preset 为 `qt-debug`）。
   本验证分支已在这个 preset 打开 `I2R_RLC_ENABLE_KCF_DIAGNOSTICS=ON`，
   并沿用 `I2R_RLC_ENABLE_TESTS=ON`、`I2R_RLC_ENABLE_OPENCV=ON`。
   其他配置可在本机配置中显式设置这三个开关，不需要新建完整 build tree。
3. 等待 VS 完成 CMake 配置。在 CMake 目标视图右键
   `i2r_rlc_kcf_diagnostics` → 生成；也可按 `F7` 生成当前配置全部目标。
4. 主动观察输出，确认最终成功或记录错误。不要仅看到 `.exe` 就当本轮生成成功。
   若约两分钟没有新输出，按仓库 AGENTS.md 诊断，不重复发起生成。

聚合目标包括：

| VS/CMake 目标 | 原生程序 |
| --- | --- |
| `i2r_rlc_kcf_detector_replay` | `DetectorReplay.exe` |
| `i2r_rlc_kcf_identity_evidence` | `IdentityEvidence.exe` |
| `i2r_rlc_kcf_mother_observation` | `MotherObservation.exe` |
| `i2r_rlc_kcf_control_replay` | `ControlReplay.exe` |
| `i2r_rlc_kcf_refiner_replay` | `RefinerReplay.exe` |
| `i2r_rlc_kcf_surface_replay` | `SurfaceReplay.exe` |

无 QObject 的目标均关闭 AUTOMOC，复用公共 `/utf-8`、`/bigobj` 编译选项。
根项目 OpenCV 3.4.x 限制不变，也未修改任何生产算法。

对于现有 Ninja `qt-debug` 配置，输出位置为：

```text
out/build/qt-debug/tests/diagnostics/native/Debug/
```

同目录还必须有 `replay-native.json`。它由聚合目标在六个程序构建后生成，记录
程序哈希、源码哈希、编译器/配置/Qt/OpenCV 版本及本机 DLL 目录。
只构建单个 `.exe` 不会更新整组清单，应生成聚合目标。

## 3. 运行公开回归

以下 PowerShell 命令仅运行已生成程序，不触发 CMake 构建。在仓库根目录执行：

```powershell
py -3 --version
$env:PYTHONUTF8 = "1"
$env:I2R_KCF_REPLAY_BIN_DIR = (Resolve-Path ".\out\build\qt-debug\tests\diagnostics\native\Debug").Path
Test-Path "$env:I2R_KCF_REPLAY_BIN_DIR\replay-native.json"
py -3 -X utf8 -m unittest discover -s tests/diagnostics/online_detector_replay -p "test_*.py"
```

确认 Python 至少 3.10，`Test-Path` 为 `True`。若使用其他已核实的配置或 build tree，
将变量指向该配置实际输出的 `native/<配置>` 目录，不混用 Debug/Release。
也可使用 VS 测试资源管理器中的 `i2r_rlc_kcf_replay_regression`；
该 CTest 入口已配置程序目录和 UTF-8 环境，但不会替你编译程序。

原因果探针另行运行：

```powershell
py -3 -X utf8 tests/diagnostics/online_correction_repro/verify_replays.py
```

`12/12` 表示原始因果现象按预期复现；其中原生产拒绝案例仍为 RED，
不表示已修复生产故障。对应 VS 测试名为 `i2r_rlc_kcf_causal_probes`。

## 4. 生成组合报告

保留上述两个环境变量，使用一个尚不存在的输出目录：

```powershell
py -3 -X utf8 tests/diagnostics/online_detector_replay/make_integrated_demo.py --output output/analysis/windows_kcf_first
Start-Process .\output\analysis\windows_kcf_first\index.html
```

重复运行时换一个新目录，保留原证据。自定义会话仍使用
`INTEGRATED_README.md` 的 `integrated_replay.py` JSONL CLI，环境变量保持不变。
报告的 `native_runtime` 保留实际构建来源；编译器名称并不授予现场精度、独立校验或运动资格。

## 5. 常见错误及验收边界

- 缺少清单/程序或哈希不符：回到 VS 生成聚合目标，不复制其他配置的旧程序，不关闭检查。
- 提示源码变化或不符合固定基准：重新生成；归档因果探针对固定源代码还会进行
  忽略 CRLF/LF 差异的内容核对，不能把另一套算法假标为原基准。
- DLL 加载失败（常见为 `0xC0000135`）：检查 VS 配置是否成功、清单中的 Qt/OpenCV
  DLL 目录是否仍存在，必要时重新配置/生成。Python 会把这些目录加入子进程 PATH；
  它不下载、不替换 DLL，也不全局修改 Windows PATH。
- Git 原始数据缺失：使用完整 Git 仓库和可访问的 `origin`，归档由固定提交读取，
  不用 ZIP 的缺失数据代替。图片、TCP 或标定关联未知仍按未知处理。

需要在 Windows 记录：实际提交、VS 配置、构建末尾状态、Qt/OpenCV/MSVC 版本、
回归与探针结果，以及组合报告的 `reproducibility.json`。本机 macOS 辅助编译结果
不能替代这些记录。真实连续预焊/反光数据、独立焊缝真值、TCP 同材料关联、
现场横向公差、完整队列/PID与生产集成仍未验收，本分支不发送运动。

## 本次辅助验证记录（2026-10-25）

AppleClang 17 + Qt 6.9.0 + OpenCV 4.11.0 的独立 Unix Makefiles 辅助工程成功构建六程序。
CTest 2/2 通过，包含62项公开CLI/证据回归及12项原因果探针。运行时隔离PATH，
没有提供clang/pkg-config，验证只使用预编译程序。两个部署问题已先红后绿：显式目录
缺少清单时不能静默回退、首次证据回放自动创建事务缓存。Standards与Spec复审无未解决问题。
这些记录不等同于Windows/MSVC/OpenCV3.4.3验收，需按前文在Windows完成并保留结果。
