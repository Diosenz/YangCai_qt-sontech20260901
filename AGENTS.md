# AGENTS.md

## 仓库概览

本仓库根目录就是正在重构的 `i2r_rlc` Qt Widgets + CMake 独立项目，目标是机器人定位与纠偏系统。当前架构以模块独立生成为主：根 CMake 只做全局选项、公共编译选项和 `add_subdirectory`，各模块在自己的目录内维护 `include/`、`src/`、`CMakeLists.txt` 和直接依赖发现。

主要目录含义：

- `Camera/`、`Localization/`、`Vision/`、`Control/`、`IBVS/`、`DigitalTwin/`、`RobotComm/`、`UI/`：当前 8 个生产模块。`UI/src/main.cpp` 是应用入口。
- `tests/`：可执行测试和 smoke tests。RobotComm 原有 Test 不作为生产模块挂入，必要测试迁移到这里。
- `docs/`：架构、迁移、线程、环境、风险和 legacy 分析文档。
- `config/`、`data/`、`resources/`：当前 Qt 项目的配置、运行数据、离线回放数据和资源文件。
- `oldproject/`：旧版工业视觉应用程序，只读参考。分析 legacy 代码前先阅读 `oldproject/sontech-works-version-storage/AGENTS.md`，但旧工程规则不得覆盖当前根目录约定。
- `RobotSim/`：机器人仿真与离线编程系统，只读参考。当前 Qt 项目与 RobotSim 的对接代码优先放在 `DigitalTwin/`，机器人通信生产库放在 `RobotComm/`。

本项目不是机械翻译 MFC。目标是理解 `CRobotleadingDlg` 相关能力，重新拆分职责，形成可测试、可维护、线程边界清晰的现代 C++ / Qt Widgets 应用。

## 严格规则

1. 禁止修改 `oldproject/` 下任何内容，只能读取和分析。
2. 禁止修改 `RobotSim/` 下任何内容，除非用户以后明确改变策略。
3. 所有新代码、重构工作、文档、CMake 文件和测试都必须放在当前仓库根目录下的对应目录中。
4. 实现或改动业务功能前，先基于当前代码、测试和文档确认现有行为与调用链。仅当任务涉及 legacy 兼容、行为迁移、协议语义、历史数据格式，或当前实现存在无法仅靠现有代码确认的行为时，才检查与当前任务直接相关的 legacy 文件。legacy 仅作为行为证据和兼容性参考，不作为默认实现模板。
5. 不要创建单体 Qt 窗口复制 `CRobotleadingDlg`。
6. 只使用 Qt Widgets、CMake、Visual Studio 2022 / MSVC；除非用户明确要求，不使用 Qt Quick、QML 或 qmake。

## 通用协作与执行偏好

默认使用中文回答。除非用户明确要求其他格式，回答应先给结论，再给结构化说明，最后给操作建议或后续建议。

优先基于当前 workspace、已引用文件和实际代码内容分析，不要脱离代码臆测。若信息不足但仍需判断，必须明确说明“根据当前代码推测”。

若用户没有要求落地修改，默认只分析，不改文件。中大型任务应先分析、确认目标并制定计划；简单且低风险的任务可以直接处理。

当用户要求下拉、推送、合并分支、解决 Git 冲突或操作 `master` / `integrate` / `Twin` / `dev` 协作分支时，必须先阅读并遵守：

```text
docs/collaboration_git_workflow.md
```

其中“拉取与本地冲突解决”和“验证通过后再推送”必须分开处理。

执行和修改时遵守以下偏好：

- 默认采用最小修改原则，只修改完成当前任务所必需的内容。
- 不做无关重构，不顺手清理历史代码，不进行大范围命名统一。
- 不新增依赖，除非用户明确要求或当前任务确实需要。
- 修改前优先确认入口文件、调用链上下游、共享状态、线程、通信、状态机和 UI 流程影响。
- 修改后汇报涉及文件、是否验证，以及仍需注意的风险或限制。
- 分析 `oldproject/` 时可以参考其内部 `AGENTS.md` 的 legacy 阅读顺序和风险提示，但不要把 VS2017、MFC 主工程定位、“不要默认使用 CMake”等旧工程规则套用到当前 Qt 项目。

### 行为证据与 legacy 使用原则

判断现有系统行为时，证据优先级原则上为：

1. 用户当前任务中明确给出的事实和约束；
2. 当前生产代码的实际数据流和运行行为；
3. 当前自动化测试及可重复的运行验证结果；
4. 当前架构和设计文档；
5. legacy 实现。

如果这些证据之间存在冲突，不要静默选择其中一个。先指出冲突，并以当前架构和当前任务目标为边界决定是否需要兼容 legacy。
## 构建与验证首选路径

Codex 需要生成、重新生成或验证 MSVC/Ninja 目标时，默认不再通过 PowerShell
直接发起 `cmake --preset`、`cmake --build` 或 `ctest`。优先使用 Computer Use
直接操控已经打开的 Visual Studio 2022，让 VS 自己维护 Developer 环境、CMake
配置、Ninja 路径和输出窗口。

首选流程固定为：

1. 先用 Computer Use 连接 `qt-sontech - Microsoft Visual Studio` 窗口。
2. 读取 VS 当前 CMake 配置下拉框和输出窗口，确认正在使用的配置。
3. 如需切换配置，优先操作 VS 工具栏里的配置下拉框；必要时用鼠标/键盘在下拉项中选择，不根据 `CMakePresets.json` 文件名盲猜当前配置。
4. 触发生成时优先使用 VS 快捷键或菜单：当前 VS 键位下 `F7` 可作为“全部生成/生成当前配置”的快捷入口；若使用 `Alt+B`，注意它在本机曾直接触发“全部生成”，不要把它当成纯粹的菜单展开动作。
5. 生成后读取 VS 输出窗口里的最终状态，例如“全部生成 已成功”或具体编译错误，再向用户汇报。

只有在以下情况之一成立时，才允许 Codex 改走命令行生成：

- 用户明确要求使用命令行、Developer PowerShell 或特定 CMake/CTest 命令；
- Computer Use 在当前线程不可用，且用户同意用命令行兜底；
- 只做只读诊断，例如查看进程、锁文件、CMakeCache 或磁盘空间，不发起生成。

普通 PowerShell 里仍然禁止直接裸跑：

```powershell
cmake --build out\build\<preset-dir> --target <target> --config Debug
```

原因：普通 shell 可能没有 VS2022 Developer 环境，`cl.exe` / `link.exe` /
`ninja.exe` 和 `INCLUDE` / `LIB` / `LIBPATH` / `VCINSTALLDIR` 等变量不完整。
即使 CMakeCache 里有工具绝对路径，也不能等价替代 VS Developer 环境。

OpenCV 默认复用 `qt-debug` / `qt-release` preset 中配置的外部 OpenCV 3.4.3。
ONNX Runtime 不再作为默认共享 preset 暴露；如需验证 IBVS YOLO，应在既有 build tree
上显式启用 `I2R_RLC_ENABLE_ONNX_RUNTIME=ON`、`VCPKG_MANIFEST_FEATURES=ibvs-yolo`
和 `VCPKG_OVERLAY_PORTS=${sourceDir}/vcpkg-ports/onnxruntime`，不要为了 Codex/命令行
另建完整 CLI build 目录，除非已确认磁盘空间足够。该 feature 会复制/还原 ONNX Runtime
等依赖，新 build tree 可能额外占用数 GB，C 盘空间不足时会出现
`There is not enough space on the disk` 或 vcpkg lock 残留。

若日志出现以下错误：

```text
ninja: error: remove(.ninja_lock): Access is denied.
```

处理顺序固定为：

1. 关闭 VS2022，避免 IDE 监视或占用同一个 Ninja build tree。
2. 检查并停止残留构建进程：

   ```powershell
   Get-Process devenv,cmd,cmake,ninja,cl,link,vcpkg -ErrorAction SilentlyContinue | Select-Object Id,ProcessName,CPU,StartTime,Path
   ```

3. 在确认没有相关构建进程后，删除陈旧锁文件：

   ```powershell
   Remove-Item out\build\qt-debug\.ninja_lock -Force
   ```

4. 回到 VS2022，通过 Computer Use 重新触发对应配置的生成。

如果 `.ninja_lock` 或 `vcpkg-running.lock` 在普通 PowerShell 中仍然删不掉，说明当前
shell 权限或外部进程持有状态异常；不要反复启动生成。先清残留进程和锁文件，必要时
使用提升权限只做锁/临时目录清理，再回到既有 build tree 构建。

Codex 操作 VS 发起构建时必须主动观察输出窗口。若构建超过约 2 分钟没有新输出，
或进程列表里只有低 CPU 的 `cmake` / `ninja` 而没有 `cl.exe` / `link.exe`，
应立即停止等待并转入诊断，不要继续空等。

不要只依赖 Codex shell 工具的 `timeout_ms` 作为 Windows 嵌套构建的保护；
`cmd -> cmake -> ninja -> cl/link` 这种子进程链可能无法被工具超时可靠回收。
Codex 不应反复发起完整 MSVC/Ninja 命令行构建；需要构建时优先交由 VS IDE。

纯 C++ 静态库或手写 smoke test 若不含 `QObject` / `Q_OBJECT` / `Q_GADGET`，应在对应 CMake target 上显式 `AUTOMOC OFF`。全局 `CMAKE_AUTOMOC ON` 会让这类目标进入不必要的 CMake AUTOGEN 阶段；在 Codex/普通 shell 环境中，这一阶段如果中断容易留下 `.ninja_lock` 或 `ParseCache.txt.tmp*`，造成“没有 cl.exe、只有 cmake/ninja 低 CPU”的假卡死。

当 Windows/MSVC 构建被用户中断或 Codex 工具中断后，先检查是否仍有残留构建进程占用构建目录：

```powershell
Get-Process cmake,ninja,cl,link -ErrorAction SilentlyContinue | Select-Object Id,ProcessName,CPU,StartTime,Path
```

若确认只是本轮中断遗留的 `cmake` / `ninja` / `cl` / `link` 进程，可按具体进程 ID 停止，避免后续构建继续卡住或抢占文件：

```powershell
Get-Process -Id <pid1>,<pid2> -ErrorAction SilentlyContinue | Stop-Process -Force
```

若 MSVC 报 `C1128: 节数超过对象文件格式限制`，通常是 Eigen/PCL/OpenCV 等模板代码导致单个 `.obj` section 数过多；当前项目应通过公共 CMake helper 统一添加 `/bigobj`，不要在单个源文件中绕过该问题。

## 当前架构要求

模块分层以当前 8 个根级模块为准：

- `UI`：最上层，合并原 app/ui，采用 MVC。View 是 QWidget，Controller/ViewModel 管理界面状态和用户意图，业务执行下放到模块线程。
- `IBVS`：视觉伺服模块。
- `Control`：焊接纠偏与流程控制模块。
- `Vision`：焊缝识别模块，只放焊接纠偏前置视觉算法，不承载 IBVS/Localization 专属视觉处理。
- `Localization`：定位模块，拥有全局/局部定位和点云配准边界；公共接口不暴露 PCL 类型。
- `RobotComm`：机器人通信生产库，替代旧 `robot`/`communication` 生产边界。
- `Camera`：相机采集模块。
- `DigitalTwin`：数字孪生模块，用于与 RobotSim 交互和提供仿真参考数据边界。

`Vision/` 和 `Control/` 由其他开发者负责，虽然名称容易被误解为公共视觉或公共流程模块，但在定位、视觉伺服、数字孪生/RobotSim 对接任务中不得默认把它们当作可改动或可依赖的公共模块。除非用户明确要求，否则不要把用户负责的 `Localization/`、`IBVS/`、`DigitalTwin/` 缺口归入 `Vision/` 或 `Control/`，也不要为完成这些任务修改 `Vision/` 或 `Control/`。

不要重新引入 `core`、`infrastructure`、`app`、`robot`、`communication` 作为生产模块。各模块需要的数据结构应定义在本模块公开头文件中，其他模块通过 include 使用。

依赖方向默认自上而下：`ui` 可以依赖中间层和底层；中间层可以依赖底层；同层依赖必须显式、单向，避免循环依赖。

UI 层不能直接包含长时间运行循环、阻塞式 camera acquisition、阻塞式 robot communication、重型 image processing、原始设备生命周期管理或复杂 workflow logic。

## 线程要求

`main.cpp` 负责运行时组装和线程生命周期：创建 UI、创建模块线程、注入模块对象、启动、请求停止、join/回收。业务细节留在模块内。

当前线程模型要求：

- UI 线程只处理 QWidget、用户输入和 MVC 状态更新。
- camera acquisition、vision/weld detection、localization、ibvs/control loop、RobotComm、digital_twin 等任务不在 UI 线程执行。
- 核心模块默认少继承 QObject、少用 connect；优先使用纯 C++ worker/interface、值对象、状态快照、回调接口或轻量消息对象。
- Qt signal/slot 主要留在 UI 边界和必须进入 Qt event loop 的地方。
- 不从非 UI 线程直接访问 widgets。
- 避免 detached raw threads；stop 必须幂等，退出时必须安全回收。

## C++ 编程规范入口

根目录 `AGENTS.md` 是当前仓库的入口约束，不是完整 C++ 编程规范。完整规则库见：

```text
docs/I2R_Cpp_Codex_Rules.md
```

修改 `.h` / `.cpp`、补注释、重构、修复编译错误、调整接口前，必须阅读并遵守该规则库中与当前任务相关的章节。

发生冲突时，按以下顺序执行：

1. 用户当前任务中的明确要求；
2. 更近层级的 `AGENTS.md`；
3. 当前文件或当前模块已有风格；
4. `docs/I2R_Cpp_Codex_Rules.md`；
5. 通用 C++ 最佳实践。

C++ 修改必须遵守以下最小硬约束：

- 头文件使用 `.h`，源文件使用 `.cpp`，除非当前模块已有强约定。
- 新增头文件必须有 include guard，不使用含糊宏名。
- 缩进使用 4 个空格，不使用 Tab。
- 不做无关格式化、无关重排、无关重命名。
- 不主动引入大范围依赖、全局变量、RTTI、默认参数或 C 风格强制类型转换。
- 公共接口、跨模块类、虚函数/重写函数、跨线程函数、生命周期管理函数、非直观算法函数必须有必要注释。
- 注释必须解释职责、边界、约束、副作用或原因，不写显而易见的代码翻译。
- 禁止生成空 `@brief`、空 `@param`、空 `@return` 等 Doxygen 标签。
- TODO/FIXME 必须包含责任人、日期和明确事项，不得生成无责任人的 TODO。

## 编码和文档注释要求

该项目在 Windows 上使用 Visual Studio 2022 / MSVC 开发。所有生成或修改的源文件都应避免在 Visual Studio 中出现中文乱码。

- 所有新建或修改的文本文件应保存为 UTF-8。
- C++ headers/source 如可行优先使用 UTF-8 with BOM。
- MSVC target 必须通过公共 CMake helper 添加 `/utf-8`。
- 不创建 ANSI、GBK 或依赖本地代码页的文件。
- public classes、public functions、重要 data structures、interfaces 和不明显的 module boundaries 使用 Doxygen-style comments。
- 注释中文优先，解释 intent、ownership、threading behavior、units、安全约束和 legacy compatibility decisions。
- C++ identifiers 保持英文；中文可用于 comments 和 Qt user-facing text。

## 文档要求

`docs/` 用于记录需要跨任务长期保留的架构、行为和兼容性知识，不要求每次代码修改都同步产生文档修改。

当本次工作发现或改变以下内容时，应优先更新已有文档；没有合适文档时再考虑新增：

- 跨模块接口或依赖边界；
- 对外协议、通信语义或持久化数据格式；
- 状态机的重要状态、事件或生命周期语义；
- 线程、ownership、资源生命周期或安全约束；
- legacy 兼容行为、迁移规则或与旧系统的重要差异；
- 会影响后续开发者判断的重要架构决策；
- 当前文档与实际代码存在会误导后续开发的明显不一致。

以下修改通常不要求单独更新 `docs/`，除非它们同时改变了上述长期约束：

- 局部 UI 展示或格式调整；
- 使用现有数据流增加一个显示字段；
- 局部 bug 修复；
- 补充或调整测试；
- 不改变外部行为的内部实现整理；
- 机械性修改、局部命名或样式调整。

检查 legacy 后，如果没有发现需要长期保留的新约束，不要为了证明已经检查过 legacy 而创建新的文档。必要时只需在任务结果中简要说明检查结论。

涉及定位、视觉伺服、数字孪生或 RobotSim 对接时，应先阅读 `docs/localization_visual_servo_digital_twin_scope.md`，再根据当前任务需要结合旧项目代码、`oldproject/sontech-works-version-storage/AGENTS.md` 和当前架构文档继续分析。

## 跨平台编辑与 Windows 验证

本仓库允许在 macOS/Linux 环境中进行源码编辑、代码审查和非平台相关的静态分析，但 Windows + Visual Studio 2022 / MSVC 是当前生产代码的最终构建与运行验证环境。

在非 Windows 环境修改代码时：

- 不得因为当前执行环境是 macOS/Linux 而引入 POSIX 专属 API 或改变 Windows/Qt 的既有实现策略；
- 不得提交本机绝对路径、构建目录或平台专属临时配置；
- include、文件名和 CMake 路径的大小写必须与仓库实际文件完全一致；
- 保持仓库规定的 UTF-8 编码和换行策略；
- 非 Windows 环境中的编译结果只能作为辅助验证，不能替代 MSVC 验证。

涉及 `.h`、`.cpp`、CMake 或 Qt 运行行为的修改，最终以 Windows 上 Visual Studio 2022 / MSVC 的构建、测试和必要的运行验证结果为准。

## Agent skills

### Issue tracker

创建、读取或更新任务前，阅读 `docs/agents/issue-tracker.md`。
本仓库使用 `.scratch/` 下的本地 Markdown 跟踪问题。

### Triage labels

分诊或变更任务状态前，阅读 `docs/agents/triage-labels.md`，
使用其中的默认状态名称。

### Domain docs

探索领域概念或讨论架构决策前，阅读 `docs/agents/domain.md`。
采用 single-context 布局。
