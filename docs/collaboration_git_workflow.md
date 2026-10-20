# 协作 Git 工作流程

本文档用于约定两人通过 `integrate` 分支协同开发时的分支职责、合入规则、冲突处理和最小验证流程。

本文档只说明协作流程和操作约定，不负责创建远程分支、不替代 `AGENTS.md`、不替代模块设计文档，也不要求两位开发者拥有完全相同的本地参考目录。

## 分支职责

### `master`

`master` 是稳定分支，只接收已经确认可构建、可运行、阶段性可交付的成果。

不要把日常开发中的半成品、未验证集成结果或临时调试改动直接合入 `master`。

### `integrate`

`integrate` 是共同集成分支，用于汇合 `Twin`、`dev` 以及后续短功能分支中的阶段性成果。

`integrate` 不是个人日常开发分支。它的作用是提前暴露跨模块冲突、完成本地集成验证，并为后续进入 `master` 提供共同基准。

合并到远程 `integrate` 前，必须先在本地完成冲突解决、代码分析和最小验证。

### `Twin`

初始责任范围包括：

- 数字孪生相关模块；
- RobotSim 对接、IPC、状态同步和数据转换设计；
- `digital_twin`、`localization`、`ibvs` 相关接口和阶段性实现；
- 与定位、视觉伺服、虚拟参考相关的文档和验证。

如果这些工作需要修改共享模型、状态机、CMake、通信协议或 UI 流程，应先同步影响范围，再合入 `integrate`。

### `dev`

初始推测责任范围包括：

- 焊缝检测；
- 焊接纠偏算法；
- 纠偏流程和工作流控制；
- `vision`、`control` 中与焊接纠偏直接相关的实现；
- 相关测试、验证和文档。

以上内容是根据当前项目结构和协作计划做出的初始推测，可以后续修正。

## 模块责任边界

为了减少集成冲突，按模块先约定主责范围。

`Twin`主导：

- `DigitalTwin/`
- `Localization/`
- `Ibvs/`
- RobotSim IPC、虚拟参考、仿真状态同步相关设计

`dev`主导：

- `Vision/`
- 焊缝检测
- 焊接纠偏算法
- 纠偏流程实现

共同维护：

- `Ui/`
- `Ibvs/`
- `Control/`
- `Localization/`
- `Camera/`
- `RobotComm/`
- `DigitalTwin/`
- `CMakeLists.txt`
- `CMakePresets.json`
- `docs/`
- `tests/`
- `config/`、`data/`、`resources/` 中需要共享的项目文件

共同维护区域是高冲突区域。涉及公共数据结构、跨模块接口、线程模型、状态机、通信协议、配置格式或 CMake 目标时，应先说明影响范围，再合入 `integrate`。

## 拉取、合并、验证、推送分离

拉取和推送必须分开处理。

推荐顺序是：

1. 拉取最新远程信息；
2. 更新本地 `integrate`；
3. 将个人分支合入本地 `integrate`；
4. 本地解决冲突；
5. 开发者分析、构建、测试并确认可行；
6. 确认后再推送到远程 `integrate`。

不要把“文本冲突已经解决”理解为“可以推送”。只有本地分析和最小验证通过后，才应推送 `integrate`。

## 合入 `integrate` 前检查

合入前至少检查：

- 当前工作区是否有意外未提交内容；
- 是否已经获取最新远程分支；
- 本地 `integrate` 是否已经同步最新远程 `integrate`；
- 当前个人分支是否已经基于最新 `integrate` 做过必要同步；
- 是否修改了公共接口、CMake、配置格式、通信协议、状态机或线程模型；
- 是否需要同步更新 `docs/`；
- 是否至少完成了相关构建或最小测试；
- 是否包含本地生成文件、个人配置或临时调试文件。

涉及共享模块时，应优先提交小而清晰的阶段性改动，避免长时间积累后再一次性合入。

## 标准操作流程

以下命令是操作模板。执行前应确认当前分支和工作区状态符合预期。

### 1. 查看状态

```powershell
git branch --show-current
git status --short
```

如果存在未提交内容，先判断这些内容是否属于当前任务。不要在未确认的情况下覆盖、清理或重置工作区。

### 2. 获取远程更新

```powershell
git fetch origin
```

只获取远程引用，不等同于推送，也不等同于发布集成结果。

### 3. 更新本地 `integrate`

```powershell
git switch integrate
git pull --ff-only origin integrate
```

如果 `--ff-only` 失败，说明本地 `integrate` 与远程 `integrate` 已经产生分叉。此时不要强行覆盖，应先分析本地提交来源。

### 4. 合入个人分支

合入 `Twin`：

```powershell
git merge Twin
```

合入 `dev`：

```powershell
git merge dev
```

如果发生冲突，先查看冲突文件和冲突类型，不要急于接受任一方全部改动。

### 5. 冲突分析

查看冲突文件：

```powershell
git status --short
git diff --name-only --diff-filter=U
```

冲突处理原则：

- 普通文档、测试、局部实现冲突，可以按最小修改原则解决；
- `CMakeLists.txt`、公共头文件、模块公开数据结构、状态机、通信协议、配置格式冲突，先沟通再改；
- 如果冲突涉及 `AGENTS.md`，不要把它当作业务代码冲突，应按个人约定文件处理；
- 如果冲突涉及 `oldproject/` 或 `RobotSim/`，默认不合入、不修改，先确认这些路径为什么进入变更。

### 6. 最小验证

根据改动范围选择验证方式：

- 只改文档：人工阅读和拼写检查即可；
- 改 CMake 或公共接口：至少重新配置或构建相关目标；
- 改模块公开数据结构、状态机、配置解析、协议解析：运行相关单元测试；
- 改视觉、定位、纠偏、通信或线程流程：运行对应测试，并进行一次最小集成场景检查；
- 涉及真实硬件、真实机器人或真实运动命令时，必须保持 dry-run 或显式授权。

验证通过后，再进入推送步骤。

### 7. 推送 `integrate`

```powershell
git push origin integrate
```

推送前应再次确认当前分支是 `integrate`，并确认本地验证结果可接受。

## 冲突处理规则

冲突按风险分级处理。

可以直接按最小修改解决的冲突：

- 同一文档的说明文字冲突；
- 测试用例增加位置冲突；
- 同一 `.cpp` 中不同局部实现的文本冲突；
- CMake 中新增测试或源文件列表的简单顺序冲突。

必须先沟通再修改的冲突：

- 公共类型、枚举、单位或字段语义冲突；
- 模块公开数据结构冲突；
- 工作流状态机和安全门控冲突；
- 机器人通信协议、RobotSim IPC 协议或配置格式冲突；
- CMake 目标、依赖、开关和第三方库版本冲突；
- 线程所有权、worker 生命周期、UI 与后台线程边界冲突。

处理冲突时，应优先保留模块边界和行为语义，而不是机械拼接两边代码。

## 特殊文件和本地参考目录

`oldproject/` 和 `RobotSim/` 是个人本地参考目录，不要求协作者仓库存在，也不作为协作 Git 内容。

如果需要引用旧项目或 RobotSim 的事实，应把结论写入 `docs/`，而不是要求对方拥有完全相同的参考目录。

根目录 `AGENTS.md` 允许不同独立开发分支不同，各自保留各自版本。它不应作为集成冲突中的业务判断依据。

当前本地 `AGENTS.md` 仍可能被 Git 跟踪。正式协作前，建议单独确认它的跟踪或忽略策略，否则它可能在 `Twin`、`dev`、`integrate` 合并时反复产生冲突。

## 在线纠偏结果共享

`output/online_correction/` 是允许纳入 Git 管理的运行结果目录，用于 Windows 与 Mac 之间共享在线纠偏分析数据。保留原有时间戳会话目录及完整结果文件，不覆盖历史会话。

每次在线纠偏结束并确认结果文件写入完成后，单独检查、暂存并提交新增或修改的会话结果，再推送到对应分支；Mac 拉取同一分支后即可分析。纳入 Git 管理不会自动提交或上传后续运行结果。

会话摘要中的 Windows 绝对路径仅保留为运行来源记录，Mac 分析时应使用仓库内的相对路径；摘要引用的工件、示教和标定文件不会随结果自动同步。除该目录中的分析结果及其来源记录外，其他输出、临时日志和本地机器配置仍不纳入 Git 管理。

## 禁止项

协作过程中禁止：

- 修改、合入或依赖 `oldproject/`；
- 修改、合入或依赖 `RobotSim/`；
- 提交 `out/`、`.vs/`、`vcpkg_installed/`、`logs/`，以及 `output/` 下除 `online_correction/` 分析结果以外的内容；
- 提交个人 `CMakeUserPresets.json`；
- 提交临时日志、调试输出、中间产物或本地机器路径（上述在线纠偏分析结果及其来源记录除外）；
- 未验证就推送到远程 `integrate`；
- 强推远程分支；
- 覆盖或删除对方分支；
- 为了解决冲突而做无关格式化、无关重命名或大范围重排。

## 推荐协作节奏

推荐每天或每完成一个小功能同步一次 `integrate`。

不要长期让 `Twin` 和 `dev` 彼此隔离。分支隔离时间越长，公共接口、CMake、状态机和配置冲突越难处理。

共享模块变更应尽量小步提交，并在合入说明中写清楚影响范围。

如果某项工作会大幅修改 `ui`、`RobotComm`、`vision`、`localization`、`control`、状态机或 CMake，应先在文档或沟通中说明计划，再开始大范围实现。

## 合入说明建议

合入到 `integrate` 后，建议记录以下信息：

- 来源分支；
- 主要改动；
- 涉及模块；
- 是否修改公共接口；
- 是否修改 CMake 或依赖；
- 是否修改配置、协议或状态机；
- 已完成的验证；
- 仍需对方关注的风险。

示例：

```text
来源分支：Twin
主要改动：新增 RobotSim IPC facade 的接口草案
涉及模块：digital_twin, RobotComm, localization
公共接口：是，新增 DigitalTwinState 字段
CMake/依赖：否
配置/协议/状态机：协议字段为草案，需协作者确认
验证：文档审查，通过现有构建
风险：dev 分支如同时修改 RobotComm 或 digital_twin，需要合并前确认协议命名
```

## GitHub HTTPS 推送超时处理（2026-10-20 本机实测）

本次故障表现为 VS Code 推送约 199 秒后返回 `RPC failed; HTTP 408`、`unexpected disconnect` 和 `remote end hung up unexpectedly`，并伴随 `User cancelled dialog`。末尾即使出现 `Everything up-to-date`，也不能据此确认成功，应核对远程分支 SHA。

### 已验证结果与当前首选方式

- 本机 Git Credential Manager 保存多个 GitHub 账号。未指定账号的查询停在凭据选择环节；明确指定仓库所属账号 `Diosenz` 后，查询远程及推送预检查成功。因此不能仅凭取消对话框断言没有可用凭据。
- 使用现有本机代理时，默认 HTTP 协议和 HTTP/1.1 的推送预检查均成功，分别约 1.75 秒、1.38 秒。预检查不上传对象，耗时差异不能证明大数据传输性能优劣。
- 临时绕过代理、使用 HTTP/1.1 的预检查触发低速保护，约 15.44 秒失败。因此这台机器当前继续使用已配置代理，不默认清除全局代理。
- 使用现有代理、明确账号、HTTP/1.1，实际推送 `74311f6` 成功：压缩传输约 34.63 MiB，远端确认 `8880f5b..74311f6 main -> main`。未增大 `http.postBuffer`，未关闭 TLS 证书校验，未重写提交历史。
- 以上组合是当前环境已经验证可用的首选方案；未进行相同大包的多次受控对照，不能断言 HTTP/1.1 是唯一原因或所有网络环境中的最优协议。

本仓库的本地配置（保存在 `.git/config`，不会随提交传播）已设置：

```powershell
git config --local credential.https://github.com.username Diosenz
git config --local http.version HTTP/1.1
```

账号设置仅适用于本机已有 `Diosenz` 凭据且有仓库权限的情况；协作者应使用自己的授权账号。代理地址依机器而定，不写进共享 Git 配置。

### 后续处理顺序

1. 查看当前分支、工作区和客户端 Git 日志，先区分认证失败、连接超时、远端分叉、文件限制及本地锁问题，不能把所有失败都归为代码问题。
2. 使用已授权账号查询远程分支，再做预检查。下例对应当前 `main`，其他分支必须替换分支名；本段不改变本文既有的集成验证要求。

   ```powershell
   git ls-remote origin refs/heads/main
   git push --dry-run origin main:main
   ```

3. 完成本地检查后执行实际推送。必要时临时启用低速保护，连续 45 秒低于 1024 字节/秒会退出，便于诊断；它不是总时限，也不能修复服务端返回的 HTTP 408。

   ```powershell
   git -c http.lowSpeedLimit=1024 -c http.lowSpeedTime=45 push --progress origin main:main
   ```

4. 推送返回成功后，分别读取本地与远程 SHA，确认一致；仅观察客户端提示或本地缓存引用不足以证明远程已更新。

   ```powershell
   git rev-parse HEAD
   git ls-remote origin refs/heads/main
   git status --short --branch
   ```

5. 若仍有 408，逐项测试代理/直连及 HTTP 版本，每次失败先核对远端 SHA，避免盲目重试。`--dry-run` 只验证连接和推送协商，不能验证大包上传。当前本机直连失败，不作为默认回退方案。
6. 仅有分块传输与代理不兼容的证据时，临时用 `-c http.postBuffer=<字节数>` 测试足够覆盖实际压缩包的缓冲区；成功后仍需核对 SHA。不要默认全局增至数百 MiB。SSH 443 可作候选，但本机没有现成 GitHub SSH 密钥，本次没有测试或切换。

### 依据

- [Git 官方 http.postBuffer 说明](https://git-scm.com/docs/git-config#Documentation/git-config.txt-httppostBuffer)：增大缓冲区主要用于禁用分块传输，并非多数推送问题的通用修复，会增加内存使用。
- [Git 官方 http.version 说明](https://git-scm.com/docs/git-config#Documentation/git-config.txt-httpversion)：支持指定 HTTP/1.1 或 HTTP/2，实际支持依赖客户端 libcurl。
- [GitHub 官方连接排障](https://docs.github.com/en/get-started/using-github/troubleshooting-connectivity-problems)：检查代理、防火墙和网络，必要时切换连接方式。
- [GitHub 官方 SSH 443 方案](https://docs.github.com/en/authentication/troubleshooting-ssh/using-ssh-over-the-https-port)：使用 `ssh.github.com:443`，仍需有效 SSH 身份认证。
- [相同 408 错误的用户原始案例](https://github.com/orgs/community/discussions/49789)：包含换网络成功及增大缓冲区建议，仅作案例线索，不作为统一结论。
