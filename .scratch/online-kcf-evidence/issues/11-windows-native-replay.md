# 11: Windows/MSVC 离线回放构建与分支交付

Category: enhancement
Status: ready-for-agent
Resolution: partial
Updated: 2026-10-25

## Goal

在既有 JSONL → 组合报告公开接口上支持 VS2022/MSVC 预编译原生回放程序；
用户在 Windows 拉取独立验证分支后可通过 VS 构建并运行回放。保持生产行为不变。

## Acceptance criteria

- [x] 六个离线原生程序进入可选 CMake 目标，沿用公共编译选项，AUTOMOC OFF。
- [x] Python 显式选择已编译程序，Windows 不调用 clang/pkg-config 或自动发起生成。
- [x] 程序、源码哈希及 Qt/OpenCV/编译器信息可追溯；缺失或过期产物保守失败。
- [x] 提供 Windows 拉取、VS 构建、运行库路径、公开 CLI 与回归操作说明。
- [x] 完成可执行的辅助验证并推送独立验证分支，核对远端 SHA。
- [ ] 真实 Windows/MSVC 构建与运行由 Windows 端验证并记录，不以 macOS 验证替代。

## Comments

- 2026-10-25：用户授权准备 Windows 离线构建入口、验证并推送验证分支。
- 测试沿用用户已确认的公开 CLI/JSONL → 报告边界。新增部署检查仅覆盖原生程序选择及错误处理。
- 禁止删除文件，不改生产模块及只读目录，不提交编译缓存、机器路径或生成输出，不自动运动。

- 2026-10-25：AppleClang/Unix Makefiles 辅助构建六程序；CTest 两入口通过，含62项公开回归及12项原因果探针。运行PATH隔离为Git及清单运行库目录，未提供编译器/pkg-config。首次缓存目录测试先红后绿。Standards/Spec无未解决问题。真实MSVC/OpenCV3.4.3构建仍待Windows验收。

- 2026-10-25：Windows入口源码80b69c0c6e2473381d303b64771da61855a8c015已推送至origin/codex/kcf-evidence-integration并通过直接远端SHA核对。main未修改；本事项仅因真实Windows/MSVC构建与运行尚待用户端验收而保持partial。
