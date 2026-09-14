# CommCamera vendor 接入说明

## 接入范围

本项目将 `C:\Code\Camera\CommCamera` 作为第三方/协作方相机通信模块接入。
本阶段只纳入 Allied/Vimba X 与 Basler/Pylon 后端，不纳入 Daheng/Galaxy 与
Hikvision/MVS 后端。

vendor 源码放在 `Camera/vendor/CommCamera/`，保留对方模块命名空间和实现风格。
当前项目只在外层维护 CMake 开关、适配类和 smoke test。
`Camera/vendor/CommCamera/compat` 只保存当前项目为 OpenCV 3.4.3 提供的 include
兼容 shim，不改动对方原始实现文件。

## 构建开关

- `I2R_RLC_ENABLE_VENDOR_CAMERAS`：总开关，默认关闭。
- `I2R_RLC_ENABLE_CAMERA_ALLIED`：启用 Allied/Vimba X 后端，默认关闭。
- `I2R_RLC_ENABLE_CAMERA_BASLER`：启用 Basler/Pylon 后端，默认关闭。

默认关闭时，不查找 Vimba X、Pylon 或 OpenCV，也不编译 vendor 相机代码。

## 依赖策略

CommCamera 外部仓库的 `vcpkg.json` 未导入本项目。当前项目继续沿用 OpenCV 3.4.3
策略，不通过 vcpkg 引入 `opencv4`。

启用 Allied 后端时，CMake 通过 `$ENV{VIMBA_X_HOME}/api` 查找 `Vmb::CPP`。
如果未设置该环境变量，也会尝试使用开发者环境说明中的默认安装路径：
`C:/Program Files/Allied Vision/Vimba X/api`。Windows 测试运行时会把存在的
`api/bin` 临时加入 `PATH`。

启用 Basler 后端时，CMake 通过 `find_package(pylon REQUIRED)` 查找
`pylon::pylon`。如果未显式设置 `pylon_DIR`，会尝试 `PYLON_ROOT`、
`BASLER_PYLON_ROOT` 以及开发者环境说明中的默认路径：
`C:/Program Files/Basler/pylon/Development/CMake`。Windows 测试运行时会把存在的
`C:/Program Files/Basler/pylon/Runtime/x64` 临时加入 `PATH`，并为 smoke test
临时设置 `GENICAM_GENTL64_PATH`。

开发者文档的 CommCamera 部分提到参考或合并 `CommCamera/vcpkg.json`。本项目已在根
`vcpkg.json` 保留 `fmt` 与 `magic-enum`，但不导入对方 `opencv4`，因为当前 Qt
工程要求继续复用外部 OpenCV 3.4.3。

## 当前边界

`AlliedCameraDevice` 和 `BaslerCameraDevice` 是当前项目自己的 `ICameraDevice`
适配层，公共头不暴露厂商 SDK、OpenCV 或 CommCamera 头。离线相机流程保持不变，
真实相机采集验证需在后续连接对应硬件后单独执行。
