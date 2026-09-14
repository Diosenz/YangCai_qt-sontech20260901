# Third-Party Notices

本文档记录 `newSonTech` 随源码携带的第三方代码。新增或升级第三方代码时，
需要同步更新许可证、用途和构建边界。

## shape_based_matching

- Source: https://github.com/meiqua/shape_based_matching
- License: BSD 2-Clause License
- Local path: `Vision/third_party/shape_based_matching`
- Usage: 作为 ImageProFeng `TemplateMatching_change` 的 OpenCV 形状模板匹配
  替代后端，用于 `SimulatedLaserTemplateMatcher` 的默认模板匹配路径。
- Notes: 只迁入运行所需源码和许可证，不迁入论文 PDF、示例图片和上游测试程序。

Required notice text is preserved in
`Vision/third_party/shape_based_matching/LICENSE`.

## MIPP

- Source: bundled by shape_based_matching, upstream project
  https://github.com/aff3ct/MIPP
- License: MIT License
- Local path: `Vision/third_party/shape_based_matching/MIPP`
- Usage: shape_based_matching 的 SIMD 抽象层。当前工程不定义
  `MIPP_NO_INTRINSICS`，允许在 MSVC/OpenCV 构建中使用可用的 SIMD 指令。

Required notice text is preserved in
`Vision/third_party/shape_based_matching/MIPP/mipp.h`.

## libicp

- Source: legacy project copy of Andreas Geiger's libicp, originally from
  Karlsruhe Institute of Technology.
- License: GNU General Public License, version 2 or later in source headers;
  the bundled README states GPL version 3 or later. Treat this vendored code as
  GPL-covered unless project-level legal review decides otherwise.
- Local path: `Vision/third_party/libicp`
- Usage: 承接旧工程 `robotleading_method/libicp/src` 的 point-to-point、
  point-to-plane、scale、affine、BDD ICP 内核，由 `CIcpRegistrationMatcher`
  封装后供 Vision 使用。
- Distribution note: 上游 README 明确说明，分发使用 libicp 的软件时需要按
  GPL 分发源代码。本工程接受该影响后才应发布包含该模块的二进制版本。

Original copyright and GPL notice text is preserved in the vendored source files
under `Vision/third_party/libicp`.

## libicp kd-tree helper

- Source: Matthew B. Kennel kd-tree implementation bundled by libicp.
- License: Academic Free License 1.1 as stated in the source comments.
- Local path: `Vision/third_party/libicp/kdtree.h` and
  `Vision/third_party/libicp/kdtree.cpp`
- Usage: libicp 内部最近邻搜索；不作为 Vision 公共 API 暴露。

