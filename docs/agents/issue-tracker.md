# 问题跟踪：本地 Markdown

本仓库的问题、规格与实施任务保存在 `.scratch/`。

## 文件约定

- 每个功能一个目录：`.scratch/<feature-slug>/`。
- 功能规格：`spec.md`。
- 实施任务：`issues/<NN>-<slug>.md`，从 `01` 开始编号，
  每个任务独立成文件。
- 任务文件顶部使用 `Status:` 记录分诊状态，
  名称见 `triage-labels.md`。
- 使用 `Category: bug` 或 `Category: enhancement` 记录类别。
- 讨论历史追加到文件末尾的 `## Comments`，保留已有记录。

## 技能操作映射

- “发布到问题跟踪器”：创建对应的本地 Markdown 文件。
- “读取相关任务”：读取用户提供的任务路径；
  仅给编号时，在当前功能目录中定位。
- 更新任务前先读取现有内容，保留规格、证据与讨论历史。
- 本地任务操作不自动创建 GitHub Issue。
