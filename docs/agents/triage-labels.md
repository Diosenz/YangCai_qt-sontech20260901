# 分诊状态

| 标准角色 | 本地 Status 值 | 含义 |
|---|---|---|
| needs-triage | needs-triage | 待维护者评估 |
| needs-info | needs-info | 等待补充信息 |
| ready-for-agent | ready-for-agent | 规格充分，可交给代理执行 |
| ready-for-human | ready-for-human | 需要人工处理 |
| wontfix | wontfix | 不处理 |

每个已分诊任务保留一个类别和一个分诊状态，例如：

Category: bug
Status: needs-triage

技能提到分诊角色时，使用本表中的对应值。
