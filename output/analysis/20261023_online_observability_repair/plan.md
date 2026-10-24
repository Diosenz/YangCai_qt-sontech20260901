# 在线纠偏方案二实施（2026-10-23）

约束：不修改RobotSim/oldproject及外部发布文件，不删除文件，不使用Computer Use/鼠标；保留有效反光辅助、TCP连续估计及禁止自动回退。只改当前在线纠偏调用链。

工作流依据：已读Notion本地正文快照 output/analysis/20261021_kcf_right_bias_investigation/notion_workflow_snapshot.json 共用协作与修bug页；本轮在线读取未取得正文，不声称已核验在线版本。源文件先备份，实际行为RED/GREEN，文件独占分工，主agent整合+独立diff审查。

1. [完成] 固定当前基线/备份、明确数据身份及现有CAD能力，补充针对性RED回归。
2. [完成] 双肩可观测性/异常片段运动与恢复时间语义修复；业务过渡样本身份与诊断接线。
3. [完成] 当前项目只读STEP候选重新求交；显式场景绑定及当前几何一致性保护；受约束候选评价与已有门禁整合。
4. [完成] Windows/MSVC现有build tree验证、泛化回归及独立审查，更新已有文档和证据清单。

分工：estimator_repair拥有SimulationGuidedSeamEstimator及其测试；detector_integration拥有OnlineSeamContinuity/LegacyOnlineKcfDetector及连续测试；generality_audit拥有新增OnlineSeamCandidateGeometry及测试；主agent拥有UI整合/配置/CMake/docs/统一验证。

已知边界：本地profile旧weldment与实际RobotSim模型不同，不能直接作为真值；必须显式scene_xml_file、对象/父级/传感器/焊缝身份，并在当前姿态复现服务参考。候选评价失败不得无保护放行。机器人实际运行与现场完整帧回放未授权/数据不完整，不能用合成测试宣称现场通过。
最终结果：主程序MSVC构建成功（final_build.log）。24个不同CTest项23通过，1个既有离线frame_sync夹具与启动预检契约冲突未修改，详review_findings.md；九项专项全通过76.71秒，最终五项时间边界/合法半步/原图合同31.38秒全通过。半步局部128点/帧用例329ms，生产2500ms预算不变。
未启动设备/主程序/RobotSim，不声称现场通过。原有五秒后验、冷却及新曝光门禁保留，多交点/身份未确认保留旧门；新几何缓存不自动热载。修改清单SHA256见changed_files.json；最终可执行文件SHA256见validation_summary.json。
