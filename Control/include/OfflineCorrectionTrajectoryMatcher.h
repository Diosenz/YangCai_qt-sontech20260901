/*******************************************************************************
 *       ______   ______   _______         __                  __
 *      /      | /      \ /       \       /  |                /  |
 *      $$$$$$/ /$$$$$$  |$$$$$$$  |      $$ |        ______  $$ |____
 *        $$ |  $$____$$ |$$ |__$$ |      $$ |       /      \ $$      \
 *        $$ |   /    $$/ $$    $$<       $$ |       $$$$$$  |$$$$$$$  |
 *        $$ |  /$$$$$$/  $$$$$$$  |      $$ |       /    $$ |$$ |  $$ |
 *       _$$ |_ $$ |_____ $$ |  $$ |      $$ |_____ /$$$$$$$ |$$ |__$$ |
 *      / $$   |$$       |$$ |  $$ |      $$       |$$    $$ |$$    $$/
 *      $$$$$$/ $$$$$$$$/ $$/   $$/       $$$$$$$$/  $$$$$$$/ $$$$$$$/
 *
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        OfflineCorrectionTrajectoryMatcher.h
 * @brief       离线纠偏轨迹匹配器
 * @details     读取存档轨迹，执行整体 ICP 并生成机器人示教点更新命令。
 *
 * @version     1.1
 * @author      LT
 * @date        2026 年 6 月 6 日
 *
 * @par         修改日志
 * <table>
 * <tr><th>Version  <th>Author      <th>Date        <th>Description         </tr>
 * <tr><td>1.0      <td>LT          <td>260606      <td>创建文件            </tr>
 * <tr><td>1.1      <td>LT          <td>260606      <td>初步完成            </tr>
 * </table>
 *
 ******************************************************************************/
#ifndef I2R_RLC_CONTROL_OFFLINE_CORRECTION_TRAJECTORY_MATCHER_H
#define I2R_RLC_CONTROL_OFFLINE_CORRECTION_TRAJECTORY_MATCHER_H

#include <array>
#include <string>
#include <vector>

#include "OfflineCorrectionTypes.h"

namespace i2r_rlc::control {

/**
 * @brief 离线纠偏存档文件解析结果。
 */
struct OfflineCorrectionArchive {
    /// @brief true 表示 _W/_T 文件均成功解析。
    bool valid = false;
    /// @brief 模型轨迹文件路径，通常为 *_W.txt。
    std::string modelFilePath;
    /// @brief 示教点文件路径，通常为 *_T.txt。
    std::string teachFilePath;
    /// @brief 旧工程模型轨迹点，坐标单位为 mm。
    std::vector<i2r_rlc::vision::WeldSpacePoint> modelPoints;
    /// @brief 示教点位置，坐标单位为 mm。
    std::vector<i2r_rlc::vision::WeldSpacePoint> teachPositions;
    /// @brief 示教点姿态角，x/y/z 分别表示 RX/RY/RZ，单位为 deg。
    std::vector<i2r_rlc::vision::WeldSpacePoint> teachOrientationsDeg;
    /// @brief 与示教点对应的变位机轴更新命令，单位为 deg * 10000。
    std::vector<robot_comm::StationAxisUpdateCommand> stationAxes;
    /// @brief 解析说明或错误消息。
    std::string message;
};

/**
 * @brief 单次离线 ICP 的可持久化诊断快照。
 *
 * 快照只包含本次匹配实际使用或生成的数据，不执行文件写入。Runtime 在机器人
 * 确认全部示教点后将其覆盖保存，避免失败运行替换上一份成功记录。
 */
struct OfflineCorrectionDiagnosticData {
    /// @brief true 表示本快照包含完整 ICP 诊断数据。
    bool valid = false;
    /// @brief 模型轨迹文件路径。
    std::string modelFilePath;
    /// @brief 示教点文件路径。
    std::string teachFilePath;
    /// @brief 当前实现的模型到检测点齐次变换，按行保存，平移单位为 mm。
    std::array<double, 16> modelToDetectionTransformRowMajor{};
    /// @brief 清洗后实际参与 ICP 的检测点，单位为 mm。
    std::vector<i2r_rlc::vision::WeldSpacePoint> detectionPoints;
    /// @brief ICP 变换后的模型轨迹点，单位为 mm。
    std::vector<i2r_rlc::vision::WeldSpacePoint> transformedModelPoints;
    /// @brief ICP 变换后的示教点位置，单位为 mm。
    std::vector<i2r_rlc::vision::WeldSpacePoint> transformedTeachPoints;
};

/**
 * @brief 离线纠偏轨迹匹配结果。
 */
struct OfflineCorrectionTrajectoryMatchResult {
    /// @brief true 表示匹配和命令生成成功。
    bool valid = false;
    /// @brief ICP 匹配质量报告。
    OfflineCorrectionMatchReport report;
    /// @brief 本次匹配的诊断快照，待机器人确认后持久化。
    OfflineCorrectionDiagnosticData diagnostics;
    /// @brief 生成给机器人通信层的示教点更新命令。
    robot_comm::RobotCorrectionCommand robotCommand;
    /// @brief 匹配说明或错误消息。
    std::string message;
};

/**
 * @brief 离线纠偏轨迹匹配器。
 */
class COfflineCorrectionTrajectoryMatcher final {
public:
    /**
     * @brief 读取离线纠偏存档及其配套模型/示教文件。
     * @param archiveFilePath 用户选择的 _W.txt、_T.txt 或同名前缀路径。
     * @return 解析结果，坐标统一换算到 mm，姿态统一换算到 deg。
     */
    static OfflineCorrectionArchive loadArchive(const std::string &archiveFilePath);

    /**
     * @brief 使用检测点和存档模型点执行 ICP 匹配，并生成示教点更新命令。
     * @param archive 已解析的离线存档。
     * @param detectionSamples 视觉检测点，坐标单位为 mm。
     * @param options ICP、检测点丢弃和平滑配置。
     * @return 匹配结果和机器人命令，失败时 valid 为 false。
     */
    static OfflineCorrectionTrajectoryMatchResult match(
        const OfflineCorrectionArchive &archive,
        const std::vector<OfflineCorrectionDetectionSample> &detectionSamples,
        const OfflineCorrectionOptions &options = {});
};

} // namespace i2r_rlc::control

#endif // I2R_RLC_CONTROL_OFFLINE_CORRECTION_TRAJECTORY_MATCHER_H
