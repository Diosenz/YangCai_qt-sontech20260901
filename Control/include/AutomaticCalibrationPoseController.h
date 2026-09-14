/*******************************************************************************
 * @file        AutomaticCalibrationPoseController.h
 * @brief       自动标定位姿文件读取、修正和机器人命令生成
 * @details     复刻旧 Calibration::OnAutocalib / OnPosok 中 CalibPose.txt
 *              的前置下发逻辑，不访问 Qt、相机或 socket。
 * @author      I2R Laboratory
 * @version     1.0
 ******************************************************************************/
#ifndef I2R_RLC_CONTROL_AUTOMATIC_CALIBRATION_POSE_CONTROLLER_H
#define I2R_RLC_CONTROL_AUTOMATIC_CALIBRATION_POSE_CONTROLLER_H

#include <string>
#include <vector>

#include "OnlineCorrectionCommand.h"
#include "RobotState.h"

namespace i2r_rlc::control {

/**
 * @brief 自动标定位姿文件中的一行位姿。
 *
 * 位置单位沿用旧机器人协议 um，姿态单位沿用 deg * 10000。模板文件
 * 可包含变位机 S1/S2 两列；旧 `CalibPose_new.txt` 只写六维机器人位姿。
 */
struct AutomaticCalibrationPose {
    long xUm = 0;
    long yUm = 0;
    long zUm = 0;
    long rxDeg10000 = 0;
    long ryDeg10000 = 0;
    long rzDeg10000 = 0;
    long s1Deg10000 = 0;
    long s2Deg10000 = 0;
    bool stationAxesValid = false;
};

/**
 * @brief 自动标定位姿处理结果。
 *
 * `robotCommand` 只在需要下发机器人示教点时有效；单纯解析失败时通过
 * `message` 返回结构化错误说明。
 */
struct AutomaticCalibrationPoseResult {
    bool succeeded = false;
    std::string message;
    std::string sourceFilePath;
    std::string outputFilePath;
    std::vector<AutomaticCalibrationPose> poses;
    robot_comm::RobotCorrectionCommand robotCommand;
};

/**
 * @brief 自动标定模板位姿控制器。
 *
 * 本类位于 Control 层，只做文件解析、首点对齐修正和 DTO 生成。
 * 真实机器人发送由 UI/RuntimeCoordinator 投递给 RobotComm worker。
 */
class CAutomaticCalibrationPoseController final {
public:
    /**
     * @brief 读取 `CalibPose.txt`，校验旧自动标定所需的 19 个点。
     * @param filePath 模板或修正位姿文件路径。
     * @return 解析出的位姿列表；失败时 `succeeded=false`。
     */
    AutomaticCalibrationPoseResult loadPoseFile(
        const std::string &filePath) const;

    /**
     * @brief 生成旧 `OnAutocalib()` 首次下发模板点位的 O 命令。
     * @param templateFilePath `config/calibration/CalibPose.txt` 路径。
     * @return 包含 `offlineSignal=4` 的 RobotCorrectionCommand。
     */
    AutomaticCalibrationPoseResult makeTemplateCommand(
        const std::string &templateFilePath) const;

    /**
     * @brief 按当前机器人位姿修正模板点位并生成下发命令。
     * @param templateFilePath 原始模板 `CalibPose.txt` 路径。
     * @param outputFilePath 运行时 `CalibPose_new.txt` 输出路径。
     * @param currentState 最近一次有效 D 回包中的当前机器人位姿。
     * @return 写出修正文件并生成 O 命令；失败时不生成命令。
     */
    AutomaticCalibrationPoseResult makeCorrectedCommand(
        const std::string &templateFilePath,
        const std::string &outputFilePath,
        const robot_comm::RobotState &currentState) const;

private:
    robot_comm::RobotCorrectionCommand makeRobotCommand(
        const std::vector<AutomaticCalibrationPose> &poses,
        const std::string &message) const;
};

} // namespace i2r_rlc::control

#endif // I2R_RLC_CONTROL_AUTOMATIC_CALIBRATION_POSE_CONTROLLER_H
