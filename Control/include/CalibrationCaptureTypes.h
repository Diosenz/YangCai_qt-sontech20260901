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
 * @file        CalibrationCaptureTypes.h
 * @brief       标定数据采集流程数据结构
 * @details     定义 Calibration 数据采集流程在新工程中的状态、选项和结果。
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
#ifndef I2R_RLC_CONTROL_CALIBRATION_CAPTURE_TYPES_H
#define I2R_RLC_CONTROL_CALIBRATION_CAPTURE_TYPES_H

#include <string>
#include <vector>

#include "CalibrationCommand.h"

namespace i2r_rlc::control {

/**
 * @brief 标定采集模式。
 */
enum class CalibrationCaptureMode {
    /// @brief 人工点击采集标定板和激光图像。
    Manual,
    /// @brief 由机器人标定握手信号触发采集。
    Automatic
};

/**
 * @brief 标定采集阶段。
 */
enum class CalibrationCapturePhase {
    /// @brief 空闲，尚未进入采集流程。
    Idle,
    /// @brief 手动模式下采集标定板图像。
    ManualCapturingBoard,
    /// @brief 手动模式下采集激光图像。
    ManualCapturingLaser,
    /// @brief 自动标定首帧红框预览，等待用户确认后才进入机器人握手。
    AutomaticGuidePreview,
    /// @brief 自动模式下等待机器人标定采集信号。
    AutomaticWaitingRobot,
    /// @brief 自动模式下正在采集标定板图像。
    AutomaticCapturingBoard,
    /// @brief 自动模式下正在采集激光图像。
    AutomaticCapturingLaser,
    /// @brief 所需样本已采集完成。
    Completed,
    /// @brief 输入、文件或握手流程出现错误。
    Error
};

/**
 * @brief 标定采集配置。
 */
struct CalibrationCaptureOptions {
    /// @brief 输出根目录，默认相对当前工作目录的 `Data`。
    std::string outputRootDirectory = "Data";

    /// @brief 旧 two-step 方法目录。
    std::string methodDirectory = "CALID/4 TWO_STEP";

    /// @brief 普通标定板图像数量，后续还会为每张激光图保存一张同位置标定板图。
    int boardImageCount = 15;

    /// @brief 激光图像数量，单位为张。
    int laserImageCount = 4;

    /// @brief true 表示首次采集时覆盖旧 `CalibOriginData.txt`。
    bool overwriteOriginData = false;

    /// @brief true 表示自动标定期间在 UI 图像上显示旧工程红色提示框。
    bool automaticGuideOverlayEnabled = true;

    /// @brief 自动标定提示框左上角 X 坐标，单位 pixel。
    int automaticGuideLeftPixel = 422;

    /// @brief 自动标定提示框左上角 Y 坐标，单位 pixel。
    int automaticGuideTopPixel = 272;

    /// @brief 自动标定提示框右下角 X 坐标，单位 pixel。
    int automaticGuideRightPixel = 860;

    /// @brief 自动标定提示框右下角 Y 坐标，单位 pixel。
    int automaticGuideBottomPixel = 692;

    /// @brief 自动标定提示框线宽，单位 pixel。
    int automaticGuideLineWidthPixel = 3;
};

/**
 * @brief 单个标定采集样本。
 *
 * 位置单位沿用机器人协议 `um`，姿态单位沿用 `deg * 10000`。
 */
struct CalibrationCaptureSample {
    /// @brief 机器人 TCP X 坐标，单位为 um，沿用旧机器人协议。
    long xUm = 0;
    /// @brief 机器人 TCP Y 坐标，单位为 um，沿用旧机器人协议。
    long yUm = 0;
    /// @brief 机器人 TCP Z 坐标，单位为 um，沿用旧机器人协议。
    long zUm = 0;
    /// @brief 机器人 RX 姿态角，单位为 deg * 10000。
    long rxDeg10000 = 0;
    /// @brief 机器人 RY 姿态角，单位为 deg * 10000。
    long ryDeg10000 = 0;
    /// @brief 机器人 RZ 姿态角，单位为 deg * 10000。
    long rzDeg10000 = 0;
    /// @brief 样本图像保存路径，UTF-8 字符串，由控制器写入。
    std::string imagePath;
    /// @brief true 表示激光图像，false 表示标定板图像。
    bool laserImage = false;
    /// @brief 样本是否包含有效图像和机器人位姿。
    bool valid = false;
};

/**
 * @brief 标定采集控制器持有的会话状态。
 */
struct CalibrationCaptureState {
    /// @brief 当前采集模式。
    CalibrationCaptureMode mode = CalibrationCaptureMode::Manual;
    /// @brief 当前状态机阶段。
    CalibrationCapturePhase phase = CalibrationCapturePhase::Idle;
    /// @brief 已采集标定板图像数量，包含后 4 个激光配对位置的标定板图。
    int boardImageCount = 0;
    /// @brief 已采集激光图像数量，单位为张。
    int laserImageCount = 0;
    /// @brief 自动模式下是否正在等待机器人高电平信号释放。
    bool waitingRobotSignalRelease = false;
    /// @brief 当前会话采集到的样本列表，按采集顺序排列。
    std::vector<CalibrationCaptureSample> samples;
    /// @brief 当前输出目录，通常为 Data/CALID/4 TWO_STEP。
    std::string outputDirectory;
    /// @brief 最近一次状态转换或错误说明。
    std::string message;
};

/**
 * @brief 标定采集单次状态更新结果。
 */
struct CalibrationCaptureUpdate {
    /// @brief 当前事件处理是否成功。
    bool succeeded = true;
    /// @brief 事件处理后的状态机阶段。
    CalibrationCapturePhase phase = CalibrationCapturePhase::Idle;
    /// @brief 事件处理后的完整会话快照。
    CalibrationCaptureState state;
    /// @brief 发给机器人标定流程的握手命令 DTO。
    robot_comm::RobotCalibrationCommand robotCommand;
    /// @brief 本轮需要保存的标定板图总数，包含激光配对位置。
    int requiredBoardImageCount = 0;
    /// @brief 本轮需要保存的激光图总数。
    int requiredLaserImageCount = 0;
    /// @brief 建议相机曝光时间，单位为 us。
    int requestedExposureUs = 0;
    /// @brief true 表示调用方应打开激光。
    bool requestLaserOn = false;
    /// @brief true 表示 UI 当前应叠加自动标定红色提示框。
    bool automaticGuideOverlayActive = false;
    /// @brief 自动标定提示框左上角 X 坐标，单位 pixel。
    int automaticGuideLeftPixel = 422;
    /// @brief 自动标定提示框左上角 Y 坐标，单位 pixel。
    int automaticGuideTopPixel = 272;
    /// @brief 自动标定提示框右下角 X 坐标，单位 pixel。
    int automaticGuideRightPixel = 860;
    /// @brief 自动标定提示框右下角 Y 坐标，单位 pixel。
    int automaticGuideBottomPixel = 692;
    /// @brief 自动标定提示框线宽，单位 pixel。
    int automaticGuideLineWidthPixel = 3;
    /// @brief 事件处理说明或错误消息。
    std::string message;
};

} // namespace i2r_rlc::control

#endif // I2R_RLC_CONTROL_CALIBRATION_CAPTURE_TYPES_H
