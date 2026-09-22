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
 * @file        CalibrationCaptureController.h
 * @brief       标定数据采集控制器
 * @details     以事件驱动方式承接采集流程。
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
#ifndef I2R_RLC_CONTROL_CALIBRATION_CAPTURE_CONTROLLER_H
#define I2R_RLC_CONTROL_CALIBRATION_CAPTURE_CONTROLLER_H

#include <string>

#include <opencv2/core.hpp>

#include "CalibrationCaptureTypes.h"
#include "RobotState.h"

namespace i2r_rlc::control {

/**
 * @brief 标定数据采集状态机。
 *
 * 本类只负责保存标定采集数据与生成握手 DTO，不执行 HALCON 标定计算，
 * 不访问 Qt、真实相机或 socket。
 */
class CCalibrationCaptureController final {
public:
    /**
     * @brief 配置标定采集参数并重置会话状态。
     * @param options 输出目录、采集张数和旧 CalibOriginData 写入策略。
     * @note 本函数不创建目录、不访问相机，仅更新控制器内存状态。
     */
    void configure(const CalibrationCaptureOptions &options);

    /**
     * @brief 启动手动标定采集流程。
     * @return 状态更新结果，包含建议曝光和激光开关请求。
     */
    CalibrationCaptureUpdate startManualCapture();

    /**
     * @brief 进入自动标定红框预览阶段。
     *
     * 该阶段仅用于 UI 引导用户把首张标定板移动到旧工程红框内，
     * 不发送机器人命令，也不响应机器人采集握手信号。
     */
    CalibrationCaptureUpdate previewAutomaticCapture();

    /**
     * @brief 启动自动标定采集流程。
     * @return 状态更新结果，随后需通过 onRobotStateChanged 输入机器人握手状态。
     */
    CalibrationCaptureUpdate startAutomaticCapture();

    /**
     * @brief 采集一张标定板样本。
     * @param image 当前相机图像，所有权由调用方持有，函数内部只同步写文件。
     * @param robotState 当前机器人状态，位置单位沿用机器人协议 um。
     * @return 采集状态和下一步机器人/相机请求。
     */
    CalibrationCaptureUpdate captureBoardSample(
        const cv::Mat &image,
        const robot_comm::RobotState &robotState);

    /**
     * @brief 采集一张激光样本。
     * @param image 当前相机图像，所有权由调用方持有，函数内部只同步写文件。
     * @param robotState 当前机器人状态，位置单位沿用机器人协议 um。
     * @return 采集状态和下一步机器人/相机请求。
     */
    CalibrationCaptureUpdate captureLaserImage(
        const cv::Mat &image,
        const robot_comm::RobotState &robotState);

    /**
     * @brief 删除当前会话中最后一个样本记录。
     * @return 删除后的状态快照；当前不删除磁盘图像，兼容旧工程撤销语义。
     */
    CalibrationCaptureUpdate deleteLastSample();

    /**
     * @brief 输入机器人状态变化，驱动自动采集握手。
     * @param image 当前相机图像，机器人触发采集时写入磁盘。
     * @param robotState 当前机器人状态，s_Bcalib 用作自动采集触发信号。
     * @return 状态更新结果。
     */
    CalibrationCaptureUpdate onRobotStateChanged(
        const cv::Mat &image,
        const robot_comm::RobotState &robotState);

    /**
     * @brief 结束采集流程并检查样本数量是否完整。
     * @return 完整时进入 Completed，否则返回错误状态。
     */
    CalibrationCaptureUpdate finishCapture();

    /**
     * @brief 重置会话状态，保留 configure 设置的选项。
     * @return 重置后的状态快照。
     */
    CalibrationCaptureUpdate reset();

    /**
     * @brief 获取当前会话状态。
     * @return 控制器内部状态的只读引用，调用方不得长期保存超过控制器生命周期。
     */
    const CalibrationCaptureState &state() const;

private:
    /**
     * @brief 基于当前状态生成统一更新 DTO。
     * @param succeeded 当前事件是否成功。
     * @param message 事件说明或错误消息。
     * @return 带曝光和激光请求的状态更新。
     */
    CalibrationCaptureUpdate makeUpdate(bool succeeded, const std::string &message) const;

    /**
     * @brief 进入 Error 阶段并生成失败更新。
     * @param message 错误消息。
     * @return 失败状态更新。
     */
    CalibrationCaptureUpdate makeError(const std::string &message);

    /**
     * @brief 设置状态机阶段和说明文本。
     * @param phase 新阶段。
     * @param message 状态说明。
     */
    void setPhase(CalibrationCapturePhase phase, const std::string &message);

    /**
     * @brief 确保标定输出目录存在。
     * @param errorMessage 可选错误输出。
     * @return 目录存在或创建成功时返回 true。
     */
    bool ensureOutputDirectory(std::string *errorMessage) const;

    /**
     * @brief 按旧格式初始化 CalibOriginData.txt 头部。
     * @param errorMessage 可选错误输出。
     * @return 文件可写且头部已准备时返回 true。
     */
    bool writeOriginHeaderIfNeeded(std::string *errorMessage);

    /**
     * @brief 追加一行机器人原始标定位姿。
     * @param robotState 机器人状态，位置单位 um，姿态单位 deg * 10000。
     * @param errorMessage 可选错误输出。
     * @return 写入成功时返回 true。
     */
    bool appendOriginDataLine(
        const robot_comm::RobotState &robotState,
        std::string *errorMessage);

    /**
     * @brief 按外部标定软件格式写出本次会话的 CalibResult.txt。
     * @param errorMessage 可选错误输出。
     * @return 19 张标定板图与 TCP 位姿一一对应并写入成功时返回 true。
     * @note 激光图与后 4 张标定板图共用位姿，不单独写入数组。
     */
    bool writeCalibrationResult(std::string *errorMessage) const;

    /**
     * @brief 保存一张标定图像。
     * @param image 输入图像，所有权由调用方持有。
     * @param fileName 文件名，不含目录。
     * @param outputPath 可选输出完整路径。
     * @param errorMessage 可选错误输出。
     * @return 图像写入成功时返回 true。
     */
    bool saveImage(
        const cv::Mat &image,
        const std::string &fileName,
        std::string *outputPath,
        std::string *errorMessage) const;

    /**
     * @brief 执行标定板或激光样本的通用采集流程。
     * @param image 当前图像。
     * @param robotState 当前机器人位姿。
     * @param laserImage true 表示保存激光样本。
     * @return 采集状态更新。
     */
    CalibrationCaptureUpdate captureSample(
        const cv::Mat &image,
        const robot_comm::RobotState &robotState,
        bool laserImage);

    /**
     * @brief 当前配置要求保存的标定板图总数。
     * @return 普通标定板图数量加上后续激光配对位置数量。
     */
    int totalBoardImageCount() const;

    /**
     * @brief 判断当前会话是否已完成所有板图和激光图采集。
     * @return 全部样本采集完成时返回 true。
     */
    bool isCaptureComplete() const;

    /**
     * @brief 判断当前机器人位置是否已保存板图但尚未保存对应激光图。
     * @return 处于后 4 个配对位置且缺少同编号激光图时返回 true。
     */
    bool isPairedLaserImagePending() const;

    /**
     * @brief 生成请求机器人移动到下一个标定点的命令。
     * @return 机器人标定命令 DTO。
     */
    robot_comm::RobotCalibrationCommand makeNextPositionCommand() const;

    CalibrationCaptureOptions m_options;
    CalibrationCaptureState m_state;
    bool m_originDataInitialized = false;
};

} // namespace i2r_rlc::control

#endif // I2R_RLC_CONTROL_CALIBRATION_CAPTURE_CONTROLLER_H
