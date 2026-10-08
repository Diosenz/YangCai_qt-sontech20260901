/**
 * @file        OnlineCorrectionP001Presentation.h
 * @brief       在线纠偏最终 P001 的 UI 展示状态
 */
#ifndef I2R_RLC_UI_ONLINE_CORRECTION_P001_PRESENTATION_H
#define I2R_RLC_UI_ONLINE_CORRECTION_P001_PRESENTATION_H

#include <QString>

#include "OnlineCorrectionTypes.h"

namespace i2r_rlc::ui {

/**
 * @brief 锁存准备写入 RobotComm 的首个示教点与变位机轴，并生成 UI 文本。
 *
 * 原始整数 DTO 与同一目标的变位机固定基座位姿在首点生成时一起锁存。
 * 坐标转换仅供 UI 展示，不参与轨迹计算或机器人协议发送。
 */
class COnlineCorrectionP001Presentation {
public:
    /** @brief 清除本轮锁存值，恢复“未计算”。 */
    void clear();

    /**
     * @brief 消费在线纠偏状态更新并维护 P001 锁存与写入状态。
     *
     * 空命令不会清除已锁存坐标；Idle 和 InitialDetectionRunning 表示新会话边界。
     * 标定仅在首点锁存时使用；固定基座显示不包含外轴旋转和工装 TCP 偏移。
     */
    void update(
        const control::OnlineCorrectionUpdate &update,
        const vision::PositionerCalibration &calibration);

    /** @brief 返回可直接显示在在线纠偏状态区域的 P001 文本。 */
    QString text() const;

private:
    enum class WriteState {
        NotCalculated,
        PendingConfirmation,
        Written,
        WriteFailed
    };

    robot_comm::TeachPointUpdateCommand m_teachPoint;
    robot_comm::StationAxisUpdateCommand m_stationAxis;
    vision::PositionerPose m_stationBasePose;
    WriteState m_writeState = WriteState::NotCalculated;
    bool m_hasP001 = false;
};

} // namespace i2r_rlc::ui

#endif // I2R_RLC_UI_ONLINE_CORRECTION_P001_PRESENTATION_H
