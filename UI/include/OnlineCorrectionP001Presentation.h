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
 * 本类只保存 `OnlineCorrectionUpdate::command` 中的原始整数 DTO。单位换算仅在
 * 生成展示文本时发生，不参与轨迹计算或机器人协议发送。
 */
class COnlineCorrectionP001Presentation {
public:
    /** @brief 清除本轮锁存值，恢复“未计算”。 */
    void clear();

    /**
     * @brief 消费在线纠偏状态更新并维护 P001 锁存与写入状态。
     *
     * 空命令不会清除已锁存坐标；Idle 和 InitialDetectionRunning 表示新会话边界。
     */
    void update(const control::OnlineCorrectionUpdate &update);

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
    WriteState m_writeState = WriteState::NotCalculated;
    bool m_hasP001 = false;
};

} // namespace i2r_rlc::ui

#endif // I2R_RLC_UI_ONLINE_CORRECTION_P001_PRESENTATION_H
