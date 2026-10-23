/**
 * @file OnlineLocalRegistrationRecovery.h
 * @brief 在线写后事务结案的冷却及新曝光边界；仅 coordinator 线程持有。
 */
#ifndef I2R_RLC_UI_ONLINE_LOCAL_REGISTRATION_RECOVERY_H
#define I2R_RLC_UI_ONLINE_LOCAL_REGISTRATION_RECOVERY_H

#include <QtGlobal>

namespace i2r_rlc::ui {

/** @brief 只管理保留/显式恢复后的采样资格，不清累计预算、不访问设备或晋升位姿。 */
class COnlineLocalRegistrationRecovery final {
public:
    /** @brief 回滚命令、归档和 worker 事务快照共同证明恢复已确认。 */
    struct Outcome {
        bool commandSucceeded = false;
        bool archived = false;
        bool rollbackAttempted = false;
        bool hasVerifiedPose = false;
        bool hasPendingPose = true;
        bool pendingPoseConfirmed = false;
    };

    /** @brief 保留当前已确认位姿的结案结果；不要求或伪造恢复尝试标记。 */
    struct RetainedOutcome {
        bool commandSucceeded = false;
        bool archived = false;
        bool hasConfirmedPose = false;
        bool hasPendingPose = true;
    };

    /** @brief 新会话显式清除旧边界；普通清窗和热调不能缩短恢复冷却。 */
    void reset() { *this = COnlineLocalRegistrationRecovery{}; }

    /**
     * @brief 回滚结束时冻结单调冷却截止与曝光/帧号下界；未知结果保持禁止采样。
     * @details nowMs 是会话单调时钟，captureAfterMs 是恢复完成的曝光时间域；
     * 两者不可混用。只有恢复及事务归档均成功，且 worker 不再有待确认位姿才返回 true。
     */
    bool complete(const Outcome &outcome, qint64 nowMs, qint64 captureAfterMs,
        qint64 frameAfter, int cooldownMs)
    {
        m_active = true;
        m_confirmed = outcome.commandSucceeded && outcome.archived
            && outcome.rollbackAttempted && outcome.hasVerifiedPose
            && !outcome.hasPendingPose && !outcome.pendingPoseConfirmed
            && nowMs >= 0 && captureAfterMs >= 0 && frameAfter >= 0 && cooldownMs >= 0;
        m_startedAtMs = nowMs;
        m_cooldownMs = cooldownMs;
        m_captureAfterMs = captureAfterMs;
        m_frameAfter = frameAfter;
        return m_confirmed;
    }

    /**
     * @brief 不发恢复命令的保留结案：仅成功回读且归档后允许冷却并使用新曝光。
     * @details 当前位姿不晋升几何可信，未知写入、未归档或pending未结束保持禁止采样。
     */
    bool completeRetained(const RetainedOutcome &outcome, qint64 nowMs,
        qint64 captureAfterMs, qint64 frameAfter, int cooldownMs)
    {
        m_active = true;
        m_confirmed = outcome.commandSucceeded && outcome.archived
            && outcome.hasConfirmedPose && !outcome.hasPendingPose
            && nowMs >= 0 && captureAfterMs >= 0 && frameAfter >= 0 && cooldownMs >= 0;
        m_startedAtMs = nowMs;
        m_cooldownMs = cooldownMs;
        m_captureAfterMs = captureAfterMs;
        m_frameAfter = frameAfter;
        return m_confirmed;
    }

    /** @brief 冷却后仍须新曝光及递增帧号；参数为调用方已校核的当前会话时间。 */
    bool allowsSample(qint64 nowMs, qint64 captureTimeMs, qint64 frameId) const
    {
        return !m_active || (m_confirmed && nowMs >= m_startedAtMs
            && nowMs - m_startedAtMs >= m_cooldownMs
            && captureTimeMs > m_captureAfterMs && frameId > m_frameAfter);
    }

private:
    bool m_active = false;
    bool m_confirmed = false;
    qint64 m_startedAtMs = -1;
    int m_cooldownMs = 0;
    qint64 m_captureAfterMs = -1;
    qint64 m_frameAfter = -1;
};

} // namespace i2r_rlc::ui
#endif // I2R_RLC_UI_ONLINE_LOCAL_REGISTRATION_RECOVERY_H
