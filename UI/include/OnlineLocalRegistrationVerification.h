/** @file OnlineLocalRegistrationVerification.h
 * @brief 写回后使用新帧独立焊点检查效果；不访问设备。
 */
#ifndef I2R_RLC_UI_ONLINE_LOCAL_REGISTRATION_VERIFICATION_H
#define I2R_RLC_UI_ONLINE_LOCAL_REGISTRATION_VERIFICATION_H

#include <cmath>

#include "LocalizationTypes.h"

namespace i2r_rlc::ui {

/**
 * @brief 单次写回的后验检查器，只在 coordinator 线程串行使用。
 * @details 独立 KCF 观测仍可能错误；连续有效新帧仅提供交叉证据。
 * 原位姿误差使用本次修正的逆变换作同帧对照，不混用上一帧机器人位姿。
 */
class COnlineLocalRegistrationVerification final {
public:
    enum class Decision { Waiting, Verified, Rollback };

    /** @brief 长度单位 mm；样本须来自写回完成后的不同帧。 */
    struct Options {
        int requiredGoodFrames = 3;
        int maximumBadFrames = 3;
        double maximumErrorMm = 5.5;
        double maximumWorseningMm = 0.1;
    };

    /** @brief 冻结本次虚拟到真实修正及验收上限；不负责启动计时。 */
    COnlineLocalRegistrationVerification(const localization::RigidTransform &correction,
        const Options &options)
        : m_correction(correction), m_options(options)
    {
    }

    /** @brief 接收一份新帧观测；重复/旧帧无效果，失效观测计为未能验证。 */
    Decision observe(qint64 frameId, bool valid,
        const localization::Point3D_mm &realPoint,
        const localization::Point3D_mm &virtualPoint)
    {
        // 重复、旧帧及终态调用都不产生新的可信历史样本。
        m_lastObservationGood = false;
        if (frameId <= m_lastFrameId) { return m_decision; }
        m_lastFrameId = frameId;
        if (m_decision != Decision::Waiting) { return m_decision; }
        const auto &m = m_correction.rowMajor;
        const double x = virtualPoint.x_mm - m[3];
        const double y = virtualPoint.y_mm - m[7];
        const double z = virtualPoint.z_mm - m[11];
        const localization::Point3D_mm previousVirtual{
            m[0] * x + m[4] * y + m[8] * z,
            m[1] * x + m[5] * y + m[9] * z,
            m[2] * x + m[6] * y + m[10] * z};
        m_currentErrorMm = distance(realPoint, virtualPoint);
        m_previousErrorMm = distance(realPoint, previousVirtual);
        const bool good = valid && std::isfinite(m_currentErrorMm)
            && std::isfinite(m_previousErrorMm)
            && m_currentErrorMm <= m_options.maximumErrorMm
            && m_currentErrorMm <= m_previousErrorMm + m_options.maximumWorseningMm;
        m_lastObservationGood = good;
        if (good) {
            ++m_goodFrames;
            m_badFrames = 0;
            if (m_goodFrames >= m_options.requiredGoodFrames) {
                m_decision = Decision::Verified;
            }
        } else {
            m_goodFrames = 0;
            ++m_badFrames;
            if (m_badFrames >= m_options.maximumBadFrames) {
                m_decision = Decision::Rollback;
            }
        }
        return m_decision;
    }

    /** @brief 记录已消费但曝光状态过旧的帧；打断连续证据而不判为恶化。 */
    void skipObservation(qint64 frameId)
    {
        m_lastObservationGood = false;
        if (m_decision != Decision::Waiting || frameId <= m_lastFrameId) { return; }
        m_lastFrameId = frameId;
        m_goodFrames = 0;
    }

    Decision decision() const { return m_decision; }
    double currentErrorMm() const { return m_currentErrorMm; }
    double previousErrorMm() const { return m_previousErrorMm; }
    /// @brief 最近一份不同新帧是否通过冻结的验收门槛，供可信历史采样。
    bool lastObservationGood() const { return m_lastObservationGood; }

private:
    static double distance(const localization::Point3D_mm &a,
        const localization::Point3D_mm &b)
    {
        return std::hypot(a.x_mm - b.x_mm, a.y_mm - b.y_mm, a.z_mm - b.z_mm);
    }

    localization::RigidTransform m_correction;
    Options m_options;
    qint64 m_lastFrameId = -1;
    int m_goodFrames = 0;
    int m_badFrames = 0;
    Decision m_decision = Decision::Waiting;
    double m_currentErrorMm = 0.0;
    double m_previousErrorMm = 0.0;
    bool m_lastObservationGood = false;
};

} // namespace i2r_rlc::ui
#endif // I2R_RLC_UI_ONLINE_LOCAL_REGISTRATION_VERIFICATION_H
