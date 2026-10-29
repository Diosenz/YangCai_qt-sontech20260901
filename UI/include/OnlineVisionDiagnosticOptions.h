/**
 * @file OnlineVisionDiagnosticOptions.h
 * @brief 在线二维检测诊断的会话级配置；不改变算法和控制参数。
 */
#ifndef I2R_RLC_UI_ONLINE_VISION_DIAGNOSTIC_OPTIONS_H
#define I2R_RLC_UI_ONLINE_VISION_DIAGNOSTIC_OPTIONS_H

#include <array>
#include <cmath>

#include <QtGlobal>

namespace i2r_rlc::ui {

/**
 * @brief 新会话固定的诊断策略；图像总量不随采样阶段增加。
 * @details 默认 30 对、32 MiB，在启动/在线三个采样进度层/异常间按 3/6/6/6/9
 * 分配。进度层只是处理序号退避，不表示物理焊缝段，也不保证任意长会话的尾帧。
 */
struct COnlineVisionDiagnosticOptions {
    bool enabled = true;
    bool imagesEnabled = true;
    int imageSampleEveryN = 15;
    int maxImagePairs = 30;
    qint64 maxImageBytes = 32LL * 1024 * 1024;
    /// @brief 临时首丢窗口：独立原图预算，关闭诊断或图像时也关闭。
    bool lossWindowEnabled = true;
    /// @brief 显式诊断模式：从会话首帧连续归档，队列满/预算不足均记录缺口。
    bool replayCaptureEnabled = false;
    int replayMaxFrames = 2048;
    qint64 replayMaxBytes = 1024LL * 1024 * 1024;
};

/**
 * @brief Vision 线程独占的纯值采样状态；不影响检测、融合或业务状态。
 * @details select 每次实际检测调用一次；正式阶段仅使用关联上下文冻结的在线窗口。
 * 返回桶编号 0=启动、1/2/3=在线采样进度层、4=异常，-1 表示本次不复制图像。
 * 三个在线层采用逐步增长间隔，异常首发立即，后续至少间隔 60 秒。
 */
class COnlineVisionImageSampling final {
public:
    /// @brief 按同一会话总量同比分桶；单对配置允许首个阶段使用唯一配额。
    static int pairBudget(const COnlineVisionDiagnosticOptions &options, int bucket)
    {
        if (bucket < 0 || bucket >= 5 || options.maxImagePairs <= 0) { return 0; }
        if (options.maxImagePairs == 1) { return 1; }
        const std::array<int, 6> boundary{0, 3, 9, 15, 21, 30};
        return static_cast<int>(static_cast<qint64>(options.maxImagePairs) * boundary[bucket + 1] / 30
            - static_cast<qint64>(options.maxImagePairs) * boundary[bucket] / 30);
    }

    /// @brief 压缩后字节预算与对数使用同份额；总量仍由后台写入器再次限制。
    static qint64 byteBudget(const COnlineVisionDiagnosticOptions &options, int bucket)
    {
        const int pairs = pairBudget(options, bucket);
        if (pairs <= 0 || options.maxImageBytes <= 0) { return 0; }
        return options.maxImageBytes / options.maxImagePairs * pairs
            + (options.maxImageBytes % options.maxImagePairs) * pairs / options.maxImagePairs;
    }

    /// @brief 仅使用原始 KCF 与仿真点的 10 px 偏差或已有跳变拒绝标志。
    static bool isAnomaly(double rawX, double rawY, double simulatedX, double simulatedY,
        bool rawValid, bool jumpRejected)
    {
        return jumpRejected || (rawValid && std::isfinite(rawX) && std::isfinite(rawY)
            && std::isfinite(simulatedX) && std::isfinite(simulatedY)
            && std::hypot(rawX - simulatedX, rawY - simulatedY) >= 10.0);
    }

    /// @brief 仅接受会话内本机单调 elapsed 毫秒；相机/系统时钟不参与异常限频。
    int select(const COnlineVisionDiagnosticOptions &options, bool onlineWindow,
        bool anomaly, qint64 monotonicElapsedMs)
    {
        if (!options.enabled || !options.imagesEnabled || options.maxImagePairs <= 0
            || options.maxImageBytes <= 0 || options.imageSampleEveryN <= 0
            || m_total >= static_cast<quint64>(options.maxImagePairs)) { return -1; }
        const quint64 frame = onlineWindow ? m_onlineFrames++ : m_startupFrames++;
        if (onlineWindow && anomaly && monotonicElapsedMs >= 0
            && m_counts[4] < pairBudget(options, 4)
            && (m_lastAnomalyMs < 0 || (monotonicElapsedMs >= m_lastAnomalyMs
                && monotonicElapsedMs - m_lastAnomalyMs >= 60000))) {
            m_lastAnomalyMs = monotonicElapsedMs;
            return take(4);
        }
        if (!onlineWindow) {
            return frame % static_cast<quint64>(options.imageSampleEveryN) == 0
                    && m_counts[0] < pairBudget(options, 0) ? take(0) : -1;
        }
        if (frame < m_nextOnlineFrame) { return -1; }
        int bucket = 1;
        while (bucket <= 3 && m_counts[bucket] >= pairBudget(options, bucket)) { ++bucket; }
        if (bucket > 3) { return -1; }
        ++m_onlineRoutineCount;
        const quint64 level = 1 + m_onlineRoutineCount / 2;
        m_nextOnlineFrame = frame + static_cast<quint64>(options.imageSampleEveryN) * level * level;
        return take(bucket);
    }

private:
    int take(int bucket)
    {
        ++m_counts[bucket];
        ++m_total;
        return bucket;
    }

    std::array<int, 5> m_counts{};
    quint64 m_total = 0;
    quint64 m_startupFrames = 0;
    quint64 m_onlineFrames = 0;
    quint64 m_onlineRoutineCount = 0;
    quint64 m_nextOnlineFrame = 0;
    qint64 m_lastAnomalyMs = -1;
};

} // namespace i2r_rlc::ui

#endif // I2R_RLC_UI_ONLINE_VISION_DIAGNOSTIC_OPTIONS_H
