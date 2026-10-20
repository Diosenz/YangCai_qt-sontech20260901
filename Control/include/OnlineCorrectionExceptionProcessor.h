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
 * @file        OnlineCorrectionExceptionProcessor.h
 * @brief       在线检测段异常处理服务
 * @details     承接 OnlineExceptionHandling_beta 的活动 ICP 匹配语义。
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
#ifndef I2R_RLC_CONTROL_ONLINE_CORRECTION_EXCEPTION_PROCESSOR_H
#define I2R_RLC_CONTROL_ONLINE_CORRECTION_EXCEPTION_PROCESSOR_H

#include <vector>

#include "OnlineCorrectionTrajectoryPlanner.h"

namespace i2r_rlc::control {

struct OnlineCorrectionExceptionResult {
    bool valid = false;
    bool segmentChanged = false;
    /// @brief 本次调用释放的批次诊断，不影响求解与缓存生命周期。
    std::vector<OnlineCorrectionIcpBatchDiagnostic> batches;
    bool icpAttempted = false;
    bool icpApplied = false;
    std::size_t bufferedSampleCount = 0;
    double rmsErrorMm = 0.0;
    std::vector<OnlineCorrectionDetectionSample> samples;
    std::string message;
};

/**
 * @brief 串行累计在线检测并用轨迹 ICP 生成尚未释放批次的预测点。
 * @details 由控制器持有并在控制线程调用；历史只拟合，失败保留原始点，不拥有设备资源。
 */
class COnlineCorrectionExceptionProcessor final {
public:
    /**
     * @brief 处理新检测段，并在仿真焊缝变化时用 ICP 生成连续预测段。
     */
    OnlineCorrectionExceptionResult process(
        const std::vector<OnlineCorrectionDetectionSample> &newSamples,
        const OnlineCorrectionPlannedTrajectory &trajectory);

    /**
     * @brief flush 时输出段尾，条件不足则保留原始点，并清空窗口。
     * @details 历史仅参与拟合，每个检测批次只输出一次；控制线程串行调用。
     */
    OnlineCorrectionExceptionResult process(
        const std::vector<OnlineCorrectionDetectionSample> &newSamples,
        const OnlineCorrectionPlannedTrajectory &trajectory,
        bool flush);

    /**
     * @brief 兼容旧测试入口，仅提供当前参考段时按单段轨迹处理。
     */
    OnlineCorrectionExceptionResult process(
        const std::vector<OnlineCorrectionDetectionSample> &newSamples,
        const std::vector<OnlineCorrectionPoint> &referenceSegment);

    /// @brief 清空历史、pending、边界及前向相位，供新会话或新程序段重新累计。
    void reset();

private:
    std::vector<OnlineCorrectionDetectionSample> m_normalSamples;
    std::vector<OnlineCorrectionDetectionSample> m_pendingSamples;
    int m_lastSimulatedSeamIndex = -1;
    std::size_t m_curveFlag = 0;
    std::optional<OnlineCorrectionPoint> m_lastChangePoint;
    /// @brief 当前参考内已释放的连续相位；只前进一次，不自动绕回闭环起点。
    std::optional<double> m_lastReleasedPhase;
    std::optional<OnlineCorrectionPoint> m_lastReleasedRawPoint;

    /** @brief 拟合当前窗口，只输出 pending，成功后保留窗口后 80%。 */
    OnlineCorrectionExceptionResult processPending(
        const OnlineCorrectionPlannedTrajectory &trajectory, bool flush);
};

} // namespace i2r_rlc::control

#endif // I2R_RLC_CONTROL_ONLINE_CORRECTION_EXCEPTION_PROCESSOR_H
