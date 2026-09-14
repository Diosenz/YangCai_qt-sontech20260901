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
    std::vector<OnlineCorrectionDetectionSample> samples;
    std::string message;
};

/**
 * @brief 使用检测段与示教段 ICP 结果修复分段连续性。
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
     * @brief 兼容旧测试入口，仅提供当前参考段时按单段轨迹处理。
     */
    OnlineCorrectionExceptionResult process(
        const std::vector<OnlineCorrectionDetectionSample> &newSamples,
        const std::vector<OnlineCorrectionPoint> &referenceSegment);

    /// @brief 清空累计正常段和仿真焊缝索引。
    void reset();

private:
    std::vector<OnlineCorrectionDetectionSample> m_normalSamples;
    int m_lastSimulatedSeamIndex = -1;
    int m_curveFlag = 0;
    int m_matchTime = 0;
    int m_segDetFstPtToAllTpId = 0;
    int m_segDetLstPtToAllTpId = 0;
    int m_preDetLstPtToAllTpId = 0;
    OnlineCorrectionPoint m_lastMatchPoint;
};

} // namespace i2r_rlc::control

#endif // I2R_RLC_CONTROL_ONLINE_CORRECTION_EXCEPTION_PROCESSOR_H
