/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      LocalizationLocalCorrectionPipeline.h
 * @brief     局部定位流程封装。
 * @details   本文件定义局部窗口点云修正流程的模块内编排入口。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 11 日
 */

#ifndef I2R_RLC_LOCALIZATION_LOCALIZATION_LOCAL_CORRECTION_PIPELINE_H
#define I2R_RLC_LOCALIZATION_LOCALIZATION_LOCAL_CORRECTION_PIPELINE_H

#include "ILocalizationService.h"
#include "LocalizationTypes.h"

namespace i2r_rlc::localization {

/**
 * @brief 局部定位模块内流程封装。
 *
 * 该类负责校验局部窗口点云、调用 Local ICP、计算虚拟点云修正结果并输出残差诊断。
 * UI 状态、线程同步、虚拟参考刷新和工件位姿实际写回由上层模块处理。
 */
class LocalizationLocalCorrectionPipeline final {
public:
    /**
     * @brief 绑定定位求解服务。
     */
    explicit LocalizationLocalCorrectionPipeline(ILocalizationService& localizationService);

    /**
     * @brief 执行一次局部定位离线流程。
     */
    LocalCorrectionPipelineResult run(const LocalCorrectionPipelineRequest& request);

private:
    ILocalizationService& m_localizationService;
};

} // namespace i2r_rlc::localization

#endif // I2R_RLC_LOCALIZATION_LOCALIZATION_LOCAL_CORRECTION_PIPELINE_H
