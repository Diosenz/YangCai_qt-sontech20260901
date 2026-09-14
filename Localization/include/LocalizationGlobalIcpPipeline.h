/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      LocalizationGlobalIcpPipeline.h
 * @brief     全局 ICP 定位流程封装。
 * @details   本文件定义整件点云或分段点云的全局 ICP 模块内编排入口。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 11 日
 */

#ifndef I2R_RLC_LOCALIZATION_LOCALIZATION_GLOBAL_ICP_PIPELINE_H
#define I2R_RLC_LOCALIZATION_LOCALIZATION_GLOBAL_ICP_PIPELINE_H

#include "ILocalizationService.h"
#include "LocalizationTypes.h"

namespace i2r_rlc::localization {

/**
 * @brief 全局 ICP 定位模块内流程封装。
 *
 * 该类负责校验全局点云输入、执行单段或分段受限 ICP、组合虚拟到真实的修正矩阵，
 * 并输出修正后的虚拟参考点云和分段诊断。
 */
class LocalizationGlobalIcpPipeline final {
public:
    /**
     * @brief 绑定定位求解服务。
     */
    explicit LocalizationGlobalIcpPipeline(ILocalizationService& localizationService);

    /**
     * @brief 执行一次全局 ICP 定位流程。
     */
    GlobalIcpPipelineResult run(const GlobalIcpPipelineRequest& request);

private:
    ILocalizationService& m_localizationService;
};

} // namespace i2r_rlc::localization

#endif // I2R_RLC_LOCALIZATION_LOCALIZATION_GLOBAL_ICP_PIPELINE_H
