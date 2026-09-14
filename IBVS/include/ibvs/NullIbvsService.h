/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      NullIbvsService.h
 * @brief     IBVS 空服务实现。
 * @details   本文件定义服务不可用或未启用时使用的空实现，避免 UI 直接依赖算法细节。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 8 日
 */

#ifndef I2R_RLC_IBVS_NULL_IBVS_SERVICE_H
#define I2R_RLC_IBVS_NULL_IBVS_SERVICE_H

#include "ibvs/IIbvsService.h"

namespace i2r_rlc::ibvs {

/**
 * @brief IBVS 空服务实现。
 */
class NullIbvsService final : public IIbvsService {
public:
    IbvsStartResult start() override;
    IbvsStartResult configure(const IbvsServiceConfig& config) override;
    IbvsStartResult startDryRun() override;
    IbvsStartResult stop() override;
    IbvsOnlineStepResult processFrame(const IbvsOnlineFrameInput& input) override;
    IbvsServiceState state() const override;
    IbvsOnlineStepResult lastDiagnostics() const override;
};

} // namespace i2r_rlc::ibvs

#endif // I2R_RLC_IBVS_NULL_IBVS_SERVICE_H
