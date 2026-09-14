/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      NullIbvsService.cpp
 * @brief     IBVS 空服务实现。
 * @details   本文件返回结构化不可用状态，用于 IBVS 在线服务未启用时的运行时组装。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 8 日
 */

#include "ibvs/NullIbvsService.h"

namespace i2r_rlc::ibvs {

IbvsStartResult NullIbvsService::start()
{
    return {
        IbvsServiceState::Unavailable,
        u8"IbvsService 不可用：未启用 IBVS 在线服务。"
    };
}

IbvsStartResult NullIbvsService::configure(const IbvsServiceConfig& config)
{
    (void)config;
    return start();
}

IbvsStartResult NullIbvsService::startDryRun()
{
    return start();
}

IbvsStartResult NullIbvsService::stop()
{
    return start();
}

IbvsOnlineStepResult NullIbvsService::processFrame(
    const IbvsOnlineFrameInput& input)
{
    IbvsOnlineStepResult result;
    result.frameIndex = input.frameIndex;
    result.state = IbvsServiceState::Unavailable;
    result.error = {
        IbvsStatus::RUNTIME_ERROR,
        "null IBVS service cannot process frames"
    };
    result.message = u8"IbvsService 不可用：无法处理在线验证帧。";
    return result;
}

IbvsServiceState NullIbvsService::state() const
{
    return IbvsServiceState::Unavailable;
}

IbvsOnlineStepResult NullIbvsService::lastDiagnostics() const
{
    IbvsOnlineStepResult result;
    result.state = IbvsServiceState::Unavailable;
    result.error = {
        IbvsStatus::RUNTIME_ERROR,
        "null IBVS service has no diagnostics"
    };
    result.message = u8"IbvsService 不可用：没有在线诊断。";
    return result;
}

} // namespace i2r_rlc::ibvs
