/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsOnlineService.cpp
 * @brief     IBVS 在线验证服务实现。
 * @details   本文件将配置读取、传感融合、统一速度求解和验证命令草案串联。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 15 日
 */

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "ibvs/IbvsOnlineService.h"
#include "ibvs/IbvsSensorFusion.h"
#include "ibvs/IbvsUnifiedVelocitySolver.h"

namespace i2r_rlc::ibvs {

namespace {

constexpr double MIN_JACOBIAN_NORM = 1e-9;

} // namespace

IbvsOnlineService::IbvsOnlineService()
{
    m_lastDiagnostics.state = m_state;
    m_lastDiagnostics.error = { IbvsStatus::OK, std::string() };
    m_lastDiagnostics.message = u8"IBVS 服务空闲，等待加载配置。";
}

IbvsStartResult IbvsOnlineService::start()
{
    if (m_state == IbvsServiceState::Configured
        || m_state == IbvsServiceState::DryRunReady
        || m_state == IbvsServiceState::RunningDryRun)
    {
        m_state = IbvsServiceState::DryRunReady;
        return makeStartResult(u8"IBVS dry-run 服务已就绪。");
    }

    if (m_state == IbvsServiceState::Idle)
    {
        return {
            m_state,
            u8"IBVS 服务未加载配置。"
        };
    }

    return makeStartResult(u8"IBVS 服务当前不可启动。");
}

IbvsStartResult IbvsOnlineService::configure(const IbvsServiceConfig& config)
{
    const IbvsRuntimeTextConfigResult runtimeResult =
        IbvsTextConfigReader::readRuntimeDefaults(config.runtimeDefaultsPath);
    if (!runtimeResult.error.isOk())
    {
        setError("failed to load IBVS runtime config: "
            + runtimeResult.error.message);
        return makeStartResult(m_lastDiagnostics.message);
    }

    const IbvsWorkpieceProfilesResult profileResult =
        IbvsTextConfigReader::readWorkpieceProfiles(config.workpieceProfilesPath);
    if (!profileResult.error.isOk())
    {
        setError("failed to load IBVS workpiece profiles: "
            + profileResult.error.message);
        return makeStartResult(m_lastDiagnostics.message);
    }

    m_runtimeConfig = runtimeResult.config;
    m_hasPositionConstraint = false;
    m_positionConstraint = IbvsPositionConstraint();

    const IbvsWorkpieceProfile* profile =
        IbvsTextConfigReader::findProfileById(
            profileResult.profiles,
            config.workpieceId);
    if (profile == NULL)
    {
        profile = IbvsTextConfigReader::findProfileForLegacyType(
            profileResult.profiles,
            config.workpieceType);
    }
    if (profile != NULL)
    {
        if (profile->hasFeatureGeometry)
        {
            m_runtimeConfig.featureGeometry = profile->featureGeometry;
            m_runtimeConfig.fusion.geometryParameters = profile->featureGeometry;
        }
        if (profile->hasLvsFusionThresholds)
        {
            m_runtimeConfig.fusion.minLvsConfidence = profile->minLvsConfidence;
            m_runtimeConfig.fusion.minLvsCenterlineQuality =
                profile->minLvsCenterlineQuality;
        }
        if (profile->hasPositionConstraint)
        {
            m_positionConstraint = profile->positionConstraint;
            m_hasPositionConstraint = true;
        }
    }

    m_state = IbvsServiceState::Configured;
    m_lastDiagnostics = IbvsOnlineStepResult();
    m_lastDiagnostics.state = m_state;
    m_lastDiagnostics.message = u8"IBVS 配置已加载，等待启动 dry-run。";
    m_lastDiagnostics.error = { IbvsStatus::OK, std::string() };
    return makeStartResult(m_lastDiagnostics.message);
}

IbvsStartResult IbvsOnlineService::startDryRun()
{
    if (m_state != IbvsServiceState::Configured
        && m_state != IbvsServiceState::DryRunReady
        && m_state != IbvsServiceState::RunningDryRun)
    {
        setError("IBVS dry-run requires configured service");
        return makeStartResult(m_lastDiagnostics.message);
    }

    m_state = IbvsServiceState::RunningDryRun;
    m_lastDiagnostics.state = m_state;
    m_lastDiagnostics.error = { IbvsStatus::OK, std::string() };
    m_lastDiagnostics.message = u8"IBVS dry-run 已启动；不会发送真实机器人命令。";
    return makeStartResult(m_lastDiagnostics.message);
}

IbvsStartResult IbvsOnlineService::stop()
{
    if (m_state == IbvsServiceState::RunningDryRun)
    {
        m_state = IbvsServiceState::DryRunReady;
    }
    else if (m_state == IbvsServiceState::Error)
    {
        m_state = IbvsServiceState::Idle;
    }

    m_lastDiagnostics.state = m_state;
    m_lastDiagnostics.message = u8"IBVS dry-run 已停止。";
    return makeStartResult(m_lastDiagnostics.message);
}

IbvsOnlineStepResult IbvsOnlineService::processFrame(
    const IbvsOnlineFrameInput& input)
{
    IbvsOnlineStepResult result;
    result.frameIndex = input.frameIndex;
    result.state = m_state;
    result.currentFeature = input.currentFeature;

    if (m_state != IbvsServiceState::RunningDryRun)
    {
        result.error = {
            IbvsStatus::RUNTIME_ERROR,
            "IBVS dry-run is not running"
        };
        result.message = u8"IBVS dry-run 未启动，拒绝处理帧。";
        m_lastDiagnostics = result;
        return result;
    }

    if (!isFiniteFeature(input.currentFeature)
        || !isFiniteFeature(input.fallbackDesiredFeature)
        || !input.cameraJacobian.allFinite()
        || input.cameraJacobian.norm() <= MIN_JACOBIAN_NORM)
    {
        result.error = {
            IbvsStatus::INVALID_ARGUMENT,
            "IBVS online frame input is invalid"
        };
        result.message = u8"IBVS dry-run 输入无效。";
        m_lastDiagnostics = result;
        return result;
    }

    IbvsFeatureVector desiredFeature = input.fallbackDesiredFeature;
    bool usedLvsPrior = false;
    if (input.torchObservation.has_value())
    {
        const IbvsFusionResult fusion = IbvsSensorFusion::fuseDesiredFeature(
            *input.torchObservation,
            input.lvsObservation,
            input.fallbackDesiredFeature,
            m_runtimeConfig.fusion);
        if (!fusion.error.isOk())
        {
            result.error = fusion.error;
            result.message = u8"IBVS 传感融合失败。";
            m_lastDiagnostics = result;
            return result;
        }

        desiredFeature = fusion.desiredFeature;
        usedLvsPrior = fusion.usedLvs;
    }

    IbvsLvsConstraint lvsConstraint = m_runtimeConfig.lvsConstraint;
    if (input.lvsConstraint.has_value())
    {
        lvsConstraint = *input.lvsConstraint;
    }

    IbvsPositionConstraint positionConstraint;
    if (input.positionConstraint.has_value())
    {
        positionConstraint = *input.positionConstraint;
    }

    const IbvsUnifiedVelocityResult velocity =
        IbvsUnifiedVelocitySolver::computeUnifiedVelocity(
            input.cameraJacobian,
            input.currentFeature,
            desiredFeature,
            lvsConstraint,
            positionConstraint,
            m_runtimeConfig.unifiedVelocity);
    if (!velocity.error.isOk())
    {
        result.error = velocity.error;
        result.message = u8"IBVS 统一速度求解失败。";
        m_lastDiagnostics = result;
        return result;
    }

    result.error = { IbvsStatus::OK, std::string() };
    result.state = m_state;
    result.desiredFeature = desiredFeature;
    result.featureErrorPx = velocity.featureErrorPx;
    result.velocityMps = velocity.velocityMps;
    result.velocityLimited = velocity.velocityLimited;
    result.usedLvsPrior = usedLvsPrior;
    result.usedLvsConstraint = velocity.usedLvs;
    result.usedPositionConstraint = velocity.usedPositionConstraint;
    result.rowCount = velocity.rowCount;
    result.dampingMu = velocity.dampingMu;
    result.dryRunCommand = makeDryRunCommand(velocity.velocityMps);
    result.message = u8"IBVS dry-run 单帧处理完成；未发送真实机器人命令。";
    m_lastDiagnostics = result;
    return result;
}

IbvsServiceState IbvsOnlineService::state() const
{
    return m_state;
}

IbvsOnlineStepResult IbvsOnlineService::lastDiagnostics() const
{
    return m_lastDiagnostics;
}

std::string IbvsOnlineService::makeDryRunCommand(
    const IbvsVelocityVector& velocityMps)
{
    char buffer[160] = {};
    std::snprintf(
        buffer,
        sizeof(buffer),
        "DRYRUN IBVS_VELOCITY_MPS %.9f,%.9f,%.9f",
        velocityMps(0),
        velocityMps(1),
        velocityMps(2));
    return std::string(buffer);
}

bool IbvsOnlineService::isFiniteFeature(const IbvsFeatureVector& feature)
{
    for (int i = 0; i < feature.size(); ++i)
    {
        if (!std::isfinite(feature(i)))
        {
            return false;
        }
    }

    return true;
}

IbvsStartResult IbvsOnlineService::makeStartResult(
    const std::string& message) const
{
    return { m_state, message };
}

void IbvsOnlineService::setError(const std::string& message)
{
    m_state = IbvsServiceState::Error;
    m_lastDiagnostics = IbvsOnlineStepResult();
    m_lastDiagnostics.state = m_state;
    m_lastDiagnostics.error = { IbvsStatus::RUNTIME_ERROR, message };
    m_lastDiagnostics.message = message;
}

} // namespace i2r_rlc::ibvs
