/** @file test_online_local_registration_history_collection.cpp
 * @brief 通过明确的测试友元验证协调器连续历史采集，不改变 MSVC 私有成员链接名称。
 */
#include <iostream>
#include <QDateTime>
#include "RuntimeCoordinator.h"

namespace {
bool expect(bool condition, const char *message)
{
    if (!condition) { std::cerr << "FAIL: " << message << '\n'; }
    return condition;
}
}

namespace i2r_rlc::ui {
namespace ui = i2r_rlc::ui;
namespace twin = i2r_rlc::digital_twin;
/** @brief 只在测试目标定义，构造无设备的协调器状态并调用原生产采集入口。 */
struct CRuntimeCoordinatorHistoryTestAccess {
    /** @brief 历史采样须在ICP繁忙/冷却及无求解worker时继续，但不得跨写回阶段或身份。 */
    static bool run()
    {
        ui::CRuntimeCoordinator runtime;
        runtime.m_running = true;
        runtime.m_localRegistrationClock.start();
        runtime.m_localRegistrationOptions.enabled = true;
        // 本专项只检验既有焊点历史采集；母材通道在独立surface专项中覆盖。
        runtime.m_localRegistrationOptions.surfaceAssistedUpdatesEnabled = false;
        runtime.m_localRegistrationOptions.stabilityProtectionEnabled = true;
        runtime.m_localRegistrationOptions.sampleIntervalMs = 0;
        runtime.m_trackingPurpose = ui::RuntimeTrackingPurpose::OnlineCorrection;
        runtime.m_correctionSession.active = true;
        runtime.m_correctionSession.onlineControlWindowActive = true;
        runtime.m_correctionSession.sessionId = 44;
        runtime.m_localRegistrationGeneration = 2;
        runtime.m_globalLocateRobotSimScene.workpieceParentId = 1063;
        runtime.m_robotSimConnected = true;
        runtime.m_latestRobotState.sStart = 1;
        runtime.m_latestRobotState.sHold = 0;
        runtime.m_latestRobotState.sMode = 2;
        runtime.m_latestRobotStateReceivedMs = QDateTime::currentMSecsSinceEpoch();
        runtime.m_localRegistrationBusy = true;
        runtime.m_lastLocalRegistrationAttemptMs = runtime.m_localRegistrationClock.elapsed();
        ui::CRuntimeCoordinator::FrozenVisionFrameContext context;
        context.correctionSessionId = 44;
        context.onlineControlWindowActive = true;
        context.modelGeneration = 2;
        context.workpiecePoseRevision = runtime.m_localWorkpiecePoseRevision;
        context.robotStateReceivedMs = runtime.m_latestRobotStateReceivedMs;
        context.frame.frameId = 1;
        twin::DigitalTwinVisionLaserObservation observation;
        observation.valid = true;
        observation.frameId = 1;
        observation.referenceObjectId = 1063;
        observation.hasWeldPoint_mm = true;
        observation.weldPoint_mm = {0.0, 0.0, 0.0};
        context.synchronizedObservation = observation;
        ui::RuntimeTrackingResult tracking;
        tracking.valid = true;
        tracking.triggerAccepted = true;
        tracking.frameId = 1;
        tracking.independentWeldPointInParent_mm = {1.0, 0.0, 0.0, true};
        runtime.submitOnlineLocalRegistration(context, tracking);
        bool ok = expect(runtime.m_localRegistrationStability.anchors().size() == 1,
            "valid history must collect despite in-flight solve, cooldown and absent solve worker");
        if (runtime.m_localRegistrationStability.anchors().empty()) {
            runtime.m_running = false;
            runtime.m_localRegistrationBusy = false;
            return false;
        }
        context.frame.frameId = 2;
        context.synchronizedObservation->frameId = 2;
        tracking.frameId = 2;
        runtime.submitOnlineLocalRegistration(context, tracking);
        ok &= expect(runtime.m_localRegistrationStability.anchors().front().sampleCount == 2,
            "a new same-region sample must extend monitoring independently of successful writes");
        runtime.m_localRegistrationWritebackBusy = true;
        context.frame.frameId = 3;
        context.synchronizedObservation->frameId = 3;
        tracking.frameId = 3;
        runtime.submitOnlineLocalRegistration(context, tracking);
        ok &= expect(runtime.m_localRegistrationStability.anchors().front().sampleCount == 2,
            "new-pose observations must not be collected during unresolved writeback");
        runtime.m_localRegistrationWritebackBusy = false;
        context.modelGeneration = 1;
        runtime.submitOnlineLocalRegistration(context, tracking);
        ok &= expect(runtime.m_localRegistrationStability.anchors().front().sampleCount == 2,
            "old generation must never extend current monitoring evidence");
        context.modelGeneration = 2;
        runtime.m_localVerification.emplace();
        runtime.submitOnlineLocalRegistration(context, tracking);
        ok &= expect(runtime.m_localRegistrationStability.anchors().front().sampleCount == 2,
            "unresolved post-write verification must exclude new-pose history samples");
        runtime.m_localVerification.reset();
        context.onlineControlWindowActive = false;
        runtime.submitOnlineLocalRegistration(context, tracking);
        ok &= expect(runtime.m_localRegistrationStability.anchors().front().sampleCount == 2,
            "startup frame must not enter history when current online window opens later");
        runtime.m_running = false;
        runtime.m_localRegistrationBusy = false;
        return ok;
    }

};
}

bool testHistoryCollectionIndependentOfSolve()
{
    return i2r_rlc::ui::CRuntimeCoordinatorHistoryTestAccess::run();
}
