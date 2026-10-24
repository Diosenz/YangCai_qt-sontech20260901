/**
 * @file test_online_control_target_presentation.cpp
 * @brief 控制业务目标经真实 Controller/View 显示，不受当前测量缺失或旧会话覆盖。
 */
#include <cmath>
#include <cstdio>
#include <QApplication>
#include <QLabel>
#include "MainWidget.h"
#include "UiController.h"

namespace i2r_rlc::ui {
/** @brief 仅测试持有私有访问权；不初始化设备或替代实际显示函数。 */
struct COnlineControlTargetPresentationTestAccess {
    static bool run()
    {
        bool ok = true;
        const auto check = [&ok](bool condition, const char *message) {
            if (!condition) { std::fprintf(stderr, "FAIL: %s\n", message); ok = false; }
        };
        MainWidget view;
        UiController controller(view);
        controller.m_positionerCalibration.valid = true;
        controller.m_positionerCalibration.toolTcpInLastLinkMm = Eigen::Vector3d(0, 0, 231);
        controller.m_onlineCorrectionSyncState.sessionId = 7;
        controller.m_onlineCorrectionSyncState.purpose = CorrectionPurpose::OnlineCorrection;
        controller.m_onlineCorrectionSyncState.acceptingFrames = true;
        vision::WeldProjectionContext projection;
        projection.valid = true;
        projection.calibration.valid = true;
        projection.calibration.focalLengthM = 0.01;
        projection.calibration.sxM = projection.calibration.syM = 0.00001;
        projection.calibration.intrinsics.cxPixel = 320;
        projection.calibration.intrinsics.cyPixel = 240;
        projection.calibration.intrinsics.fxPixel = projection.calibration.intrinsics.fyPixel = 1000;
        const auto calibration = controller.m_positionerCalibration;
        const auto frame = [&projection, &calibration](qint64 id) {
            RuntimeTrackingResult result;
            result.frameId = id;
            result.triggerAccepted = true;
            result.correlation.purpose = CorrectionPurpose::OnlineCorrection;
            result.correlation.sessionId = 7;
            result.correlation.frameId = id;
            result.correlation.onlineControlWindowActive = true;
            result.displayProjectionContext = projection;
            result.displayPositionerCalibration = calibration;
            result.frameContext.emplace();
            result.frameContext->sourceFrame.frameId = id;
            result.frameContext->sourceFrame.image = QImage(640, 480, QImage::Format_Grayscale8);
            result.frameContext->sourceFrame.image.fill(20);
            result.frameContext->observation.valid = true;
            result.frameContext->observation.frameId = id;
            return result;
        };
        auto image = frame(10);
        image.valid = true;
        image.imageXPixel = 100;
        image.imageYPixel = 120;
        controller.m_onlineCorrectionSyncState.lastConsumedFrameId = 10;
        controller.showRuntimeTrackingResult(image);
        control::OnlineCorrectionUpdate update;
        update.succeeded = true;
        update.phase = control::OnlineCorrectionPhase::OnlineRunning;
        update.state.phase = update.phase;
        update.state.controlTarget.valid = true;
        update.state.controlTarget.sessionId = 7;
        update.state.controlTarget.generation = 1;
        update.state.controlTarget.source = control::OnlineCorrectionTargetSource::Measured;
        update.state.controlTarget.targetPointInLastLink_mm = {10, 0, 331, true};
        controller.showOnlineCorrectionUpdate(update);
        check(view.m_detectedWeldPointOverlayVisible
                && std::abs(view.m_detectedWeldPointOverlay.x() - 420) < 0.01,
            "actual control target replaces the different latest camera measurement using one TCP-offset conversion");
        update.state.controlTarget.source = control::OnlineCorrectionTargetSource::PredictedReference;
        update.state.controlTarget.generation = 2;
        update.state.controlTarget.targetPointInLastLink_mm.xMm = 20;
        controller.showOnlineCorrectionUpdate(update);
        auto missing = frame(11);
        controller.m_onlineCorrectionSyncState.lastConsumedFrameId = 11;
        controller.showRuntimeTrackingResult(missing);
        check(view.m_detectedWeldPointOverlayVisible
                && std::abs(view.m_detectedWeldPointOverlay.x() - 520) < 0.01
                && view.m_trackingFrameCaption.contains(QStringLiteral("预测")),
            "invalid image measurement still displays the current predicted control target on the current image");
        check(!missing.valid && !missing.sample.point.valid && !missing.independentWeldPointInParent_mm.valid,
            "display projection cannot create a measurement or independent ICP point");
        auto moved = frame(12);
        moved.displayProjectionContext->toolInRobotBase.translationMm.x() = 10;
        controller.m_onlineCorrectionSyncState.lastConsumedFrameId = 12;
        controller.showRuntimeTrackingResult(moved);
        check(view.m_detectedWeldPointOverlayVisible
                && std::abs(view.m_detectedWeldPointOverlay.x() - 420) < 0.01,
            "display uses this exposure pose rather than reusing the previous target pixel");
        auto older = update;
        older.state.controlTarget.generation = 1;
        older.state.controlTarget.targetPointInLastLink_mm.xMm = -20;
        controller.showOnlineCorrectionUpdate(older);
        check(std::abs(view.m_detectedWeldPointOverlay.x() - 420) < 0.01,
            "older target snapshots cannot overwrite a newer target");
        auto wrongSession = update;
        wrongSession.state.controlTarget.sessionId = 6;
        wrongSession.state.controlTarget.generation = 999;
        controller.showOnlineCorrectionUpdate(wrongSession);
        check(std::abs(view.m_detectedWeldPointOverlay.x() - 420) < 0.01,
            "late old-session target cannot overwrite the active session target");
        auto noProjection = frame(13);
        noProjection.displayProjectionContext.reset();
        controller.m_onlineCorrectionSyncState.lastConsumedFrameId = 13;
        controller.showRuntimeTrackingResult(noProjection);
        auto newer = update;
        newer.state.controlTarget.generation = 3;
        newer.state.controlTarget.targetPointInLastLink_mm.xMm = 30;
        controller.showOnlineCorrectionUpdate(newer);
        check(!view.m_detectedWeldPointOverlayVisible
                && view.m_onlineCoordinateStatusLabel->text().contains(QStringLiteral("30.000")),
            "without projection the current business target remains numerically visible in its declared coordinate frame");
        auto unavailable = newer;
        unavailable.state.controlTarget.valid = false;
        unavailable.state.controlTarget.source = control::OnlineCorrectionTargetSource::Unavailable;
        controller.showOnlineCorrectionUpdate(unavailable);
        check(!controller.m_onlineControlTarget.valid && !view.m_detectedWeldPointOverlayVisible
                && view.m_onlineCoordinateStatusLabel->text().contains(QStringLiteral("不可用")),
            "same-session invalidation cannot keep presenting an old valid target as current");
        controller.showOnlineCorrectionUpdate(update);
        check(!controller.m_onlineControlTarget.valid,
            "a delayed older target cannot resurrect a target invalidated by a newer snapshot");
        auto current = frame(14);
        controller.m_onlineCorrectionSyncState.lastConsumedFrameId = 14;
        controller.showRuntimeTrackingResult(current);
        newer.state.controlTarget.generation = 4;
        controller.showOnlineCorrectionUpdate(newer);
        check(view.m_detectedWeldPointOverlayVisible, "lifecycle case starts with a visible current target");
        auto finishing = newer;
        finishing.phase = finishing.state.phase = control::OnlineCorrectionPhase::SegmentFinishing;
        finishing.state.controlTarget.valid = false;
        controller.showOnlineCorrectionUpdate(finishing);
        check(!view.m_detectedWeldPointOverlayVisible && !controller.m_onlineKcfLatestValid,
            "leaving the online phase immediately revokes the visible target and coordinate validity");
        auto reverseScan = frame(15);
        reverseScan.correlation.onlineControlWindowActive = false;
        reverseScan.valid = true;
        reverseScan.imageXPixel = 170;
        reverseScan.imageYPixel = 190;
        reverseScan.detectedRobotBasePoint = {1, 2, 3, true};
        controller.m_onlineCorrectionSyncState.lastConsumedFrameId = 15;
        controller.showRuntimeTrackingResult(reverseScan);
        check(view.m_detectedWeldPointOverlayVisible
                && std::abs(view.m_detectedWeldPointOverlay.x() - 170) < 0.01
                && controller.m_onlineKcfLatestValid,
            "next reverse scan keeps showing its own accepted image point after the previous online segment exits");
        newer.state.segmentIndex = 1;
        newer.state.controlTarget.generation = 5;
        controller.showOnlineCorrectionUpdate(newer);
        CorrectionSyncUpdate cleared;
        cleared.purpose = CorrectionPurpose::OnlineCorrection;
        cleared.sessionId = 7;
        cleared.state = CorrectionSyncState::Cleared;
        controller.handleCorrectionSyncUpdate(cleared);
        check(!controller.m_onlineControlTarget.valid && !view.m_detectedWeldPointOverlayVisible,
            "clearing a session revokes both its image target and numerical target identity");
        newer.state.controlTarget.generation = 6;
        controller.showOnlineCorrectionUpdate(newer);
        check(!controller.m_onlineControlTarget.valid && !view.m_detectedWeldPointOverlayVisible,
            "late target publication cannot revive a cleared session");
        CorrectionSyncUpdate replacement;
        replacement.purpose = CorrectionPurpose::OnlineCorrection;
        replacement.sessionId = 8;
        replacement.state = CorrectionSyncState::Idle;
        controller.handleCorrectionSyncUpdate(replacement);
        auto next = frame(13);
        next.correlation.sessionId = 8;
        controller.m_onlineCorrectionSyncState.lastConsumedFrameId = 13;
        controller.showRuntimeTrackingResult(next);
        check(!view.m_detectedWeldPointOverlayVisible,
            "new session with no established target cannot inherit the previous session target");
        COnlineCorrectionWorkerAdapter worker;
        quint64 publishedSession = 0;
        QObject::connect(&worker, &COnlineCorrectionWorkerAdapter::onlineCorrectionUpdated,
            &controller, [&publishedSession](const control::OnlineCorrectionUpdate &value) {
                publishedSession = value.state.controlTarget.sessionId;
            });
        worker.setControlSessionId(71);
        worker.reset();
        check(publishedSession == 71, "runtime worker stamps the actual target publication with session identity");
        worker.setControlSessionId(72);
        worker.reset();
        check(publishedSession == 72, "runtime worker does not retain an old session on target reset publication");
        controller.shutdown();
        return ok;
    }
};
}

int main(int argc, char **argv)
{
    qputenv("QT_QPA_PLATFORM", "offscreen");
    qputenv("I2R_RLC_DISABLE_AUTO_DEVICE_START", "1");
    QApplication app(argc, argv);
    const bool ok = i2r_rlc::ui::COnlineControlTargetPresentationTestAccess::run();
    std::printf("%s: online control target presentation\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
