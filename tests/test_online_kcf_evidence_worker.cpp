/**
 * @file test_online_kcf_evidence_worker.cpp
 * @brief 实际视觉 worker 的生产证据策略测试，不连接机器人或相机。
 */
#include <cmath>
#include <cstdio>

#include <QCoreApplication>
#include <QDateTime>

#include "RuntimeWorkerAdapters.h"

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    using namespace i2r_rlc;
    ui::CVisionTrackingWorkerAdapter worker;
    ui::RuntimeTrackingOptions options;
    options.detType = 2;
    options.virtualStatus = control::TrackingVirtualStatus::OnlineRecord;
    vision::WeldProjectionContext projection;
    projection.valid = projection.calibration.valid = true;
    projection.calibration.focalLengthM = 0.01;
    projection.calibration.sxM = projection.calibration.syM = 0.00001;
    projection.calibration.intrinsics.cxPixel = 320;
    projection.calibration.intrinsics.cyPixel = 240;
    projection.calibration.laserPlaneC = -1;
    worker.configure(options, projection);
    worker.startTracking();
    ui::RuntimeTrackingResult result;
    QObject::connect(&worker, &ui::CVisionTrackingWorkerAdapter::trackingResultReady,
        &app, [&result](const ui::RuntimeTrackingResult &value) { result = value; });
    robot_comm::RobotState robot;
    robot.sMode = 2;
    robot.sStart = robot.traflag = 1;
    bool online = false;
    const auto submit = [&](qint64 id, bool blank) {
        camera::CameraFrame frame;
        frame.frameId = id;
        frame.timestamp = QDateTime::fromMSecsSinceEpoch(1700000000000LL + id * 100);
        frame.image = QImage(640, 480, QImage::Format_Grayscale8);
        for (int y = 0; y < 480; ++y) {
            for (int x = 0; x < 640; ++x) {
                frame.image.scanLine(y)[x] = !blank
                    && std::abs(y - (260 - 0.45 * std::abs(x - 360))) < 1.5 ? 225 : 14;
            }
        }
        ui::RuntimeVisionCorrelation correlation;
        correlation.sessionId = 42;
        correlation.purpose = ui::CorrectionPurpose::OnlineCorrection;
        correlation.frameId = id;
        correlation.onlineControlWindowActive = online;
        // 实际 transport 失败 DTO；不得回退到任意旧仿真参考。
        const auto observation = digital_twin::makeInvalidVisionLaserObservation(id, "fixture missing CAD");
        worker.submitFrameWithRobotStateObservationAndCorrelation(frame, robot, -1, observation, correlation);
        return result;
    };
    bool ok = true;
    const auto check = [&ok](bool value, const char *message) {
        if (!value) { std::fprintf(stderr, "FAIL: %s\n", message); ok = false; }
    };
    const auto first = submit(1, false);
    check(first.valid && first.imageObservationAccepted
        && !first.independentWeldPointInParent_mm.valid,
        "first current-image observation remains distinct from independently confirmed geometry");
    const auto measured = submit(2, false);
    check(measured.valid && measured.evidencePolicyEnabled && measured.controlEligible
        && measured.kcfPixelAvailable && measured.independentWeldPointInParent_mm.valid,
        "missing CAD does not block independently confirmed geometry with diagnostics disabled");
    check(std::abs(measured.imageXPixel - 360) < 2, "measurement remains at real image seam");
    const auto blank = submit(3, true);
    check(!blank.valid && !blank.kcfPixelAvailable && !blank.controlEligible,
        "unconfigured continuation cannot manufacture a control sample");
    const auto firstRecovery = submit(4, false);
    check(firstRecovery.valid && firstRecovery.imageObservationAccepted
        && !firstRecovery.independentWeldPointInParent_mm.valid,
        "first recovery image is visible while independent geometry awaits confirmation");
    check(submit(5, false).controlEligible, "second fresh matching exposure recovers");
    const auto duplicate = submit(5, false);
    check(!duplicate.valid && !duplicate.independentWeldPointInParent_mm.valid,
        "duplicate cannot produce independent geometry or control sample");
    online = true;
    submit(6, false);
    check(submit(7, false).controlEligible, "online mode starts with confirmed image identity");
    for (qint64 id = 8; id < 28; ++id) {
        const auto held = submit(id, true);
        check(!held.valid && !held.kcfPixelAvailable && !held.controlEligible,
            "fully blank input cannot manufacture an online control sample");
        check(!held.independentWeldPointInParent_mm.valid
            && !held.localizationEvidence.currentImageSupported
            && !held.localizationEvidence.imageConstrainedEstimate,
            "absent image support cannot masquerade as a shoulder estimate or independent geometry");
    }
    const auto recovering = submit(28, false);
    check(recovering.valid && recovering.kcfPixelAvailable,
        "first recovery exposure remains visible while geometry reconfirms");
    check(std::hypot(recovering.imageXPixel - 360, recovering.imageYPixel - 260) < 2,
        "visible recovery returns to the original seam without a displaced continuation");
    check(submit(29, false).independentWeldPointInParent_mm.valid,
        "confirmed recovery returns to independent measurement");
    worker.stopTracking();
    std::puts(ok ? "PASS: production Vision worker evidence contract" : "FAIL: production Vision worker evidence contract");
    return ok ? 0 : 1;
}
