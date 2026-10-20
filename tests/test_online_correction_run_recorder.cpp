/**
 * @file test_online_correction_run_recorder.cpp
 * @brief 验证批次矩阵、目标点及跟随几何的实际文件记录和会话隔离。
 */
#include <iostream>

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QUuid>

#include "OnlineCorrectionRunRecorder.h"

namespace {

QList<QByteArray> readRows(const QString &directory, const QString &name)
{
    QFile file(QDir(directory).filePath(name));
    if (!file.open(QIODevice::ReadOnly)) {
        return {};
    }
    auto rows = file.readAll().split('\n');
    if (!rows.isEmpty() && rows.last().isEmpty()) {
        rows.removeLast();
    }
    return rows;
}

bool expect(bool condition, const char *message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

} // namespace

int main(int argc, char **argv)
{
    QCoreApplication application(argc, argv);
    using namespace i2r_rlc;
    ui::COnlineCorrectionRunRecorder recorder;
    ui::RuntimeOnlineCorrectionConfiguration configuration;
    configuration.visionDiagnostics.imageSampleEveryN = 1;
    configuration.visionDiagnostics.maxImagePairs = 1;
    const QString root = QDir::tempPath() + QStringLiteral("/i2r_icp_records_")
        + QUuid::createUuid().toString(QUuid::WithoutBraces);
    QString error;
    if (!expect(recorder.beginSession(root, configuration, &error), "begin session")) {
        return 1;
    }
    control::OnlineCorrectionUpdate update;
    update.onlineIcpUpdated = true;
    update.phase = control::OnlineCorrectionPhase::OnlineRunning;
    control::OnlineCorrectionIcpBatchDiagnostic applied;
    applied.attempted = true;
    applied.applied = true;
    applied.detectionToModel = std::array<double, 16>{
        1, 0, 0, 2, 0, 1, 0, 3, 0, 0, 1, -4, 0, 0, 0, 1};
    applied.rmsErrorMm = 0.125;
    applied.fitPoints = {{1, 2, 3, true}};
    applied.modelPoints = {{3, 5, -1, true}};
    applied.pendingSamples = {{{1, 2, 3, true}, 2.5F, 7}};
    applied.releasedSamples = {{{1, 2, 3.25, true}, 2.5F, 7}};
    applied.message = "applied\twith\ncontext";
    auto fallback = applied;
    fallback.applied = false;
    fallback.attempted = false;
    fallback.flush = true;
    fallback.detectionToModel.reset();
    fallback.rmsErrorMm.reset();
    fallback.releasedSamples = fallback.pendingSamples;
    fallback.message = "raw tail";
    // 同一事件包含两批，不能只留下最后一批。
    update.onlineIcpBatches = {applied, fallback};
    update.followDiagnostic.valid = true;
    update.followDiagnostic.current = {0, 0, 0, true};
    update.followDiagnostic.first = {1, 2, 3.25, true};
    update.followDiagnostic.directionPoint = {2, 3, 4, true};
    update.followDiagnostic.foot = {0.5, 1, 2, true};
    update.followDiagnostic.futureFoot = {0.6, 1.1, 2.1, true};
    update.taughtPointIndex = 9;
    update.realtimeCommand.valid = true;
    update.realtimeCommand.dzUm = -3;
    vision::PositionerCalibration calibration;
    bool ok = expect(recorder.recordUpdate(update, calibration), "write two batches");
    update.onlineIcpUpdated = false;
    update.realtimeCommand.valid = false;
    ok &= expect(recorder.recordUpdate(update, calibration), "ordinary status update");
    const auto batches = readRows(recorder.sessionDirectory(), "online_icp_batches.txt");
    const auto points = readRows(recorder.sessionDirectory(), "online_icp_points.txt");
    const auto targets = readRows(recorder.sessionDirectory(), "online_follow_targets.txt");
    const auto legacy = readRows(recorder.sessionDirectory(), "realtime_correction.txt");
    ok &= expect(batches.size() == 3 && points.size() == 9 && targets.size() == 2,
                 "both batches and one follow target must persist without status duplicates");
    for (const auto &rows : {batches, points, targets, legacy}) {
        if (rows.isEmpty()) {
            ok = false;
            continue;
        }
        const auto columns = rows.front().split('\t').size();
        for (const auto &row : rows) {
            ok &= expect(row.split('\t').size() == columns, "TSV column alignment");
        }
    }
    if (batches.size() == 3) {
        const auto row = batches[1].split('\t');
        const auto raw = batches[2].split('\t');
        ok &= expect(row[2] == "1" && row[14] == "1" && row[26] == "-4.000000000000"
                         && row.last() == "applied with context", "matrix direction and sanitization");
        ok &= expect(raw[2] == "2" && raw[7] == "0" && raw[13].isEmpty()
                         && raw[14] == "0" && raw[15].isEmpty(), "fallback has blank matrix and RMS");
    }
    if (targets.size() == 2) {
        const auto row = targets[1].split('\t');
        ok &= expect(row[1] == "1" && row[4] == "9" && row[12] == "3.250000"
                         && row[24] == "-3", "actual target geometry and command share an update");
    }
    ui::RuntimeTrackingResult tracking;
    tracking.frameId = 42;
    tracking.correlation.sessionId = 7;
    tracking.correlation.frameId = 42;
    tracking.correlation.purpose = ui::CorrectionPurpose::OnlineCorrection;
    tracking.triggerAccepted = true;
    tracking.valid = true;
    tracking.sample.point = {1, 2, 3, true};
    tracking.diagnostic.emplace();
    auto &diagnostic = *tracking.diagnostic;
    diagnostic.cameraTimestampMs = 1000;
    diagnostic.robotStateReceivedMs = 995;
    diagnostic.processingStartedMs = 1010;
    diagnostic.processingFinishedMs = 1020;
    diagnostic.robotState.curx_r = 123456;
    diagnostic.robotState.curRx_r = -25000;
    diagnostic.algorithm.attempted = true;
    diagnostic.algorithm.templateReinitialized = true;
    diagnostic.algorithm.gaborSucceeded = true;
    diagnostic.algorithm.simulatedPoint = {100, 200};
    diagnostic.algorithm.responsePoint = {101, 210};
    diagnostic.algorithm.beforeFusionPoint = {101, 210};
    diagnostic.algorithm.finalPoint = {100.5, 205};
    diagnostic.algorithm.tracker.hasResponse = true;
    diagnostic.algorithm.tracker.completed = true;
    diagnostic.algorithm.tracker.fusionApplied = true;
    diagnostic.algorithm.tracker.responsePeak = 0.75;
    diagnostic.algorithm.tracker.gainX = 0.6;
    diagnostic.algorithm.tracker.gainY = 0.5;
    diagnostic.sourceImage = QImage(16, 12, QImage::Format_Grayscale8);
    diagnostic.enhancedImage = QImage(16, 12, QImage::Format_Grayscale8);
    diagnostic.sourceImage.fill(17);
    diagnostic.enhancedImage.fill(200);
    tracking.message = QStringLiteral("diagnostic\twith\ncontext");
    ok &= expect(recorder.recordTrackingResult(tracking, control::OnlineCorrectionPhase::OnlineRunning),
                 "record complete diagnostic frame");
    const auto diagnosticSnapshot = tracking.diagnostic;
    tracking.frameId = 43;
    tracking.diagnostic.reset();
    tracking.valid = false;
    tracking.sample.point.valid = false;
    tracking.triggerAccepted = false;
    ok &= expect(recorder.recordTrackingResult(tracking, control::OnlineCorrectionPhase::OnlineRunning),
                 "record rejected frame without inventing observations");
    tracking.frameId = 44;
    tracking.diagnostic = diagnosticSnapshot;
    ok &= expect(recorder.recordTrackingResult(tracking, control::OnlineCorrectionPhase::OnlineRunning),
                 "image pair cap must preserve numeric records");
    const auto diagnosticRows = readRows(recorder.sessionDirectory(), "vision_diagnostics.tsv");
    ok &= expect(diagnosticRows.size() == 4, "all three result rows must persist");
    if (diagnosticRows.size() == 4) {
        const auto header = diagnosticRows[0].split('\t');
        const auto first = diagnosticRows[1].split('\t');
        const auto rejected = diagnosticRows[2].split('\t');
        const auto capped = diagnosticRows[3].split('\t');
        const auto value = [&header](const QList<QByteArray> &row, const char *name) {
            const int index = header.indexOf(name);
            return index >= 0 && index < row.size() ? row[index] : QByteArray("MISSING");
        };
        ok &= expect(first.size() == header.size() && rejected.size() == header.size()
                     && capped.size() == header.size(), "diagnostic TSV column alignment");
        ok &= expect(value(first, "session_id") == "7" && value(first, "frame_id") == "42"
                     && value(first, "camera_timestamp_ms") == "1000"
                     && value(first, "robot_state_received_ms") == "995"
                     && value(first, "response_y_px") == "210.000000"
                     && value(first, "final_y_px") == "205.000000"
                     && value(first, "kcf_weight_y") == "0.500000000000"
                     && value(first, "robot_x_mm") == "123.456000"
                     && value(first, "robot_rx_deg") == "-2.500000"
                     && value(first, "message") == "diagnostic with context",
                     "same-frame stages, timestamp and robot units must survive serialization");
        ok &= expect(value(rejected, "response_peak").isEmpty()
                     && value(rejected, "robot_x_mm").isEmpty()
                     && value(rejected, "result_valid") == "0",
                     "missing stages must be blank rather than valid zeroes");
        ok &= expect(value(capped, "image_request_status") == "pair_limit",
                     "image cap must be explicit");
    }
    const QString firstSession = recorder.sessionDirectory();
    ok &= expect(recorder.finishSession("completed", "test"), "finish session");
    const auto manifest = readRows(firstSession, "vision_images/image_manifest.tsv");
    ok &= expect(manifest.size() == 2 && manifest[1].split('\t')[1] == "saved",
                 "finish must drain the background image queue");
    const QImage source(QDir(firstSession).filePath("vision_images/sample_1_session_7_frame_42_source.png"));
    const QImage enhanced(QDir(firstSession).filePath("vision_images/sample_1_session_7_frame_42_enhanced.png"));
    ok &= expect(!source.isNull() && !enhanced.isNull() && source.pixelColor(0, 0).red() == 17
                     && enhanced.pixelColor(0, 0).red() == 200,
                 "source and enhanced pixels must round-trip losslessly");
    configuration.visionDiagnostics.enabled = false;
    ok &= expect(recorder.beginSession(root, configuration, &error), "second session");
    update.onlineIcpUpdated = true;
    update.onlineIcpBatches = {fallback};
    update.followDiagnostic.valid = false;
    ok &= expect(recorder.recordUpdate(update, calibration), "second session release");
    ok &= expect(!QFile::exists(QDir(recorder.sessionDirectory()).filePath("vision_diagnostics.tsv"))
                     && !QDir(QDir(recorder.sessionDirectory()).filePath("vision_images")).exists(),
                 "disabled diagnostics must create neither diagnostic file nor images");
    ok &= expect(recorder.recordTrackingResult(tracking, control::OnlineCorrectionPhase::OnlineRunning),
                 "legacy recording must remain available when diagnostics are disabled");
    const auto fresh = readRows(recorder.sessionDirectory(), "online_icp_batches.txt");
    ok &= expect(fresh.size() == 2 && fresh[1].split('\t')[2] == "1",
                 "batch numbering must restart for the new session");
    ok &= expect(recorder.finishSession("completed", "test"), "finish second session");
    configuration.visionDiagnostics.enabled = true;
    configuration.visionDiagnostics.maxImageBytes = 1;
    ok &= expect(recorder.beginSession(root, configuration, &error), "budget-limited session");
    tracking.diagnostic = diagnosticSnapshot;
    ok &= expect(recorder.recordTrackingResult(tracking, control::OnlineCorrectionPhase::OnlineRunning),
                 "budget must not prevent numeric diagnostics");
    const QString budgetSession = recorder.sessionDirectory();
    ok &= expect(recorder.finishSession("completed", "budget"), "finish budget session");
    const auto budgetManifest = readRows(budgetSession, "vision_images/image_manifest.tsv");
    ok &= expect(budgetManifest.size() == 2
                     && budgetManifest[1].split('\t')[1] == "budget_or_disk_limit"
                     && QDir(QDir(budgetSession).filePath("vision_images"))
                         .entryList({"*.png"}, QDir::Files).isEmpty(),
                 "compressed pair exceeding byte budget must leave no PNG files");
    std::cout << (ok ? "PASS" : "FAIL") << ": online ICP result recording; files="
              << root.toStdString() << '\n';
    return ok ? 0 : 1;
}
