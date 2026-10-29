/**
 * @file test_online_correction_run_recorder.cpp
 * @brief 验证批次矩阵、目标点及跟随几何的实际文件记录和会话隔离。
 */
#include <array>
#include <chrono>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <iostream>
#include <thread>

#include <QCoreApplication>
#include <QBuffer>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QImageWriter>
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

/** @brief 启动饱和后仍须保留正式阶段、延后采样和异常配额；总预算不变。 */
bool testStagedImageSamplingPolicy()
{
    using namespace i2r_rlc::ui;
    const COnlineVisionDiagnosticOptions options;
    COnlineVisionImageSampling sampling;
    std::array<int, 5> counts{};
    bool ok = expect(options.maxImagePairs == 30
        && options.maxImageBytes == 32LL * 1024 * 1024, "default image budgets must not increase");
    for (int frame = 0; frame < 450; ++frame) {
        const int bucket = sampling.select(options, false, false, frame * 100);
        if (bucket >= 0) { ++counts[static_cast<std::size_t>(bucket)]; }
    }
    ok &= expect(counts[0] == 3, "startup must stop after three pairs and retain online budget");
    bool lateImage = false;
    for (int frame = 0; frame < 12000; ++frame) {
        const int bucket = sampling.select(options, true, true, 45000 + frame * 100);
        if (bucket >= 0) {
            ++counts[static_cast<std::size_t>(bucket)];
            lateImage |= bucket == 3 && frame > 1000;
        }
    }
    ok &= expect(counts == std::array<int, 5>{3, 6, 6, 6, 9} && lateImage,
        "formal online sampling must retain all progress layers and bounded anomaly pairs");
    COnlineVisionImageSampling anomalies;
    anomalies.select(options, true, false, 0);
    ok &= expect(anomalies.select(options, true, true, 100) == 4
        && anomalies.select(options, true, true, 200) == -1
        && anomalies.select(options, true, true, 60100) == 4,
        "first off-cadence anomaly must be immediate and later anomalies limited to one per minute");
    ok &= expect(COnlineVisionImageSampling::isAnomaly(0, 0, 11, 0, true, false)
        && !COnlineVisionImageSampling::isAnomaly(0, 0, 2, 0, true, false)
        && COnlineVisionImageSampling::isAnomaly(0, 0, 0, 0, false, true),
        "anomalies must use raw KCF-to-simulation deviation or jump flags independently of fusion");
    COnlineVisionDiagnosticOptions single = options;
    single.maxImagePairs = 1;
    COnlineVisionImageSampling limited;
    ok &= expect(limited.select(single, false, false, 0) == 0
        && limited.select(single, true, true, 60000) == -1,
        "one-pair configuration must preserve first-frame sampling and the total cap");
    qint64 bytes = 0;
    int pairs = 0;
    for (int bucket = 0; bucket < 5; ++bucket) {
        bytes += COnlineVisionImageSampling::byteBudget(options, bucket);
        pairs += COnlineVisionImageSampling::pairBudget(options, bucket);
    }
    ok &= expect(bytes <= options.maxImageBytes && pairs == options.maxImagePairs,
        "bucket budgets must partition the existing byte and pair limits");
    return ok;
}

/** @brief recorder 不重新推测阶段，启动超额不能挤占正式窗口或延后采样桶。 */
bool testRecorderImageStageReservation(const QString &root)
{
    using namespace i2r_rlc;
    ui::COnlineCorrectionRunRecorder recorder;
    ui::RuntimeOnlineCorrectionConfiguration configuration;
    QString error;
    if (!expect(recorder.beginSession(root, configuration, &error), "begin staged recorder session")) {
        return false;
    }
    ui::RuntimeTrackingResult result;
    result.correlation.sessionId = 123;
    result.correlation.purpose = ui::CorrectionPurpose::OnlineCorrection;
    result.diagnostic.emplace();
    result.diagnostic->imageSampleBucket = 0;
    result.diagnostic->sourceImage = QImage(16, 12, QImage::Format_Grayscale8);
    result.diagnostic->sourceImage.fill(20);
    result.diagnostic->enhancedImage = result.diagnostic->sourceImage;
    bool ok = true;
    for (int index = 0; index < 4; ++index) {
        result.frameId = index;
        ok &= expect(recorder.recordTrackingResult(result,
            control::OnlineCorrectionPhase::OnlineRunning), "record startup quota request");
    }
    result.correlation.onlineControlWindowActive = true;
    for (int bucket : {1, 2, 3, 4}) {
        result.frameId = 10 + bucket;
        result.diagnostic->imageSampleBucket = bucket;
        ok &= expect(recorder.recordTrackingResult(result,
            control::OnlineCorrectionPhase::OnlineRunning), "record reserved online quota request");
    }
    result.diagnostic->imageSampleBucket = 0;
    ok &= expect(recorder.recordTrackingResult(result,
        control::OnlineCorrectionPhase::OnlineRunning), "record mismatched sampling context numerically");
    const QString directory = recorder.sessionDirectory();
    ok &= expect(recorder.finishSession("completed", "stage reservation"), "finish staged recorder session");
    const auto rows = readRows(directory, QStringLiteral("vision_diagnostics.tsv"));
    ok &= expect(rows.size() == 10, "all reserved and rejected requests must retain numeric rows");
    if (rows.size() == 10) {
        const auto header = rows[0].split('\t');
        const int statusColumn = header.indexOf("image_request_status");
        const int bucketColumn = header.indexOf("image_sample_bucket");
        if (statusColumn < 0 || bucketColumn < 0) {
            return expect(false, "sample bucket and request outcome must be auditable");
        }
        ok &= expect(rows[4].split('\t')[statusColumn] == "pair_limit",
            "fourth startup image must be rejected even while the session total has room");
        for (int row = 5; row < 9; ++row) {
            const auto fields = rows[row].split('\t');
            ok &= expect((fields[statusColumn] == "queued" || fields[statusColumn] == "queue_full")
                && fields[bucketColumn] == QByteArray::number(row - 4),
                "reserved online buckets must remain eligible after startup saturation");
        }
        ok &= expect(rows[9].split('\t')[statusColumn] == "invalid_sampling_context",
            "recorder must reject a startup bucket attached to the frozen formal-online window");
    }
    return ok;
}

/** @brief 启动桶字节耗尽不得挤占正式在线桶；PNG 编码仍由后台执行。 */
bool testImageWriterBucketReservation(const QString &root)
{
    using namespace i2r_rlc::ui;
    const QString directory = QDir(root).filePath(QStringLiteral("bucket_writer"));
    bool ok = expect(QDir().mkpath(directory), "create reserved-byte writer directory");
    QImage source(16, 12, QImage::Format_Grayscale8);
    source.fill(17);
    CTrackingDiagnosticImageWriter writer(directory, 4096);
    ok &= expect(writer.submit(QStringLiteral("startup"), source, source, 0, 1)
            == QStringLiteral("queued")
        && writer.submit(QStringLiteral("online"), source, source, 1, 2048)
            == QStringLiteral("queued"),
        "both bucket requests should enter the bounded writer");
    writer.stop();
    const auto rows = readRows(directory, QStringLiteral("image_manifest.tsv"));
    ok &= expect(rows.size() == 2 && rows[0].split('\t')[1] == "budget_or_disk_limit"
        && rows[1].split('\t')[1] == "saved",
        "startup byte rejection must leave the reserved online byte budget available");
    return ok;
}
/** @brief 已验收 KCF 的测量身份不依赖可选诊断，且不能变成独立焊点。 */
bool testImageMeasurementProvenance(const QString &root)
{
    using namespace i2r_rlc;
    ui::COnlineCorrectionRunRecorder recorder;
    ui::RuntimeOnlineCorrectionConfiguration configuration;
    configuration.visionDiagnostics.enabled = true;
    configuration.visionDiagnostics.imagesEnabled = false;
    QString error;
    if (!expect(recorder.beginSession(root, configuration, &error), "begin image measurement provenance session")) {
        return false;
    }
    ui::RuntimeTrackingResult result;
    result.correlation.purpose = ui::CorrectionPurpose::OnlineCorrection;
    result.valid = true;
    result.sample.point = {1.0, 2.0, 3.0, true};
    result.imageXPixel = 101.5;
    result.imageYPixel = 202.25;
    result.pointSource = vision::OnlineSeamPointSource::CorrectedKcf;
    result.imageMeasurementOutputEnabled = true;
    result.imageObservationAccepted = true;
    bool ok = recorder.recordTrackingResult(result, control::OnlineCorrectionPhase::InitialDetectionRunning);
    result.sample.estimated = true;
    result.correlation.onlineControlWindowActive = true;
    ok &= recorder.recordTrackingResult(result, control::OnlineCorrectionPhase::OnlineRunning);
    result.diagnostic.emplace();
    result.diagnostic->algorithm.attempted = true;
    result.diagnostic->algorithm.imageMeasurementOutputEnabled = true;
    result.diagnostic->algorithm.imageObservationAccepted = true;
    result.diagnostic->algorithm.simulatedPointAvailable = false;
    result.diagnostic->algorithm.simulatedPoint = {999.0, 888.0};
    result.diagnostic->algorithm.finalPoint = {101.5, 202.25};
    ok &= recorder.recordTrackingResult(result, control::OnlineCorrectionPhase::OnlineRunning);
    ok &= expect(recorder.flush(), "drain accepted text before offline readback");
    const auto rows = readRows(recorder.sessionDirectory(), "vision_diagnostics.tsv");
    ok &= expect(rows.size() == 4, "KCF stages and a missing CAD hint retain numeric provenance");
    if (rows.size() == 4) {
        const auto header = rows[0].split('\t');
        for (int index = 1; index < rows.size(); ++index) {
            const auto row = rows[index].split('\t');
            const auto value = [&header, &row](const char *name) {
                const int column = header.indexOf(name);
                return column >= 0 && column < row.size() ? row[column] : QByteArray("MISSING");
            };
            ok &= expect(row.size() == header.size()
                    && value("image_measurement_output_enabled") == "1"
                    && value("image_observation_accepted") == "1"
                    && value("estimated") == (index == 1 ? "0" : "1")
                    && value("point_source") == "2"
                    && value("final_x_px") == "101.500000"
                    && value("final_y_px") == "202.250000"
                    && value("sim_x_px").isEmpty() && value("sim_y_px").isEmpty()
                    && value("geometry_valid") == "0"
                    && value("geometry_x_px").isEmpty(),
                "KCF stage identity preserves initial admission and formal ICP exclusion without geometry");
        }
    }
    ok &= expect(recorder.finishSession("completed", "measurement provenance"),
        "finish image measurement provenance session");
    return ok;
}
/** @brief 首丢窗口保留连续帧、实际曝光投影和收到结果时的控制参考，且按阶段有界。 */
bool testFirstLossWindow(const QString &root)
{
    using namespace i2r_rlc;
    ui::COnlineCorrectionRunRecorder recorder;
    ui::RuntimeOnlineCorrectionConfiguration configuration;
    QString error;
    bool ok = expect(recorder.beginSession(root, configuration, &error), "begin loss window session");
    bool visibleEstimate = false;
    const auto submit = [&](int id, bool valid, control::OnlineCorrectionPhase phase) {
        ui::RuntimeTrackingResult result;
        result.frameId = result.correlation.frameId = id;
        result.correlation.sessionId = 19;
        result.correlation.purpose = ui::CorrectionPurpose::OnlineCorrection;
        result.triggerAccepted = true;
        result.valid = result.kcfPixelAvailable = valid || visibleEstimate;
        result.controlEligible = valid;
        result.evidencePolicyEnabled = true;
        result.diagnostic.emplace();
        result.diagnostic->cameraTimestampMs = 1000 + id;
        result.diagnostic->lossWindowSourceImage = QImage(32, 24, QImage::Format_Grayscale8);
        result.diagnostic->lossWindowSourceImage.fill(id);
        result.diagnostic->algorithm.evidence.debugDetail = "[DEBUG-kcf-20261026] identity_geometry=0";
        result.displayProjectionContext.emplace();
        result.displayProjectionContext->valid = true;
        result.displayProjectionContext->calibration.kappa = 2.5;
        control::OnlineCorrectionTargetSnapshot target;
        target.generation = 200 + id;
        target.source = control::OnlineCorrectionTargetSource::PredictedReference;
        return recorder.recordTrackingResult(result, phase, target);
    };
    for (int stage = 0; stage < 2; ++stage) {
        for (int index = 1; index <= 20; ++index) {
            ok &= expect(submit(stage * 20 + index, index <= 8 || index == 18,
                stage == 0 ? control::OnlineCorrectionPhase::InitialDetectionRunning
                           : control::OnlineCorrectionPhase::OnlineRunning), "record loss frame");
        }
    }
    const QString directory = recorder.sessionDirectory();
    ok &= expect(recorder.finishSession("completed", "loss capture"), "drain loss capture");
    const auto index = readRows(directory, "loss_window_index.tsv");
    const auto contexts = readRows(directory, "loss_window_context.jsonl");
    const auto manifest = readRows(directory, "loss_window_images/image_manifest.tsv");
    ok &= expect(index.size() == 31 && contexts.size() == 30 && manifest.size() == 31,
        "each stage captures exactly 6 before, trigger and 8 after, with no later retrigger");
    if (contexts.size() == 30) {
        const auto first = QJsonDocument::fromJson(contexts.front()).object();
        const auto last = QJsonDocument::fromJson(contexts.back()).object();
        ok &= expect(first["frame_id"].toInt() == 3 && last["frame_id"].toInt() == 37
            && first["control_reference"].toObject()["generation"].toInt() == 203
            && first["timestamp_ms"].toInt() == 1003
            && first["projection"].toObject()["calibration"].toObject()["kappa"].toDouble() == 2.5,
            "ring eviction preserves exposure context and the original per-frame reference");
        const QImage saved(QDir(directory).filePath(first["image"].toString()));
        ok &= expect(!saved.isNull() && saved.pixelColor(0, 0).red() == 3,
            "saved raw image belongs to the original pre-loss exposure");
    }
    for (const auto &row : manifest.mid(1)) {
        ok &= expect(row.split('\t').value(1) == "saved", "all bounded window images actually saved");
    }
    ok &= expect(recorder.beginSession(root, configuration, &error), "restart loss state");
    for (int id = 1; id <= 8; ++id) {
        ok &= submit(id, false, control::OnlineCorrectionPhase::InitialDetectionRunning);
    }
    ok &= submit(8, false, control::OnlineCorrectionPhase::InitialDetectionRunning);
    const QString neverAcquired = recorder.sessionDirectory();
    ok &= recorder.finishSession("completed", "never acquired");
    const auto neverRows = readRows(neverAcquired, "loss_window_index.tsv");
    ok &= expect(neverRows.size() == 8 && neverRows.back().split('\t').value(3) == "trigger",
        "eight consecutive rejected exposures capture startup failure without duplicating frame eight");
    ok &= recorder.beginSession(root, configuration, &error);
    for (int id = 1; id <= 33; ++id) {
        visibleEstimate = id == 9;
        ok &= submit(id, id != 9 && id != 25,
            control::OnlineCorrectionPhase::OnlineRunning);
    }
    visibleEstimate = false;
    const QString laterUnavailable = recorder.sessionDirectory();
    ok &= recorder.finishSession("completed", "later pixel loss after earlier ineligible estimate");
    const auto laterContexts = readRows(laterUnavailable, "loss_window_context.jsonl");
    bool savedLaterUnavailable = false;
    for (const auto &row : laterContexts) {
        const auto context = QJsonDocument::fromJson(row).object();
        savedLaterUnavailable |= context["frame_id"].toInt() == 25;
    }
    ok &= expect(laterContexts.size() == 30 && savedLaterUnavailable,
        "earlier visible ineligible estimate cannot consume the only later pixel-loss capture");
    ok &= recorder.beginSession(root, configuration, &error);
    for (int id = 1; id <= 25; ++id) {
        visibleEstimate = id == 9;
        ok &= submit(id, id != 9 && id != 17,
            control::OnlineCorrectionPhase::OnlineRunning);
    }
    visibleEstimate = false;
    const QString overlappingLoss = recorder.sessionDirectory();
    ok &= recorder.finishSession("completed", "pixel loss at the last after-frame");
    const auto overlappingContexts = readRows(overlappingLoss, "loss_window_context.jsonl");
    ok &= expect(overlappingContexts.size() == 23
        && QJsonDocument::fromJson(overlappingContexts.back()).object()["frame_id"].toInt() == 25,
        "pixel loss on the last post-frame retains eight later recovery exposures without duplicates");
    configuration.visionDiagnostics.lossWindowEnabled = false;
    ok &= recorder.beginSession(root, configuration, &error);
    ok &= expect(!QFile::exists(QDir(recorder.sessionDirectory()).filePath("loss_window_index.tsv")),
        "temporary window can be disabled independently");
    ok &= recorder.finishSession("completed", "disabled window");
    return ok;
}
/** @brief 正常有效曝光也必须完整归档，不能等到控制拒绝后才开始留图。 */
bool testContinuousReplayCapture(const QString &root)
{
    using namespace i2r_rlc;
    ui::RuntimeOnlineCorrectionConfiguration configuration;
    configuration.visionDiagnostics.replayCaptureEnabled = true;
    configuration.visionDiagnostics.replayMaxFrames = 3;
    configuration.visionDiagnostics.lossWindowEnabled = false;
    ui::COnlineCorrectionRunRecorder recorder;
    QString error;
    bool ok = expect(recorder.beginSession(root, configuration, &error), "begin continuous replay");
    for (int id = 1; id <= 4; ++id) {
        ui::RuntimeTrackingResult result;
        result.frameId = result.correlation.frameId = id;
        result.correlation.sessionId = 19;
        result.correlation.purpose = ui::CorrectionPurpose::OnlineCorrection;
        result.triggerAccepted = result.valid = result.controlEligible = true;
        result.diagnostic.emplace();
        result.diagnostic->cameraTimestampMs = 1000 + id;
        result.diagnostic->lossWindowSourceImage = QImage(32, 24, QImage::Format_Grayscale8);
        result.diagnostic->lossWindowSourceImage.fill(id);
        result.diagnostic->replaySourceImage = result.diagnostic->lossWindowSourceImage;
        result.diagnostic->algorithm.tracker.trainingFromSimulation = id == 2;
        result.diagnostic->algorithm.normalResponseSamples = 32;
        result.diagnostic->replayOptions.emplace();
        result.diagnostic->replayOptions->enablePaperTracking = true;
        result.diagnostic->replayOptions->paperUseMotherBranches = true;
        result.diagnostic->replayOptions->paperPsrSigma = 2.5;
        result.diagnostic->replayObservation.emplace();
        auto &reference = *result.diagnostic->replayObservation;
        reference.frameId = id;
        reference.valid = reference.hasWeldPointImage_px = true;
        reference.seamIndex = 7;
        reference.weldPointImage_px = {12.0, 13.0};
        reference.laserLineImagePoints_px = {{1.0, 2.0}, {3.0, 4.0}};
        result.displayProjectionContext.emplace();
        result.displayProjectionContext->valid = true;
        ok &= recorder.recordTrackingResult(result, control::OnlineCorrectionPhase::OnlineRunning);
        if (id == 2) {
            ok &= recorder.recordTrackingResult(result, control::OnlineCorrectionPhase::OnlineRunning);
        }
    }
    const QString directory = recorder.sessionDirectory();
    ok &= recorder.finishSession("completed", "continuous replay");
    const auto events = readRows(directory, "unified_session.jsonl");
    bool recordedSimulationTraining = false;
    for (const auto &line : events) {
        const auto event = QJsonDocument::fromJson(line).object();
        const auto paper = event["paper_tracking"].toObject();
        recordedSimulationTraining |= event["frame_id"].toString() == "2"
            && paper["training_from_simulation"].toBool()
            && paper["normal_samples"].toInt() == 32;
    }
    ok &= expect(recordedSimulationTraining, "PSR calibration and actual simulation training are recorded independently");
    const auto index = readRows(directory, "replay_index.tsv");
    const auto contexts = readRows(directory, "replay_context.jsonl");
    const auto manifest = readRows(directory, "replay_images/image_manifest.tsv");
    ok &= expect(index.size() == 5 && contexts.size() == 4 && manifest.size() == 4,
        "valid frames captured from the start, duplicate omitted and frame cap explicitly indexed");
    if (contexts.size() == 4 && index.size() == 5) {
        const auto first = QJsonDocument::fromJson(contexts.front()).object();
        ok &= expect(first["sim_line"].toArray().size() == 2
            && first["simulation_frame_id"].toInt() == 1
            && first["seam_index"].toInt() == 7 && first["detector_input_complete"].toBool()
            && first["paper_options"].toObject()["branch_constraints"].toBool()
            && first["paper_options"].toObject()["psr_sigma"].toDouble() == 2.5,
            "same exposure simulation shape and projection survive serialization");
        const QImage image(QDir(directory).filePath(first["image"].toString()));
        ok &= expect(!image.isNull() && image.pixelColor(0, 0).red() == 1,
            "first accepted exposure preserved without waiting for failure");
        ok &= expect(index.back().split('\t').value(2) == "frame_limit",
            "frame budget omission is visible instead of silently truncating the replay");
    }
    return ok;
}
/** @brief 在现场分辨率和曝光节奏下验收公开写入器；不把缺图归档算作完整采集。 */
bool testContinuousWriterThroughput(const QString &root, const QString &inputPath)
{
    using namespace i2r_rlc::ui;
    const QImage source = QImage(inputPath).convertToFormat(QImage::Format_Grayscale8);
    if (!expect(!source.isNull(), "load captured full-size grayscale source")) { return false; }
    const QString directory = QDir(root).filePath(QStringLiteral("continuous_writer"));
    QDir().mkpath(directory);
    constexpr int frameCount = 32;
    constexpr qint64 budget = 128LL * 1024 * 1024;
    CTrackingDiagnosticImageWriter writer(directory, budget, 8);
    const auto start = std::chrono::steady_clock::now();
    int queued = 0;
    for (int index = 0; index < frameCount; ++index) {
        std::this_thread::sleep_until(start + std::chrono::milliseconds(index * 100));
        queued += writer.submit(QString::number(index), source, QImage(), 0, budget)
            == QStringLiteral("queued") ? 1 : 0;
    }
    writer.stop();
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();
    int saved = 0;
    bool pixelsEqual = true;
    for (const auto &row : readRows(directory, QStringLiteral("image_manifest.tsv"))) {
        const auto fields = row.split('\t');
        if (fields.size() >= 2 && fields[1] == "saved") {
            ++saved;
            const QImage restored(QDir(directory).filePath(
                QString::fromUtf8(fields[0]) + QStringLiteral("_source.png")));
            pixelsEqual &= restored == source;
        }
    }
    std::cout << "Writer throughput report: " << directory.toStdString()
        << "; size=" << source.width() << 'x' << source.height()
        << "; queued=" << queued << "; saved=" << saved
        << "; elapsed_ms=" << elapsed << std::endl;
    return expect(queued == frameCount && saved == frameCount && pixelsEqual,
        "full-size 10 Hz replay input capture must save every submitted exposure losslessly");
}

/** @brief 比较 Qt PNG 编码代价，区分图像压缩和后台队列容量问题。 */
bool benchmarkPngEncoding(const QString &inputPath)
{
    const QImage source = QImage(inputPath).convertToFormat(QImage::Format_Grayscale8);
    if (!expect(!source.isNull(), "PNG benchmark requires a readable image")) { return false; }
    for (const int compression : {-1, 0, 20}) {
        const auto start = std::chrono::steady_clock::now();
        qint64 totalBytes = 0;
        for (int index = 0; index < 8; ++index) {
            QByteArray encoded;
            QBuffer buffer(&encoded);
            buffer.open(QIODevice::WriteOnly);
            QImageWriter writer(&buffer, "PNG");
            if (compression >= 0) { writer.setCompression(compression); }
            if (!expect(writer.write(source), "encode PNG benchmark input")) { return false; }
            totalBytes += encoded.size();
        }
        std::cout << "PNG benchmark compression=" << compression
            << "; average_ms=" << std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count() / 8.0
            << "; average_bytes=" << totalBytes / 8 << std::endl;
    }
    return true;
}
/** @brief 在途字节计入容量；满队列后仍排空已接纳前缀，不能无限等待慢盘。 */
bool testBoundedTextWriter(qsizetype maxBytes, std::size_t maxChunks)
{
    using i2r_rlc::ui::COnlineCorrectionTextWriter;
    std::mutex mutex;
    std::condition_variable condition;
    bool entered = false;
    bool released = false;
    QByteArray output;
    const auto caller = std::this_thread::get_id();
    bool background = true;
    COnlineCorrectionTextWriter writer([&](const QString &, const QByteArray &data, QString *) {
        std::unique_lock<std::mutex> lock(mutex);
        background &= std::this_thread::get_id() != caller;
        entered = true;
        condition.notify_all();
        condition.wait(lock, [&]() { return released; });
        output += data;
        return true;
    }, maxBytes, maxChunks);
    writer.start();
    bool ok = expect(writer.submit("fixture", "AAAA"), "first text chunk admitted");
    {
        std::unique_lock<std::mutex> lock(mutex);
        ok &= expect(condition.wait_for(lock, std::chrono::seconds(2), [&]() { return entered; }),
            "filesystem boundary is blocked before checking capacity");
    }
    ok &= expect(writer.submit("fixture", "BBBB") && !writer.submit("fixture", "C")
        && !writer.submit("fixture", "D"), "capacity includes blocked in-flight data and rejects immediately");
    {
        std::lock_guard<std::mutex> lock(mutex);
        released = true;
    }
    condition.notify_all();
    writer.stop();
    writer.stop();
    ok &= expect(output == "AAAABBBB" && background && writer.rejectedChunks() == 2
        && !writer.lastError().isEmpty() && !writer.flush(),
        "stop drains accepted chunks once, in order, and preserves incomplete status");
    return ok;
}

/** @brief 队列拒绝必须写入最终摘要，不能仅保留正常结束标记。 */
bool testTextOverflowSummary(const QString &root)
{
    using namespace i2r_rlc;
    ui::COnlineCorrectionRunRecorder recorder;
    ui::RuntimeOnlineCorrectionConfiguration configuration;
    configuration.visionDiagnostics.imagesEnabled = false;
    QString error;
    if (!expect(recorder.beginSession(root, configuration, &error) && recorder.flush(),
            "prepare overflow recorder")) { return false; }
    control::OnlineCorrectionUpdate update;
    update.message.assign(9 * 1024 * 1024, 'x');
    bool ok = expect(!recorder.recordUpdate(update, {}), "oversized text is rejected at the bounded queue");
    ok &= expect(!recorder.finishSession("completed", "overflow fixture"),
        "overflow is reported by session completion");
    const auto rows = readRows(recorder.sessionDirectory(), "session_summary.txt");
    ok &= expect(rows.contains("text_recording_complete\t0")
        && !rows.contains("text_recording_complete\t1") && !recorder.lastError().isEmpty(),
        "summary cannot claim text completeness after rejection");
    return ok;
}

/** @brief 文件系统报告刷新失败时，即使数据可读也不能宣称完整；重启会话可恢复。 */
bool testTextFlushFailure(const QString &root)
{
    using namespace i2r_rlc;
    std::atomic<bool> failFlush{false};
    ui::COnlineCorrectionRunRecorder recorder([&](const QString &path, const QByteArray &data, QString *error) {
        QFile file(path);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Append) || file.write(data) != data.size()) {
            *error = file.errorString();
            return false;
        }
        if (failFlush.load() && path.endsWith("actual_robot_path.txt")) {
            *error = QStringLiteral("injected flush failure after write");
            return false;
        }
        const bool flushed = file.flush();
        if (!flushed) { *error = file.errorString(); }
        return flushed;
    });
    ui::RuntimeOnlineCorrectionConfiguration configuration;
    configuration.visionDiagnostics.imagesEnabled = false;
    QString error;
    if (!expect(recorder.beginSession(root, configuration, &error) && recorder.flush(),
            "prepare delayed flush failure fixture")) { return false; }
    failFlush = true;
    recorder.recordRobotState({}, control::OnlineCorrectionPhase::OnlineRunning);
    bool ok = expect(!recorder.flush() && !recorder.finishSession("completed", "flush failure"),
        "flush failure remains a failed completion even after write accepted bytes");
    const auto failed = readRows(recorder.sessionDirectory(), "session_summary.txt");
    ok &= expect(!failed.contains("text_recording_complete\t1")
        && recorder.lastError().contains("injected flush failure"), "visible data cannot override a flush error");
    failFlush = false;
    ok &= expect(recorder.beginSession(root, configuration, &error)
        && recorder.recordRobotState({}, control::OnlineCorrectionPhase::OnlineRunning)
        && recorder.finishSession("completed", "fresh session"), "new session can recover from old recording error");
    ok &= expect(readRows(recorder.sessionDirectory(), "session_summary.txt").contains("text_recording_complete\t1"),
        "new session completeness is independent of prior failed writer");
    return ok;
}

/** @brief 文件系统抛错时，结束会话也必须返回失败而不是让异常逃出 worker。 */
bool testTextFailureFinalization(const QString &root)
{
    using namespace i2r_rlc;
    std::atomic<bool> fail{false};
    ui::COnlineCorrectionRunRecorder recorder([&](const QString &path, const QByteArray &data, QString *error) {
        if (fail.load()) { throw std::runtime_error("injected filesystem failure"); }
        QFile file(path);
        const bool ok = file.open(QIODevice::WriteOnly | QIODevice::Append)
            && file.write(data) == data.size();
        if (!ok) { *error = file.errorString(); }
        return ok;
    });
    ui::RuntimeOnlineCorrectionConfiguration configuration;
    configuration.visionDiagnostics.imagesEnabled = false;
    QString error;
    if (!expect(recorder.beginSession(root, configuration, &error) && recorder.flush(),
            "prepare failing filesystem")) { return false; }
    fail = true;
    recorder.recordRobotState({}, control::OnlineCorrectionPhase::OnlineRunning);
    bool ok = expect(!recorder.flush() && !recorder.lastError().isEmpty(),
        "background filesystem exception becomes a visible recording failure");
    try {
        ok &= expect(!recorder.finishSession("completed", "write failure"),
            "failed filesystem cannot produce a successful finalization");
    } catch (...) {
        ok &= expect(false, "filesystem exception must not escape session finalization");
    }
    fail = false;
    recorder.finishSession("error", "cleanup after injected failure");
    return ok;
}
} // namespace

int main(int argc, char **argv)
{
    QCoreApplication application(argc, argv);
    if (argc == 3 && QString::fromLocal8Bit(argv[1]) == QStringLiteral("--continuous-writer-contract")) {
        const QString root = QDir::current().filePath(QStringLiteral(".scratch/preweld-structural-continuity-20261028/writer_%1")
            .arg(QUuid::createUuid().toString(QUuid::WithoutBraces)));
        return testContinuousWriterThroughput(root, QString::fromLocal8Bit(argv[2])) ? 0 : 1;
    }
    if (argc == 3 && QString::fromLocal8Bit(argv[1]) == QStringLiteral("--png-encoding-benchmark")) {
        return benchmarkPngEncoding(QString::fromLocal8Bit(argv[2])) ? 0 : 1;
    }
    if (argc > 1 && QString::fromLocal8Bit(argv[1]) == QStringLiteral("--text-writer-contract")) {
        const QString root = QDir::temp().filePath(QStringLiteral("i2r_text_contract_%1")
            .arg(QUuid::createUuid().toString(QUuid::WithoutBraces)));
        bool ok = testBoundedTextWriter(8, 4);
        ok &= testBoundedTextWriter(16, 2);
        ok &= testTextFlushFailure(root);
        ok &= testTextOverflowSummary(root);
        ok &= testTextFailureFinalization(root);
        std::cout << (ok ? "PASS" : "FAIL") << ": bounded text recording contracts\n";
        return ok ? 0 : 1;
    }
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
    if (!testImageMeasurementProvenance(root)) { return 1; }
    if (!testFirstLossWindow(root)) { return 1; }
    if (!testContinuousReplayCapture(root)) { return 1; }
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
    fallback.pendingSamples[0].estimated = true;
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
    bool ok = testStagedImageSamplingPolicy();
    ok &= testImageWriterBucketReservation(root);
    ok &= testRecorderImageStageReservation(root);
    ok &= expect(recorder.recordUpdate(update, calibration), "write two batches");
    update.onlineIcpUpdated = false;
    update.realtimeCommand.valid = false;
    ok &= expect(recorder.recordUpdate(update, calibration), "ordinary status update");
    ok &= expect(recorder.flush(), "drain accepted text before offline readback");
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
    if (points.size() == 9) {
        ok &= expect(points[0].split('\t').last() == "estimated"
            && points[7].split('\t').last() == "1"
            && points[8].split('\t').last() == "1",
            "ICP pending and released samples retain estimated provenance");
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
    diagnostic.imageSampleBucket = 0;
    diagnostic.cameraTimestampMs = 1000;
    diagnostic.robotStateReceivedMs = 995;
    diagnostic.processingStartedMs = 1010;
    diagnostic.processingFinishedMs = 1020;
    diagnostic.algorithm.performance.available = true;
    diagnostic.algorithm.performance.totalMs = 8.0;
    diagnostic.algorithm.performance.trackingMs = 5.0;
    diagnostic.algorithm.performance.trainingMs = 3.0;
    diagnostic.robotState.curx_r = 123456;
    diagnostic.robotState.curRx_r = -25000;
    diagnostic.algorithm.attempted = true;
    diagnostic.algorithm.templateReinitialized = true;
    diagnostic.algorithm.gaborSucceeded = true;
    diagnostic.algorithm.simulatedPointAvailable = true;
    diagnostic.algorithm.simulatedPoint = {100, 200};
    diagnostic.algorithm.responsePoint = {101, 210};
    diagnostic.algorithm.beforeFusionPoint = {101, 210};
    diagnostic.algorithm.finalPoint = {100.5, 205};
    diagnostic.algorithm.coarseFinalPoint = {102, 209};
    diagnostic.algorithm.seamRefinementEnabled = true;
    diagnostic.algorithm.refinementAttempted = true;
    diagnostic.algorithm.refinement.horizontalValid = true;
    diagnostic.algorithm.refinement.valid = true;
    diagnostic.algorithm.refinement.status = vision::OnlineSeamRefinementStatus::Succeeded;
    diagnostic.algorithm.refinement.point = {99, 204};
    diagnostic.algorithm.refinement.leftEndpoint = {99, 208};
    diagnostic.algorithm.refinement.rightEndpoint = {99, 200};
    diagnostic.algorithm.refinement.heightJumpPixel = -8;
    diagnostic.algorithm.refinement.supportRatio = 0.95;
    diagnostic.algorithm.refinement.ambiguousColumnRatio = 0.125;
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
    bool measuredTimingFound = false;
    bool missingTimingFound = false;
    ok &= expect(recorder.flush(), "drain accepted text before offline readback");
    for (const auto &line : readRows(recorder.sessionDirectory(), "unified_session.jsonl")) {
        const auto event = QJsonDocument::fromJson(line).object();
        if (event.value("event") != "observation") { continue; }
        if (event.value("frame_id") == "42") {
            const auto timing = event.value("performance").toObject();
            measuredTimingFound = timing.value("processing_started_ms").toString() == "1010"
                && timing.value("processing_finished_ms").toString() == "1020"
                && timing.value("runtime_received_ms").isNull()
                && timing.value("detector").toObject().value("total_ms").toDouble() == 8.0
                && timing.value("detector").toObject().value("training_ms").toDouble() == 3.0;
        } else if (event.value("frame_id") == "43") {
            missingTimingFound = event.contains("performance") && event.value("performance").isNull();
        }
    }
    ok &= expect(measuredTimingFound && missingTimingFound,
        "session JSON retains measured stages and null unknown delivery, without recorder-time substitution");
    tracking.frameId = 44;
    tracking.diagnostic = diagnosticSnapshot;
    tracking.valid = true;
    tracking.sample.point.valid = true;
    tracking.sample.estimated = true;
    tracking.pointSource = vision::OnlineSeamPointSource::SimulationPrediction;
    tracking.degradedFrames = 360;
    tracking.diagnostic->algorithm.refinement.valid = false;
    ok &= expect(recorder.recordTrackingResult(tracking, control::OnlineCorrectionPhase::OnlineRunning),
                 "image pair cap must preserve numeric records");
    ok &= expect(recorder.flush(), "drain accepted text before offline readback");
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
                     && value(first, "message") == "diagnostic with context"
                     && value(first, "geometry_valid") == "1"
                     && value(first, "geometry_x_px") == "99.000000"
                     && value(first, "geometry_y_px") == "204.000000"
                     && value(first, "geometry_height_jump_px") == "-8.000000"
                     && value(first, "coarse_final_x_px") == "102.000000"
                     && value(first, "geometry_ambiguous_column_ratio") == "0.125000",
                     "same-frame stages, timestamp and robot units must survive serialization");
        ok &= expect(value(rejected, "response_peak").isEmpty()
                     && value(rejected, "robot_x_mm").isEmpty()
                     && value(rejected, "result_valid") == "0"
                     && value(rejected, "geometry_x_px").isEmpty()
                     && value(rejected, "geometry_status").isEmpty(),
                     "missing stages must be blank rather than valid zeroes");
        ok &= expect(value(capped, "estimated") == "1"
            && value(capped, "point_source") == "3"
            && value(capped, "degraded_frames") == "360"
            && value(capped, "final_y_px") == "205.000000"
            && value(capped, "geometry_y_px").isEmpty(),
            "prediction retains business coordinates without fabricating measured geometry");
        ok &= expect(value(capped, "image_request_status") == "pair_limit",
                     "image cap must be explicit");
    }
    tracking.frameId = 45;
    tracking.correlation.frameId = 45;
    tracking.correlation.workpiecePoseRevision = 1234;
    tracking.pointSource = vision::OnlineSeamPointSource::GuidedEstimate;
    tracking.sample.estimated = false;
    tracking.degradedFrames = 17;
    tracking.evidencePolicyEnabled = true;
    tracking.kcfPixelAvailable = true;
    tracking.controlEligible = false;
    auto &evidence = tracking.diagnostic->algorithm.evidence;
    evidence.enabled = true;
    evidence.anchorFrameId = 43;
    evidence.motherSupportRank = 1;
    evidence.elapsedMs = 200;
    evidence.travelMm = 0.7;
    evidence.message = "partial constraint; no control";
    auto &guided = tracking.diagnostic->algorithm.guided;
    guided.attempted = true;
    guided.valid = true;
    guided.hasTrustedAnchor = true;
    guided.predictionBudgetValid = true;
    guided.hasRawShoulderSupport = true;
    guided.localGapCenterSupported = false;
    guided.rawJumpRejected = true;
    guided.degradedFrames = 17;
    guided.anchorAgeMs = 2500;
    guided.referenceTravelPixel = 13.5;
    guided.leftSupportColumns = 32;
    guided.rightSupportColumns = 31;
    guided.ambiguousColumns = 2;
    guided.maximumBranchRmsPixel = 0.75;
    guided.prediction = {100, 205};
    guided.point = {100.5, 205};
    guided.reacquisitionPending = true;
    guided.reacquisitionFrames = 2;
    guided.imageContinuityActive = true;
    guided.imageObservationQuality = 0.73;
    guided.modelTranslationSuppressed = true;
    tracking.diagnostic->algorithm.tracker.onlineObservationMode = true;
    tracking.diagnostic->algorithm.tracker.trainingDeferred = true;
    tracking.diagnostic->algorithm.tracker.trainingCommitted = false;
    auto &fusionDiagnostic = tracking.diagnostic->algorithm;
    fusionDiagnostic.onlineObservationFusionEnabled = true;
    fusionDiagnostic.rawRefinement = diagnosticSnapshot->algorithm.refinement;
    fusionDiagnostic.coarseObservationQuality = 0.42;
    fusionDiagnostic.coarseObservationUsed = false;
    ok &= expect(recorder.recordTrackingResult(tracking, control::OnlineCorrectionPhase::OnlineRunning),
                 "record guided estimate even if caller forgot estimated provenance");
    ok &= expect(recorder.flush(), "drain accepted text before offline readback");
    const auto guidedRows = readRows(recorder.sessionDirectory(), "vision_diagnostics.tsv");
    ok &= expect(guidedRows.size() == 5, "guided diagnostic numeric record must persist");
    if (guidedRows.size() == 5) {
        const auto header = guidedRows.front().split('\t');
        const auto row = guidedRows.back().split('\t');
        const auto value = [&header, &row](const char *name) {
            const int index = header.indexOf(name);
            return index >= 0 && index < row.size() ? row[index] : QByteArray("MISSING");
        };
        ok &= expect(row.size() == header.size()
            && header.indexOf("guided_attempted") == header.indexOf("estimated") + 1
            && header.indexOf("online_observation_fusion_enabled")
                == header.indexOf("workpiece_pose_revision") + 1
            && header.indexOf("guided_image_continuity_active")
                == header.indexOf("guided_reacquisition_frames") + 1
            && header.indexOf("image_measurement_output_enabled")
                == header.indexOf("kcf_real_search_fallback_used") + 1
            && header.indexOf("evidence_policy_enabled") == header.indexOf("image_search_expanded") + 1
            && header.indexOf("debug_kcf_20261026_gates") == header.indexOf("evidence_message") + 1
            && header.last() == "debug_kcf_20261026_gates",
            "online fusion schema appends columns without moving existing fields");
        ok &= expect(value("estimated") == "1" && value("point_source") == "4"
            && value("geometry_valid") == "0" && value("geometry_x_px").isEmpty()
            && value("guided_valid") == "1" && value("guided_prediction_budget_valid") == "1"
            && value("guided_shoulder_support") == "1" && value("guided_gap_center_supported") == "0"
            && value("guided_raw_jump_rejected") == "1" && value("guided_anchor_age_ms") == "2500"
            && value("guided_reference_travel_px") == "13.500000"
            && value("guided_left_support_columns") == "32" && value("guided_right_support_columns") == "31"
            && value("guided_prediction_x_px") == "100.000000"
            && value("guided_x_px") == "100.500000" && value("workpiece_pose_revision") == "1234",
            "guided estimates retain support and pose provenance without becoming independent geometry");
        ok &= expect(value("evidence_policy_enabled") == "1"
            && value("kcf_pixel_available") == "1" && value("control_eligible") == "0"
            && value("evidence_anchor_frame") == "43" && value("evidence_mother_rank") == "1"
            && value("evidence_elapsed_ms") == "200.000000"
            && value("evidence_travel_mm") == "0.700000",
            "production estimate qualification and anchor budgets remain explicit in schema 9");
        ok &= expect(value("online_observation_fusion_enabled") == "1"
            && value("raw_geometry_valid") == "1" && value("geometry_valid") == "0"
            && value("raw_geometry_x_px") == "99.000000"
            && value("raw_geometry_y_px") == "204.000000"
            && value("coarse_observation_quality") == "0.420000"
            && value("coarse_observation_used") == "0"
            && value("observation_weight_x") == "0.000000"
            && value("guided_reacquisition_pending") == "1"
            && value("guided_reacquired") == "0"
            && value("guided_reacquisition_blocked_previous_branch") == "0"
            && value("guided_reacquisition_frames") == "2"
            && value("guided_image_continuity_active") == "1"
            && value("guided_image_observation_quality") == "0.730000"
            && value("guided_model_translation_suppressed") == "1"
            && value("kcf_online_observation_mode") == "1"
            && value("kcf_training_deferred") == "1"
            && value("kcf_training_committed") == "0",
            "raw candidate and reacquisition diagnostics do not invent accepted geometry or weight");
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
    ok &= expect(recorder.flush(), "drain accepted text before offline readback");
    const auto sourceRows = readRows(recorder.sessionDirectory(), "realtime_correction.txt");
    const auto sourceHeader = sourceRows.front().split('\t');
    const auto sourceRow = sourceRows.back().split('\t');
    const int estimatedIndex = sourceHeader.indexOf("estimated");
    const int sourceIndex = sourceHeader.indexOf("point_source");
    ok &= expect(sourceRow.size() == sourceHeader.size()
        && estimatedIndex >= 0 && sourceIndex >= 0
        && sourceRow[estimatedIndex] == "1" && sourceRow[sourceIndex] == "4",
        "business log forces guided estimated provenance even with diagnostics disabled");
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
