/**
 * @file DetectorReplay.cpp
 * @brief 按曝光顺序调用真实 process 入口，导出公开结果与诊断。
 * @details 一个会话持有一个检测器，无线程、通信、设备或运动。间隙检测禁用。
 */
#include <chrono>
#include <cmath>
#include <exception>
#include <Eigen/Geometry>
#include <cstdio>
#include <stdexcept>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <opencv2/imgcodecs.hpp>
#include "feature/ActiveContourWeldFeatureDetector.h"
#include "track/LegacyOnlineKcfDetector.h"

namespace v = i2r_rlc::vision;
namespace dt = i2r_rlc::digital_twin;
namespace {
QJsonValue pixel(const cv::Point2d &point, bool valid)
{
    if (!valid || !std::isfinite(point.x) || !std::isfinite(point.y)) {
        return QJsonValue(QJsonValue::Null);
    }
    return QJsonArray{point.x, point.y};
}

/** @brief 拒绝损坏的身份和开关，未知值保留 null，不做静默类型转换。 */
void validateInput(const QJsonObject &input)
{
    for (const auto *key : {"frame_id", "reference_revision", "seam_index"}) {
        const auto value = input.value(key);
        if (value.isUndefined() || value.isNull()) { continue; }
        const double number = value.toDouble();
        if (!value.isDouble() || !std::isfinite(number)
            || std::floor(number) != number || std::abs(number) >= 9007199254740992.0
            || (QString(key) == "reference_revision" && number < 0)) {
            throw std::runtime_error(std::string("invalid integer ") + key);
        }
    }
    for (const auto *key : {"bounded_search", "enable_gap", "evidence_policy",
             "paper_tracking", "probe_bootstrap", "performance_diagnostics",
             "simulation_valid", "has_weld_point"}) {
        if (input.contains(key) && !input.value(key).isBool()) {
            throw std::runtime_error(std::string("invalid boolean ") + key);
        }
    }
    if (input.contains("image") && !input.value("image").isNull()
        && !input.value("image").isString()) {
        throw std::runtime_error("image must be path or null");
    }
    const auto time = input.value("timestamp_ms");
    if (!time.isUndefined() && !time.isNull()
        && (!time.isDouble() || !std::isfinite(time.toDouble()))) {
        throw std::runtime_error("timestamp_ms must be finite or null");
    }
    if (input.contains("sim_point")) {
        const auto point = input.value("sim_point").toArray();
        if (point.size() != 2 || !point[0].isDouble() || !point[1].isDouble()) {
            throw std::runtime_error("sim_point needs two numeric pixels");
        }
    }
}

/** @brief 只接受显式完整上下文；缺失不使用默认有效标定。 */
v::WeldProjectionContext projection(const QJsonObject &input)
{
    v::WeldProjectionContext result;
    if (input.value("fixture") == "synthetic_identity_1mm_per_pixel") {
        result.valid = true;
        result.calibration.valid = true;
        result.calibration.focalLengthM = 0.01;
        result.calibration.sxM = result.calibration.syM = 0.00001;
        result.calibration.intrinsics.cxPixel = 320;
        result.calibration.intrinsics.cyPixel = 240;
        result.calibration.laserPlaneC = -1;
        return result;
    }
    if (input.isEmpty()) { return result; }
    const auto calibration = input.value("calibration").toObject();
    const auto required = {"focal_length_m", "sx_m", "sy_m", "cx_pixel",
        "cy_pixel", "kappa", "laser_plane_a", "laser_plane_b", "laser_plane_c"};
    for (const auto *key : required) {
        if (!calibration.value(key).isDouble()) {
            throw std::runtime_error("projection calibration incomplete");
        }
    }
    auto &c = result.calibration;
    c.focalLengthM = calibration.value("focal_length_m").toDouble();
    c.sxM = calibration.value("sx_m").toDouble();
    c.syM = calibration.value("sy_m").toDouble();
    c.intrinsics.cxPixel = calibration.value("cx_pixel").toDouble();
    c.intrinsics.cyPixel = calibration.value("cy_pixel").toDouble();
    c.kappa = calibration.value("kappa").toDouble();
    c.laserPlaneA = calibration.value("laser_plane_a").toDouble();
    c.laserPlaneB = calibration.value("laser_plane_b").toDouble();
    c.laserPlaneC = calibration.value("laser_plane_c").toDouble();
    const auto transform = [&input](const char *key) {
        const auto a = input.value(key).toArray();
        if (a.size() != 12) { throw std::runtime_error("transform needs 9 rotation + 3 translation values"); }
        v::WeldRigidTransform t;
        for (int i = 0; i < 12; ++i) {
            if (!a[i].isDouble()) { throw std::runtime_error("transform values must be numeric"); }
        }
        for (int i = 0; i < 9; ++i) { t.rotation(i / 3, i % 3) = a[i].toDouble(); }
        for (int i = 0; i < 3; ++i) { t.translationMm(i) = a[i + 9].toDouble(); }
        if (!t.rotation.allFinite() || !t.translationMm.allFinite()
            || (t.rotation.transpose() * t.rotation - Eigen::Matrix3d::Identity()).norm() > 1e-6
            || std::abs(t.rotation.determinant() - 1.0) > 1e-6) {
            throw std::runtime_error("transform must be finite rigid pose");
        }
        return t;
    };
    c.cameraInTool = transform("camera_in_tool");
    result.toolInRobotBase = transform("tool_in_robot_base");
    result.robotBaseToPositionerTcp = transform("robot_base_to_positioner_tcp");
    c.valid = result.valid = true;
    return result;
}

/** @brief 一帧记录仅暴露生产公开诊断；不猜造未公开的候选集合。 */
QJsonObject record(const QJsonObject &input, const v::LegacyOnlineKcfResult &r)
{
    const auto &d = r.diagnostic;
    const QString source = d.continuity.source == v::OnlineSeamPointSource::Geometry
        ? "geometry" : (d.coarseObservationUsed ? "kcf_identity" : "unavailable");
    QJsonObject row;
    for (const auto *key : {"frame_id", "timestamp_ms", "reference_revision", "kind", "image"}) {
        row[key] = input.contains(key) ? input.value(key) : QJsonValue(QJsonValue::Null);
    }
    row["available"] = r.valid;
    row["detector_valid"] = r.valid;
    row["detector_pixel"] = pixel(r.weldPointPixel, r.valid);
    row["paper_tracking"] = input.value("paper_tracking").toBool(false);
    row["response_template_count"] = d.tracker.responseTemplateCount;
    row["position_fusion"] = QJsonObject{
        {"applied", d.positionFusionApplied},
        {"gain_x", d.positionGainX}, {"gain_y", d.positionGainY},
        {"observation", pixel(d.positionObservationPoint, d.positionFusionApplied)},
        {"prior", pixel(d.positionPriorPoint, d.positionFusionApplied)}};
    row["simulation_shape_available"] = d.simulationShapeAvailable;
    row["paper_diagnostics"] = QJsonObject{
        {"mother_supported", d.motherBranchesSupported},
        {"mother_seeded", d.motherBranchesSeeded},
        {"mother_used", d.motherBranchEstimateUsed},
        {"mother_point", pixel(d.motherBranchPoint, d.motherBranchesSupported)},
        {"identity_quality", d.coarseObservationQuality},
        {"shoulder_accepted", d.shoulderObservationAccepted},
        {"shoulder_quality", d.shoulderObservationQuality},
        {"center_quality", d.centerObservationQuality},
        {"complete_appearance", d.completeAppearanceSupported},
        {"response_statistics_ready", d.responseStatisticsReady},
        {"response_normal", d.responseNormal},
        {"normal_response_samples", d.normalResponseSamples},
        {"response_psr", d.tracker.responsePsr},
        {"normal_response_mean", d.normalResponseMean},
        {"normal_response_std", d.normalResponseStd},
        {"training_from_simulation", d.tracker.trainingFromSimulation},
        {"training_rejected", d.tracker.trainingRejected}};
    row["image_observation_accepted"] = d.imageObservationAccepted;
    row["source"] = r.valid ? source : "unavailable";
    row["image_source"] = source;
    row["trust"] = r.valid ? (d.refinement.valid ? "independent_geometry" : "tracked_not_independent") : "unavailable";
    row["final_pixel"] = pixel(r.weldPointPixel, r.valid);
    row["accepted_image_pixel"] = pixel(d.finalPoint, d.imageObservationAccepted);
    row["raw_measurement_pixel"] = pixel(d.rawRefinement.point, d.rawRefinement.valid);
    row["raw_kcf_pixel"] = pixel(d.beforeFusionPoint, d.tracker.completed);
    row["positioner_tcp_mm"] = r.valid ? QJsonValue(QJsonArray{
        r.stationPoint.xMm, r.stationPoint.yMm, r.stationPoint.zMm}) : QJsonValue(QJsonValue::Null);
    row["message"] = QString::fromStdString(r.message);
    row["refinement_status"] = static_cast<int>(d.refinement.status);
    row["raw_refinement_status"] = static_cast<int>(d.rawRefinement.status);
    row["ambiguous_column_ratio"] = d.rawRefinement.ambiguousColumnRatio;
    row["support_ratio"] = d.rawRefinement.supportRatio;
    row["fit_rms_pixel"] = d.rawRefinement.fitRmsPixel;
    row["kcf_response_peak"] = d.tracker.hasResponse ? QJsonValue(d.tracker.responsePeak) : QJsonValue(QJsonValue::Null);
    row["training_committed"] = d.tracker.trainingCommitted;
    row["template_reinitialized"] = d.templateReinitialized;
    row["frames_since_initialize"] = d.framesSinceInitialize;
    row["simulated_point_available"] = d.simulatedPointAvailable;
    row["image_search_expanded"] = d.imageSearchExpanded;
    row["bounded_search"] = d.boundedImageSearchEnabled;
    row["refinement_calls"] = d.imageRefinementCallCount;
    row["image_search_diagnostic"] = QString::fromStdString(d.imageSearchDiagnostic);
    row["search_cycle_pending"] = d.imageSearchCyclePending;
    row["candidate_coverage"] = "public selected raw candidate only; exhaustive candidates not exposed";
    row["projection_provenance"] = input.value("projection").toObject().value("provenance");
    row["projection_kind"] = input.value("projection").toObject().value("fixture").isString()
        ? input.value("projection").toObject().value("fixture") : QJsonValue("provided_or_unknown");
    const auto &e = d.evidence;
    row["production_evidence_policy"] = e.enabled;
    row["control_eligible"] = e.controlEligible;
    if (e.enabled) {
        row["debug_kcf_20261026_gates"] = QString::fromStdString(e.debugDetail);
        const char *name = "unavailable";
        switch (e.source) {
        case v::OnlineSeamPointSource::Geometry: name = "geometry"; break;
        case v::OnlineSeamPointSource::CorrectedKcf: name = "appearance_tracking"; break;
        case v::OnlineSeamPointSource::SimulationPrediction: name = "bounded_prediction"; break;
        case v::OnlineSeamPointSource::GuidedEstimate: name = "mother_material_estimate"; break;
        case v::OnlineSeamPointSource::RecoveryTransition: name = "recovery_transition"; break;
        default: break;
        }
        row["source"] = name;
        row["available"] = e.pixelAvailable;
        row["final_pixel"] = pixel(e.point, e.pixelAvailable);
        row["trust"] = e.independentGeometry ? "independent_geometry"
            : (e.pixelAvailable ? "estimated_not_independent" : "unavailable");
        row["can_independent_validate"] = e.independentGeometry;
        row["control_eligible"] = e.controlEligible;
        row["space_available"] = r.valid;
        row["fresh_exposure"] = e.freshExposure;
        row["anchor_frame_id"] = static_cast<qint64>(e.anchorFrameId);
        row["anchor_refreshed"] = e.anchorRefreshed;
        row["confirmation_count"] = e.confirmationCount;
        row["mother_support_rank"] = e.motherSupportRank;
        row["prediction_elapsed_ms"] = e.elapsedMs;
        row["prediction_travel_mm"] = e.travelMm;
    }
    const auto &performance = d.performance;
    row["performance"] = performance.available ? QJsonValue(QJsonObject{
        {"total_ms", performance.totalMs},
        {"input_bookkeeping_ms", performance.inputBookkeepingMs},
        {"simulation_ms", performance.simulationMs},
        {"initialization_ms", performance.initializationMs},
        {"tracking_ms", performance.trackingMs},
        {"validation_projection_ms", performance.validationProjectionMs},
        {"training_ms", performance.trainingMs}}) : QJsonValue(QJsonValue::Null);
    return row;
}
}

int main(int argc, char **argv)
{
    try {
        if (argc == 3 && std::string(argv[1]) == "--tracker-benchmark") {
            const cv::Mat image = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
            if (image.empty()) { throw std::runtime_error("real benchmark image missing"); }
            const cv::Rect roi(425, 485, 250, 200);
            v::CKcfTracker tracker(true, true, false, false, v::KcfFeatureMode::Mkcf, 3);
            v::KcfFusionOptions fusion;
            fusion.mode = v::KcfFusionMode::KcfOnly;
            tracker.setFusionOptions(fusion);
            tracker.setOnlineObservationMode(true);
            tracker.init(roi, image);
            tracker.setSimulatedTemplate(image, roi);
            for (int index = 0; index < 4; ++index) {
                const auto started = std::chrono::steady_clock::now();
                tracker.update(image);
                const auto trainingStarted = std::chrono::steady_clock::now();
                tracker.commitOnlineObservationTraining(true);
                const auto finished = std::chrono::steady_clock::now();
                const auto &diagnostic = tracker.lastUpdateDiagnostic();
                const QJsonObject row{
                    {"scope", "Public tracker benchmark; captured real image also substitutes reference appearance; not field simulation truth."},
                    {"iteration", index},
                    {"threads", cv::getNumThreads()},
                    {"processing_ms", std::chrono::duration<double, std::milli>(finished - started).count()},
                    {"training_ms", std::chrono::duration<double, std::milli>(finished - trainingStarted).count()},
                    {"templates", diagnostic.responseTemplateCount},
                    {"training_committed", diagnostic.trainingCommitted},
                    {"response_peak", diagnostic.responsePeak},
                    {"response_psr", diagnostic.responsePsr},
                    {"hog_weight", diagnostic.responseHogWeight},
                    {"gray_weight", diagnostic.responseGrayWeight},
                    {"point", QJsonArray{tracker.m_roi.x, tracker.m_roi.y}}};
                const auto encoded = QJsonDocument(row).toJson(QJsonDocument::Compact);
                std::printf("%s\n", encoded.constData());
            }
            return 0;
        }
        if (argc != 2) { throw std::runtime_error("usage: DetectorReplay manifest.jsonl"); }
        QFile file(QString::fromLocal8Bit(argv[1]));
        if (!file.open(QIODevice::ReadOnly)) { throw std::runtime_error("cannot open manifest"); }
        const QDir directory = QFileInfo(file).absoluteDir();
        v::CLegacyOnlineKcfDetector detector;
        while (!file.atEnd()) {
            const auto line = file.readLine().trimmed();
            if (line.isEmpty()) { continue; }
            QJsonParseError error;
            const auto document = QJsonDocument::fromJson(line, &error);
            if (error.error != QJsonParseError::NoError || !document.isObject()) {
                throw std::runtime_error("invalid JSONL object");
            }
            const auto input = document.object();
            validateInput(input);
            if (input.value("enable_gap").toBool()) { throw std::runtime_error("gap excluded; enable_gap must be false"); }
            if (input.value("projection").toObject().contains("fixture")
                && input.value("kind") != "synthetic") {
                throw std::runtime_error("synthetic calibration cannot be used for real images");
            }
            if (input.contains("detector_actions")) {
                if (!input.value("detector_actions").isArray()) {
                    throw std::runtime_error("detector_actions must be an array");
                }
                for (const auto &action : input.value("detector_actions").toArray()) {
                    if (action == QJsonValue("reset")) { detector.reset(); }
                    else if (action == QJsonValue("restart")) { detector.restartTracking(); }
                    else if (action == QJsonValue("gap")) { detector.notifyImageObservationGap(); }
                    else { throw std::runtime_error("unknown detector lifecycle action"); }
                }
            }
            dt::DigitalTwinVisionLaserObservation observation;
            observation.frameId = input.value("frame_id").isDouble()
                ? input.value("frame_id").toInteger() : -1;
            const auto sim = input.value("sim_point").toArray();
            if (sim.size() == 2) {
                observation.valid = observation.hasWeldPointImage_px = true;
                observation.weldPointImage_px = {sim[0].toDouble(), sim[1].toDouble()};
            }
            observation.valid = input.value("simulation_valid").toBool(observation.valid);
            observation.hasWeldPointImage_px = input.value("has_weld_point").toBool(observation.hasWeldPointImage_px);
            observation.seamIndex = input.value("seam_index").toInt(-1);
            for (const auto &value : input.value("sim_line").toArray()) {
                const auto point = value.toArray();
                if (point.size() != 2 || !point[0].isDouble() || !point[1].isDouble()) {
                    throw std::runtime_error("sim_line needs numeric pixel pairs");
                }
                observation.laserLineImagePoints_px.push_back(
                    {point[0].toDouble(), point[1].toDouble()});
            }
            v::LegacyOnlineKcfOptions options;
            options.enablePaperTracking = input.value("paper_tracking").toBool(false);
            // 离线比较使用生产选项，拒绝拼错键和非物理噪声，不能静默回退默认值。
            if (input.contains("paper_options") && !input.value("paper_options").isObject()) {
                throw std::runtime_error("paper_options must be an object");
            }
            const auto tuning = input.value("paper_options").toObject();
            for (auto it = tuning.begin(); it != tuning.end(); ++it) {
                if (it.key() == "fuse_partial" && it.value().isBool()) {
                    options.paperFusePartialObservations = it.value().toBool();
                    continue;
                }
                if (it.key() == "branch_constraints" && it.value().isBool()) {
                    options.paperUseMotherBranches = it.value().toBool();
                    continue;
                }
                if (it.key() == "keep_partial_state" && it.value().isBool()) {
                    options.paperKeepPartialState = it.value().toBool();
                    continue;
                }
                if (!it.value().isDouble() || !std::isfinite(it.value().toDouble())
                    || it.value().toDouble() <= 0.0) {
                    throw std::runtime_error("paper option must be finite and positive");
                }
                const double value = it.value().toDouble();
                if (it.key() == "psr_sigma") { options.paperPsrSigma = value; }
                else if (it.key() == "process_noise") { options.paperProcessNoise = value; }
                else if (it.key() == "noise_minimum") { options.paperObservationNoiseMinimum = value; }
                else if (it.key() == "noise_maximum") { options.paperObservationNoiseMaximum = value; }
                else if (it.key() == "abnormal_multiplier") { options.paperAbnormalNoiseMultiplier = value; }
                else if (it.key() == "partial_multiplier") { options.paperPartialNoiseMultiplier = value; }
                else { throw std::runtime_error("unknown paper option"); }
            }
            if (options.paperObservationNoiseMinimum > options.paperObservationNoiseMaximum) {
                throw std::runtime_error("paper observation noise bounds reversed");
            }
            options.enablePerformanceDiagnostics = input.value("performance_diagnostics").toBool(false);
            options.enableImageMeasurementOutput = true;
            options.enableSeamRefinement = true;
            options.enableSimulationGuidance = true;
            options.enableOnlineObservationFusion = false;
            options.enableBoundedImageSearch = input.value("bounded_search").toBool(false);
            options.seamRefinement.heightPolicy = v::OnlineSeamHeightPolicy::GapCenter;
            options.referenceRevision = input.value("reference_revision").toInteger(0);
            options.enableEvidencePolicy = input.value("evidence_policy").toBool(false);
            auto &context = options.evidenceContext;
            context.timestampMs = input.value("timestamp_ms").toInteger(-1);
            const auto readPoint = [&input](const char *key) -> std::optional<cv::Point2d> {
                const auto p = input.value(key).toArray();
                if (p.size() != 2 || !p[0].isDouble() || !p[1].isDouble()
                    || !std::isfinite(p[0].toDouble()) || !std::isfinite(p[1].toDouble())) { return std::nullopt; }
                return cv::Point2d(p[0].toDouble(), p[1].toDouble());
            };
            context.referencePixel = readPoint("reference_point_at_exposure");
            context.referenceExposureFrameId = input.value("reference_exposure_frame_id").toInteger(-1);
            context.previousReferencePixel = readPoint("previous_reference_point_at_exposure");
            context.previousReferenceRevision = input.value("previous_reference_revision").toInteger(0);
            context.previousReferenceExposureFrameId = input.value("previous_reference_exposure_frame_id").toInteger(-1);
            if (input.value("weld_progress_mm").isDouble()) {
                context.weldProgressMm = input.value("weld_progress_mm").toDouble();
            }
            context.progressReference = input.value("progress_reference").toString().toStdString();
            context.progressProvenance = input.value("progress_provenance").toString().toStdString();
            context.progressExposureFrameId = input.value("progress_exposure_frame_id").toInteger(-1);
            options.evidenceOptions.maxPredictionMs = input.value("max_prediction_ms").toDouble(0);
            options.evidenceOptions.maxPredictionTravelMm = input.value("max_prediction_travel_mm").toDouble(0);
            const auto policy = input.value("evidence_options").toObject();
            auto &evidenceOptions = options.evidenceOptions;
            for (auto it = policy.begin(); it != policy.end(); ++it) {
                const auto key = it.key();
                if (key == "enable_continuous_output" && it.value().isBool()) {
                    evidenceOptions.enableContinuousOutput = it.value().toBool();
                    continue;
                }
                if (key == "enable_mother_constraints" && it.value().isBool()) {
                    evidenceOptions.enableMotherConstraints = it.value().toBool();
                    continue;
                }
                if (!it.value().isDouble() || !std::isfinite(it.value().toDouble())) {
                    throw std::runtime_error("production evidence option must be numeric or enable_mother_constraints boolean");
                }
                const double value = it.value().toDouble();
                if (key == "anchor_confirmation_frames" || key == "recovery_confirmation_frames"
                    || key == "identity_corridor_pixel") {
                    if (value < 1 || value > 10000 || std::floor(value) != value) {
                        throw std::runtime_error("invalid production evidence integer option");
                    }
                    if (key == "anchor_confirmation_frames") { evidenceOptions.anchorConfirmationFrames = static_cast<int>(value); }
                    else if (key == "recovery_confirmation_frames") { evidenceOptions.recoveryConfirmationFrames = static_cast<int>(value); }
                    else { evidenceOptions.identityCorridorPixel = static_cast<int>(value); }
                } else if (key == "max_prediction_ms") { evidenceOptions.maxPredictionMs = value; }
                else if (key == "max_prediction_travel_mm") { evidenceOptions.maxPredictionTravelMm = value; }
                else if (key == "max_continuous_step_pixel") { evidenceOptions.maxContinuousStepPixel = value; }
                else if (key == "max_recovery_step_pixel") { evidenceOptions.recoveryMaxStepPixel = value; }
                else if (key == "identity_min_patch_score") { evidenceOptions.identityMinPatchScore = value; }
                else if (key == "identity_min_branch_score") { evidenceOptions.identityMinBranchScore = value; }
                else if (key == "identity_min_uniqueness") { evidenceOptions.identityMinUniqueness = value; }
                else { throw std::runtime_error("unknown production evidence option"); }
            }
            cv::Mat image;
            if (input.value("image").isString()) {
                image = cv::imread(directory.filePath(input.value("image").toString()).toStdString(), cv::IMREAD_GRAYSCALE);
                if (image.empty()) { throw std::runtime_error("declared image missing or unreadable"); }
            }
            const auto result = detector.process(image, observation,
                projection(input.value("projection").toObject()), options);
            auto row = record(input, result);
            if (input.value("probe_bootstrap").toBool(false)) {
                v::ActiveContourDetectionState bootstrapState;
                const v::ActiveContourDetectionOptions bootstrapOptions;
                const auto candidate = v::CActiveContourWeldFeatureDetector::detect(
                    image, v::ActiveContourSeamMode::TJoint, bootstrapOptions, &bootstrapState);
                row["bootstrap_proposal"] = pixel(
                    {candidate.weldPoint.xPixel, candidate.weldPoint.yPixel}, candidate.valid);
            }
            row["image_size"] = image.empty() ? QJsonValue(QJsonValue::Null)
                : QJsonValue(QJsonArray{image.cols, image.rows});
            if (!image.empty()) {
                std::vector<unsigned char> png;
                cv::imencode(".png", image, png);
                row["image_png_base64"] = QString::fromLatin1(QByteArray(
                    reinterpret_cast<const char *>(png.data()),
                    static_cast<qsizetype>(png.size())).toBase64());
            }
            const auto encoded = QJsonDocument(row).toJson(QJsonDocument::Compact);
            std::fwrite(encoded.constData(), 1, static_cast<std::size_t>(encoded.size()), stdout);
            std::fputc('\n', stdout);
        }
    } catch (const std::exception &error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 2;
    }
    return 0;
}
