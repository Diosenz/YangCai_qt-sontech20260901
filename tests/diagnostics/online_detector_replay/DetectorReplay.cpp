/**
 * @file DetectorReplay.cpp
 * @brief 按曝光顺序调用真实 process 入口，导出公开结果与诊断。
 * @details 一个会话持有一个检测器，无线程、通信、设备或运动。间隙检测禁用。
 */
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
    for (const auto *key : {"bounded_search", "enable_gap"}) {
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
    row["search_cycle_pending"] = d.imageSearchCyclePending;
    row["candidate_coverage"] = "public selected raw candidate only; exhaustive candidates not exposed";
    row["projection_provenance"] = input.value("projection").toObject().value("provenance");
    row["projection_kind"] = input.value("projection").toObject().value("fixture").isString()
        ? input.value("projection").toObject().value("fixture") : QJsonValue("provided_or_unknown");
    return row;
}
}

int main(int argc, char **argv)
{
    try {
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
            dt::DigitalTwinVisionLaserObservation observation;
            observation.frameId = input.value("frame_id").isDouble()
                ? input.value("frame_id").toInteger() : -1;
            const auto sim = input.value("sim_point").toArray();
            if (sim.size() == 2) {
                observation.valid = observation.hasWeldPointImage_px = true;
                observation.weldPointImage_px = {sim[0].toDouble(), sim[1].toDouble()};
            }
            observation.seamIndex = input.value("seam_index").toInt(-1);
            v::LegacyOnlineKcfOptions options;
            options.enableImageMeasurementOutput = true;
            options.enableSeamRefinement = true;
            options.enableSimulationGuidance = true;
            options.enableOnlineObservationFusion = false;
            options.enableBoundedImageSearch = input.value("bounded_search").toBool(false);
            options.seamRefinement.heightPolicy = v::OnlineSeamHeightPolicy::GapCenter;
            options.referenceRevision = input.value("reference_revision").toInteger(0);
            cv::Mat image;
            if (input.value("image").isString()) {
                image = cv::imread(directory.filePath(input.value("image").toString()).toStdString(), cv::IMREAD_GRAYSCALE);
                if (image.empty()) { throw std::runtime_error("declared image missing or unreadable"); }
            }
            const auto result = detector.process(image, observation,
                projection(input.value("projection").toObject()), options);
            auto row = record(input, result);
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
