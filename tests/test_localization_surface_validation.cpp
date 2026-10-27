/**
 * @file test_localization_surface_validation.cpp
 * @brief 表面弱模式冻结、局部凸起及分时留出恶化的合成反例。
 * @details 默认合成几何与真值变换独立给定；可选--replay-dir只读回放现场归档，均不访问设备。
 * 合成通过只验证此验收器的边界，不能作为图像提取或现场配准可靠性证明。
 */
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include "LocalizationSurfaceValidation.h"

namespace {
namespace surface = i2r_rlc::localization::surface_validation;
using Point = i2r_rlc::localization::Point3D_mm;
using Transform = i2r_rlc::localization::RigidTransform;

bool expect(bool condition, const char *message)
{
    if (!condition) { std::fprintf(stderr, "FAIL: %s\n", message); }
    return condition;
}

Transform correction(double x, double y, double z, double yaw)
{
    Transform result;
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    result.rowMajor = {cosine, -sine, 0.0, x, sine, cosine, 0.0, y,
        0.0, 0.0, 1.0, z, 0.0, 0.0, 0.0, 1.0};
    return result;
}

std::vector<Point> move(const std::vector<Point> &cloud, const Transform &transform)
{
    std::vector<Point> result;
    const auto &matrix = transform.rowMajor;
    for (const auto &point : cloud) {
        result.push_back({matrix[0] * point.x_mm + matrix[1] * point.y_mm + matrix[3],
            matrix[4] * point.x_mm + matrix[5] * point.y_mm + matrix[7],
            point.z_mm + matrix[11]});
    }
    return result;
}

/** @brief 离原点较远的网格验证中心化转角/平移转换，而非只检查秩。 */
std::vector<Point> plane()
{
    std::vector<Point> result;
    for (int x = -8; x <= 8; ++x) {
        for (int y = -8; y <= 8; ++y) {
            result.push_back({100.0 + 2.0 * x, -70.0 + 2.0 * y, 10.0});
        }
    }
    return result;
}

std::vector<Point> twoSides()
{
    std::vector<Point> result;
    for (int x = -10; x <= 10; ++x) {
        if (x == 0) { continue; }
        for (int y = -8; y <= 8; ++y) {
            result.push_back({100.0 + 2.0 * x, -70.0 + 2.0 * y,
                10.0 - std::abs(x) * 0.8});
        }
    }
    return result;
}

std::vector<Point> threePatches()
{
    auto result = plane();
    // 三个互不重叠的可见面，避免中心交叠邻域掩盖解析法向。
    for (int first = -8; first <= 8; ++first) {
        for (int second = -8; second <= 8; ++second) {
            result.push_back({130.0, -70.0 + 2.0 * first, 40.0 + 2.0 * second});
            result.push_back({100.0 + 2.0 * first, -40.0, 80.0 + 2.0 * second});
        }
    }
    return result;
}

surface::Request request(const std::vector<Point> &reference,
    const Transform &truth, const Transform &candidate)
{
    surface::Request result;
    result.trainingVirtual_mm = reference;
    result.trainingReal_mm = move(reference, truth);
    // 此处仅测试验收行为；真实接线必须由不同完整帧构造留出。
    result.holdoutVirtual_mm = reference;
    result.holdoutReal_mm = move(reference, truth);
    result.candidateVirtualToReal = candidate;
    result.options.coverageDistanceMm = 1.0;
    result.options.maximumResidualMm = 2.0;
    return result;
}

bool planeFreezesTangentialAndYaw()
{
    const auto input = request(plane(), correction(0.0, 0.0, 0.6, 0.0),
        correction(3.0, -4.0, 0.6, 0.05));
    const auto result = surface::evaluate(input);
    const auto &matrix = result.observableVirtualToReal.rowMajor;
    return expect(result.accepted && result.projected && result.observableRank == 1,
        "flat plane accepts only an improving normal correction")
        && expect(std::abs(matrix[3]) < 1e-6 && std::abs(matrix[7]) < 1e-6
            && std::abs(matrix[1]) < 1e-6 && std::abs(matrix[11] - 0.6) < 1e-6,
            "plane projection freezes tangential translations and yaw")
        && expect(result.holdoutAfter.robustSymmetricResidualMm < 1e-6,
            "normal-only correction preserves heldout plane geometry");
}

bool vShapeFreezesAlongSeamAtCenter()
{
    const auto cloud = twoSides();
    const double yaw = 0.01;
    // 指定绕远处母材中心旋转，中心沿缝位移为零。
    const double x = 100.0 + 0.4 - std::cos(yaw) * 100.0 - std::sin(yaw) * 70.0;
    const double y = -70.0 - std::sin(yaw) * 100.0 + std::cos(yaw) * 70.0;
    const auto result = surface::evaluate(request(cloud,
        correction(x, y, 0.6, yaw), correction(x, y + 4.0, 0.6, yaw)));
    const auto &matrix = result.observableVirtualToReal.rowMajor;
    const double centerAlongSeam = matrix[4] * 100.0 + matrix[5] * -70.0 + matrix[7] + 70.0;
    return expect(result.accepted && result.projected && result.observableRank == 3,
        "extruded two-sided V has only three surface constraints")
        && expect(std::abs(centerAlongSeam) < 0.1,
            "V projection freezes along-seam displacement at the centered lever arm")
        && expect(std::abs(std::atan2(matrix[4], matrix[0]) - yaw) < 0.001
            && std::abs(matrix[11] - 0.6) < 0.01,
            "V retains observable yaw and normal correction after centering");
}

bool fullGeometryAndBump()
{
    const auto cloud = threePatches();
    const auto truth = correction(0.4, -0.2, 0.6, 0.015);
    auto input = request(cloud, truth, truth);
    const auto clear = surface::evaluate(input);
    bool ok = expect(clear.accepted && clear.observableRank == 4 && !clear.projected,
        "three nonparallel patches preserve an improving full candidate");
    for (std::size_t index = 0; index < input.trainingReal_mm.size(); ++index) {
        if (index % 17 == 0) { input.trainingReal_mm[index].z_mm += 6.0; }
    }
    const auto bump = surface::evaluate(input);
    ok &= expect(bump.accepted && bump.trainingAfter.realCoverageRatio > 0.9,
        "sparse local bumps do not require moving the whole workpiece");
    input.candidateVirtualToReal = correction(-1.0, 1.0, -0.5, -0.02);
    const auto wrong = surface::evaluate(input);
    ok &= expect(!wrong.accepted, "wrong correction cannot pass surface gain/coverage");
    return ok;
}

bool heldoutWorseningAndMissingEvidence()
{
    auto input = request(plane(), correction(0.0, 0.0, 0.6, 0.0),
        correction(0.0, 0.0, 0.6, 0.0));
    input.holdoutReal_mm = move(input.holdoutVirtual_mm, correction(0.0, 0.0, -0.6, 0.0));
    const auto result = surface::evaluate(input);
    bool ok = expect(!result.accepted && result.trainingAfter.robustSymmetricResidualMm < 1e-6
        && result.holdoutAfter.robustSymmetricResidualMm > result.holdoutBefore.robustSymmetricResidualMm,
        "training improvement cannot hide heldout worsening");
    input.holdoutReal_mm.clear();
    ok &= expect(!surface::evaluate(input).accepted, "missing holdout must reject");
    input = request(plane(), Transform{}, Transform{});
    ok &= expect(!surface::evaluate(input).accepted, "identity with no improvement rejects");
    input.trainingReal_mm[0].x_mm = std::numeric_limits<double>::quiet_NaN();
    ok &= expect(!surface::evaluate(input).accepted, "nonfinite input rejects");
    std::vector<Point> line;
    for (int index = 0; index < 60; ++index) { line.push_back({index * 1.0, 0.0, 0.0}); }
    ok &= expect(!surface::evaluate(request(line, correction(0.0, 0.0, 0.6, 0.0),
        correction(0.0, 0.0, 0.6, 0.0))).hasObservableCorrection,
        "near-collinear neighborhoods must not fabricate surface normals");
    return ok;
}

bool freshSurfaceUsesSamePointsAndBoundedPatches()
{
    auto reference = plane();
    std::vector<Point> fresh;
    // 新切片横跨训练网格之间；采样密度和位置不同，但解析母材面未改变。
    for (int index = -25; index <= 25; ++index) {
        fresh.push_back({100.0 + 0.4 * index, -69.1, 10.6});
    }
    surface::Options options;
    const auto truth = correction(0.0, 0.0, 0.6, 0.0);
    const auto good = surface::compareFreshSurface(fresh, reference, truth, options);
    bool ok = expect(good.valid && good.supportedRatio > 0.99
        && std::abs(good.previousResidualMm - 0.6) < 1e-6
        && good.currentResidualMm < 1e-6,
        "moving and resampled new slice compares identical points against the same local planes");
    const auto wrong = surface::compareFreshSurface(fresh, reference,
        correction(0.0, 0.0, -0.6, 0.0), options);
    ok &= expect(wrong.valid && wrong.currentResidualMm > wrong.previousResidualMm + 0.5,
        "same-frame local surfaces expose a genuinely worsening pose");
    for (auto &point : fresh) { point.y_mm += 100.0; }
    ok &= expect(!surface::compareFreshSurface(fresh, reference, truth, options).valid,
        "new slice outside frozen local support cannot extrapolate a plane as proof");
    fresh.clear();
    reference.clear();
    for (int index = 0; index < 60; ++index) {
        reference.push_back({index * 0.1, 0.0, 10.0});
        fresh.push_back({index * 0.1, 0.0, 10.6});
    }
    ok &= expect(!surface::compareFreshSurface(fresh, reference, truth, options).valid,
        "a single frozen line cannot manufacture two-dimensional surface support");
    return ok;
}

/** @brief 复现现场：求解后扫描离开训练网格3mm球，但当前CAD仍支持同一局部面。 */
bool movingCurrentCadSupportsFiniteContinuation()
{
    const auto reference = plane();
    std::vector<Point> newSlice;
    for (int index = -25; index <= 25; ++index) {
        newSlice.push_back({100.0 + index * 0.4, -48.0, 10.0});
    }
    const auto truth = correction(0.0, 0.0, 0.6, 0.0);
    const auto measured = move(newSlice, truth);
    const auto currentVirtual = move(newSlice, truth);
    surface::Options options;
    const auto previous = surface::compareFreshSurface(measured, reference, truth, options);
    const auto supported = surface::compareFreshSurfaceWithCurrentReference(
        measured, currentVirtual, reference, truth, options);
    bool ok = expect(!previous.valid && previous.supportedPointCount == 0,
        "old shared 3mm center balls reproduce the moving-slice evidence failure");
    ok &= expect(supported.valid && supported.supportedRatio > 0.99
        && supported.currentModelSupportedRatio > 0.99 && supported.frozenPatchCount >= 24
        && std::abs(supported.previousResidualMm - 0.6) < 1e-6
        && supported.currentResidualMm < 1e-6,
        "current CAD supports a finite tangential continuation without widening real coverage");

    const auto wrong = correction(0.0, 0.0, -0.6, 0.0);
    const auto wrongModel = move(newSlice, wrong);
    const auto worsening = surface::compareFreshSurfaceWithCurrentReference(
        measured, wrongModel, reference, wrong, options);
    ok &= expect(worsening.valid
        && worsening.currentResidualMm > worsening.previousResidualMm + 0.5,
        "same finite surface correspondence still exposes genuine candidate worsening");
    auto bentCurrentModel = currentVirtual;
    for (auto &point : bentCurrentModel) { point.z_mm += 0.2; }
    const auto unsupportedShape = surface::compareFreshSurfaceWithCurrentReference(
        bentCurrentModel, bentCurrentModel, reference, truth, options);
    ok &= expect(!unsupportedShape.valid
        && unsupportedShape.status == surface::FreshSurfaceComparisonStatus::InsufficientCurrentModelSupport,
        "even perfect real/current-model coincidence cannot extend an inconsistent frozen plane");
    auto tooFar = newSlice;
    for (auto &point : tooFar) { point.y_mm += 25.0; }
    const auto farModel = move(tooFar, truth);
    ok &= expect(!surface::compareFreshSurfaceWithCurrentReference(
        farModel, farModel, reference, truth, options).valid,
        "current CAD cannot extend frozen support beyond the finite tangential budget");
    options.maximumModelContinuationMm = 11.0;
    ok &= expect(!surface::compareFreshSurfaceWithCurrentReference(
        measured, currentVirtual, reference, truth, options).valid,
        "online continuation limit cannot be silently relaxed beyond 10mm");
    return ok;
}

/** @brief 原始 11164 帧候选只差训练覆盖，恢复须保留完整留出和几何约束。 */
int replayCoverageRecovery(const QString &path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) { return 1; }
    const auto data = QJsonDocument::fromJson(file.readAll()).object();
    const auto archived = data.value(QStringLiteral("surface_validation")).toObject();
    const auto readPoints = [](const QJsonValue &value) {
        std::vector<Point> result;
        for (const auto &entry : value.toArray()) {
            const auto xyz = entry.toArray();
            if (xyz.size() != 3) { return std::vector<Point>{}; }
            result.push_back({xyz[0].toDouble(), xyz[1].toDouble(), xyz[2].toDouble()});
        }
        return result;
    };
    surface::Request input;
    input.trainingReal_mm = readPoints(data.value(QStringLiteral("input_real_window_parent_mm")));
    input.trainingVirtual_mm = readPoints(data.value(QStringLiteral("solve_virtual_parent_mm")));
    input.holdoutReal_mm = readPoints(archived.value(QStringLiteral("holdout_real_parent_mm")));
    input.holdoutVirtual_mm = readPoints(archived.value(QStringLiteral("holdout_virtual_parent_mm")));
    const auto matrix = archived.value(QStringLiteral("pcl_candidate_virtual_to_real_row_major")).toArray();
    if (matrix.size() != 16) { return 1; }
    for (int i = 0; i < 16; ++i) { input.candidateVirtualToReal.rowMajor[i] = matrix[i].toDouble(); }
    const auto strict = surface::evaluate(input);
    bool ok = expect(!strict.accepted && strict.hasObservableCorrection,
        "archived candidate must reproduce the original strict rejection");
    input.options.allowTrainingCoverageRecovery = true;
    const auto recovered = surface::evaluate(input);
    ok &= expect(recovered.accepted && recovered.trainingCoverageRecoveryUsed,
        "strongly improving archived candidate must recover from slight training coverage deficit");
    ok &= expect(recovered.holdoutAfter.realCoverageRatio >= 0.7
        && recovered.holdoutAfter.virtualCoverageRatio >= 0.7,
        "recovery cannot lower the heldout coverage threshold");
    auto insufficientGain = input;
    insufficientGain.options.minimumImprovementRatio = 0.9;
    ok &= expect(!surface::evaluate(insufficientGain).accepted,
        "recovery must preserve a stricter configured improvement requirement");
    auto wrongHoldout = input;
    for (auto &point : wrongHoldout.holdoutReal_mm) { point.z_mm += 30.0; }
    ok &= expect(!surface::evaluate(wrongHoldout).accepted,
        "recovery cannot bypass heldout coverage failure");
    input.holdoutReal_mm.clear();
    ok &= expect(!surface::evaluate(input).accepted, "recovery cannot bypass missing holdout");
    std::printf("coverage_recovery: strict=%d recovery=%d rank=%d train_virtual=%.6f\n",
        strict.accepted, recovered.accepted, recovered.observableRank,
        recovered.trainingAfter.virtualCoverageRatio);
    return ok ? 0 : 1;
}

/** @brief 可选现场归档回放；只调用生产比较器，不重新拟合ICP或修改原始记录。 */
int replayArchivedSurfaces(const QString &directory)
{
    const QDir archive(directory);
    const auto files = archive.entryList({QStringLiteral("*.rollback.json")}, QDir::Files);
    if (files.isEmpty()) {
        std::fprintf(stderr, "FAIL: no rollback archives supplied for replay\n");
        return 1;
    }
    const auto readObject = [](const QString &path, QJsonObject *object) {
        QFile file(path);
        if (!file.open(QIODevice::ReadOnly)) { return false; }
        QJsonParseError error;
        const auto document = QJsonDocument::fromJson(file.readAll(), &error);
        if (error.error != QJsonParseError::NoError || !document.isObject()) { return false; }
        *object = document.object();
        return true;
    };
    const auto points = [](const QJsonValue &value) {
        std::vector<Point> result;
        for (const auto item : value.toArray()) {
            const auto xyz = item.toArray();
            if (xyz.size() != 3 || !xyz[0].isDouble()
                || !xyz[1].isDouble() || !xyz[2].isDouble()) { return std::vector<Point>{}; }
            result.push_back({xyz[0].toDouble(), xyz[1].toDouble(), xyz[2].toDouble()});
        }
        return result;
    };
    int sampleCount = 0;
    int oldValidCount = 0;
    int newValidCount = 0;
    int newEligibleCount = 0;
    int focusEligibleCount = 0;
    int preflightPassCount = 0;
    bool ok = true;
    for (const auto &name : files) {
        QJsonObject rollback;
        QJsonObject attempt;
        QString attemptName = name;
        attemptName.chop(QStringLiteral(".rollback.json").size());
        if (!readObject(archive.filePath(name), &rollback)
            || !readObject(archive.filePath(attemptName), &attempt)) {
            std::fprintf(stderr, "FAIL: cannot parse archived surface transaction\n");
            return 1;
        }
        const auto matrix = attempt.value(QStringLiteral("virtual_to_real_row_major")).toArray();
        if (matrix.size() != 16) {
            std::fprintf(stderr, "FAIL: archived correction is missing\n");
            return 1;
        }
        Transform transform;
        for (int index = 0; index < 16; ++index) {
            transform.rowMajor[static_cast<std::size_t>(index)] = matrix[index].toDouble();
        }
        surface::Options options;
        const auto frozenOptions = attempt.value(QStringLiteral("options")).toObject();
        options.coverageDistanceMm = frozenOptions.value(QStringLiteral("coverage_distance_mm")).toDouble(3.0);
        options.minimumBidirectionalCoverage = frozenOptions.value(QStringLiteral("minimum_bidirectional_coverage")).toDouble(0.7);
        options.maximumResidualMm = frozenOptions.value(QStringLiteral("maximum_residual_mm")).toDouble(5.5);
        options.maximumHoldoutWorseningMm = frozenOptions.value(QStringLiteral("maximum_independent_worsening_mm")).toDouble(0.1);
        options.maximumModelContinuationMm = std::min(10.0,
            frozenOptions.value(QStringLiteral("maximum_correspondence_distance_mm")).toDouble(10.0));
        options.maximumModelPlaneDeviationMm = std::max(1e-6,
            std::min(0.05, options.maximumHoldoutWorseningMm * 0.5));
        const auto frozen = points(attempt.value(QStringLiteral("solve_virtual_parent_mm")));
        const auto holdout = attempt.value(QStringLiteral("surface_validation")).toObject();
        auto movedHoldout = points(holdout.value(QStringLiteral("holdout_virtual_parent_mm")));
        const auto &matrixValues = transform.rowMajor;
        for (auto &point : movedHoldout) {
            const Point original = point;
            point = {matrixValues[0] * original.x_mm + matrixValues[1] * original.y_mm
                    + matrixValues[2] * original.z_mm + matrixValues[3],
                matrixValues[4] * original.x_mm + matrixValues[5] * original.y_mm
                    + matrixValues[6] * original.z_mm + matrixValues[7],
                matrixValues[8] * original.x_mm + matrixValues[9] * original.y_mm
                    + matrixValues[10] * original.z_mm + matrixValues[11]};
        }
        const auto preflight = surface::compareFreshSurfaceWithCurrentReference(
            points(holdout.value(QStringLiteral("holdout_real_parent_mm"))),
            movedHoldout, frozen, transform, options);
        preflightPassCount += preflight.valid ? 1 : 0;
        for (const auto value : rollback.value(QStringLiteral("post_write_samples")).toArray()) {
            const auto sample = value.toObject();
            const auto measured = points(sample.value(QStringLiteral("fresh_real_parent_mm")));
            const auto current = points(sample.value(QStringLiteral("fresh_virtual_parent_mm")));
            const auto previous = surface::compareFreshSurface(measured, frozen, transform, options);
            const auto repaired = surface::compareFreshSurfaceWithCurrentReference(
                measured, current, frozen, transform, options);
            const bool absolutePass = sample.value(QStringLiteral("valid")).toBool()
                && sample.value(QStringLiteral("real_to_virtual_mean_mm")).toDouble() <= options.maximumResidualMm
                && sample.value(QStringLiteral("virtual_to_real_mean_mm")).toDouble() <= options.maximumResidualMm
                && sample.value(QStringLiteral("real_coverage_ratio")).toDouble() >= options.minimumBidirectionalCoverage
                && sample.value(QStringLiteral("virtual_coverage_ratio")).toDouble() >= options.minimumBidirectionalCoverage;
            const bool eligible = absolutePass && repaired.valid
                && repaired.currentResidualMm <= options.maximumResidualMm
                && repaired.currentResidualMm <= repaired.previousResidualMm + options.maximumHoldoutWorseningMm;
            ++sampleCount;
            oldValidCount += previous.valid ? 1 : 0;
            newValidCount += repaired.valid ? 1 : 0;
            newEligibleCount += eligible ? 1 : 0;
            if (attempt.value(QStringLiteral("frame_id")).toString() == QStringLiteral("12680")) {
                focusEligibleCount += eligible ? 1 : 0;
                ok = eligible && ok;
            }
            const QJsonObject record{
                {QStringLiteral("attempt_frame"), attempt.value(QStringLiteral("frame_id"))},
                {QStringLiteral("frame"), sample.value(QStringLiteral("frame_id"))},
                {QStringLiteral("preflight_valid"), preflight.valid},
                {QStringLiteral("old_pair_valid"), previous.valid},
                {QStringLiteral("new_pair_valid"), repaired.valid},
                {QStringLiteral("comparison_status"), static_cast<int>(repaired.status)},
                {QStringLiteral("absolute_pass"), absolutePass},
                {QStringLiteral("eligible"), eligible},
                {QStringLiteral("support_count"), repaired.supportedPointCount},
                {QStringLiteral("support_ratio"), repaired.supportedRatio},
                {QStringLiteral("current_model_support_ratio"), repaired.currentModelSupportedRatio},
                {QStringLiteral("before_mm"), repaired.previousResidualMm},
                {QStringLiteral("after_mm"), repaired.currentResidualMm}};
            const auto line = QJsonDocument(record).toJson(QJsonDocument::Compact);
            std::printf("%s\n", line.constData());
        }
    }
    const QJsonObject summary{
        {QStringLiteral("archive_transactions"), files.size()},
        {QStringLiteral("samples"), sampleCount},
        {QStringLiteral("preflight_pass_transactions"), preflightPassCount},
        {QStringLiteral("old_pair_valid"), oldValidCount},
        {QStringLiteral("new_pair_valid"), newValidCount},
        {QStringLiteral("new_eligible"), newEligibleCount},
        {QStringLiteral("frame_12680_eligible"), focusEligibleCount}};
    const auto line = QJsonDocument(summary).toJson(QJsonDocument::Compact);
    std::printf("SUMMARY: %s\n", line.constData());
    return ok && sampleCount > 0 ? 0 : 1;
}

} // namespace

int main(int argc, char **argv)
{
    if (argc == 3 && QString::fromLocal8Bit(argv[1]) == QStringLiteral("--recovery-fixture")) {
        return replayCoverageRecovery(QString::fromLocal8Bit(argv[2]));
    }
    if (argc == 3 && QString::fromLocal8Bit(argv[1]) == QStringLiteral("--replay-dir")) {
        return replayArchivedSurfaces(QString::fromLocal8Bit(argv[2]));
    }
    const bool ok = planeFreezesTangentialAndYaw() && vShapeFreezesAlongSeamAtCenter()
        && fullGeometryAndBump() && heldoutWorseningAndMissingEvidence()
        && freshSurfaceUsesSamePointsAndBoundedPatches()
        && movingCurrentCadSupportsFiniteContinuation();
    if (ok) { std::printf("PASS: surface subspace projection and heldout validation\n"); }
    return ok ? 0 : 1;
}
