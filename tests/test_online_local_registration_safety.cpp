/** @file test_online_local_registration_safety.cpp
 * @brief 独立焊点否决、双向匹配覆盖及局部配准归档的反例回归。
 * @details 只使用合成点云与临时文件，不连接机器人、相机或仿真设备。
 */
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>

#include "OnlineLocalRegistrationWorker.h"

namespace {
using Worker = i2r_rlc::ui::COnlineLocalRegistrationWorker;
using Point = i2r_rlc::localization::Point3D_mm;

bool expect(bool condition, const char *message)
{
    if (!condition) { std::fprintf(stderr, "FAIL: %s\n", message); }
    return condition;
}

std::vector<Point> makeSurface(int radius, double offsetX)
{
    std::vector<Point> points;
    // 沿用现有已验证的非对称曲面；对称鞍面在当前 PCL LM 中可能返回单位阵，
    // 无法作为“已求得改善候选”的反例前提。这里不改变任何生产安全门。
    for (int x = 0; x < 3 * radius; ++x) {
        for (int y = 0; y < 2 * radius + 1; ++y) {
            points.push_back({x * 1.3 + offsetX, y * 1.7,
                0.025 * x * x + 0.013 * y * y + 0.01 * x * y});
        }
    }
    return points;
}

Worker::Options options()
{
    Worker::Options value;
    value.sampleIntervalMs = 0;
    value.minimumMatchIntervalMs = 0;
    value.minimumFrames = 1;
    value.maximumFrames = 2;
    value.maximumSolveTimeMs = 2500;
    value.coverageDistanceMm = 0.15;
    return value;
}

Worker::CloudValidationContext validation(const QString &directory, qint64 frameId,
    double independentRealX)
{
    Worker::CloudValidationContext value;
    value.archiveDirectory = directory;
    value.independentObservation.valid = true;
    value.independentObservation.frameId = frameId;
    value.independentObservation.realWeldPointInParent_mm = {independentRealX, 0.0, 0.0};
    value.independentObservation.virtualWeldPointInParent_mm = {0.0, 0.0, 0.0};
    return value;
}

bool testIndependentVetoAndArchive()
{
    QTemporaryDir directory;
    directory.setAutoRemove(false);
    Worker worker(nullptr);
    worker.configure(options());
    const auto result = worker.processClouds(1, 1, 1,
        makeSurface(5, 0.25), makeSurface(5, 0.0),
        validation(directory.path(), 1, 0.0));
    bool ok = expect(result.attempted && !result.accepted
        && result.afterResidualMm < result.beforeResidualMm * 0.95
        && result.afterIndependentErrorMm > result.beforeIndependentErrorMm + 0.1,
        "improved fitted residual must be vetoed when independent weld point gets worse");
    QFile file(result.archivePath);
    ok &= expect(result.archiveSucceeded && file.open(QIODevice::ReadOnly),
        "rejected attempts must still have a committed diagnostic archive");
    const auto record = QJsonDocument::fromJson(file.readAll()).object();
    const auto frames = record.value(QStringLiteral("raw_frames")).toArray();
    ok &= expect(frames.size() == 1
        && frames.first().toObject().value(QStringLiteral("raw_real_parent_mm")).toArray().size() == 165
        && !record.value(QStringLiteral("solve_real_parent_mm")).toArray().isEmpty()
        && record.value(QStringLiteral("diagnostic_real_to_corrected_virtual")).toArray().size() == 165
        && record.value(QStringLiteral("diagnostic_corrected_virtual_to_real")).toArray().size() == 165
        && record.value(QStringLiteral("virtual_to_real_row_major")).toArray().size() == 16
        && !record.value(QStringLiteral("accepted_before_archive")).toBool(),
        "archive must preserve raw frame, actual filtered source, matrix and rejection decision");
    return ok;
}

bool testIndependentImprovementAndMissingObservation()
{
    QTemporaryDir directory;
    directory.setAutoRemove(false);
    Worker worker(nullptr);
    worker.configure(options());
    const auto accepted = worker.processClouds(2, 1, 1,
        makeSurface(5, 0.25), makeSurface(5, 0.0), validation(directory.path(), 1, 0.25));
    bool ok = expect(accepted.accepted && accepted.archiveSucceeded
        && accepted.afterIndependentErrorMm < accepted.beforeIndependentErrorMm,
        "independent improvement plus covered geometry and archive should allow a valid correction");
    for (int fault = 0; fault < 4; ++fault) {
        auto context = validation(directory.path(), 2 + fault, 0.25);
        if (fault == 0) { context.independentObservation.valid = false; }
        if (fault == 1) { context.independentObservation.frameId = 99; }
        if (fault == 2) {
            context.independentObservation.realWeldPointInParent_mm.x_mm =
                std::numeric_limits<double>::quiet_NaN();
        }
        if (fault == 3) { context.archiveDirectory.clear(); }
        const auto rejected = worker.processClouds(2, 1, 2 + fault,
            makeSurface(5, 0.25), makeSurface(5, 0.0), context);
        ok &= expect(!rejected.accepted && !rejected.attempted,
            "missing, stale or nonfinite independent observations and missing archive must reject before solve");
    }
    return ok;
}

bool testPartialSurfaceCoverage()
{
    QTemporaryDir directory;
    directory.setAutoRemove(false);
    Worker worker(nullptr);
    worker.configure(options());
    const auto result = worker.processClouds(3, 1, 1,
        makeSurface(2, 0.25), makeSurface(5, 0.0), validation(directory.path(), 1, 0.25));
    return expect(result.attempted && !result.accepted && result.archiveSucceeded
        && result.afterResidualMm < result.beforeResidualMm
        && result.virtualCoverageRatio < 0.7,
        "a small real patch must not authorize moving an otherwise unsupported virtual surface");
}

bool testArchiveFailureAndSessionBudget()
{
    QTemporaryDir directory;
    directory.setAutoRemove(false);
    Worker worker(nullptr);
    auto configuration = options();
    configuration.maximumArchivedAttemptsPerSession = 1;
    worker.configure(configuration);
    const auto first = worker.processClouds(4, 1, 1,
        makeSurface(5, 0.25), makeSurface(5, 0.0), validation(directory.path(), 1, 0.25));
    worker.reset();
    const auto exhausted = worker.processClouds(4, 2, 2,
        makeSurface(5, 0.25), makeSurface(5, 0.0), validation(directory.path(), 2, 0.25));
    bool ok = expect(first.accepted && !exhausted.accepted && !exhausted.archiveSucceeded,
        "generation change and worker reset must not replenish the same session archive budget");
    const auto fresh = worker.processClouds(5, 1, 1,
        makeSurface(5, 0.25), makeSurface(5, 0.0), validation(directory.path(), 1, 0.25));
    ok &= expect(fresh.accepted && fresh.archiveSucceeded,
        "a new correction session may start its own archive budget");
    const QString blocked = QDir(directory.path()).filePath(QStringLiteral("ordinary_file"));
    QFile file(blocked);
    ok &= expect(file.open(QIODevice::WriteOnly), "archive failure fixture must be writable as a file");
    file.write("occupied");
    file.close();
    const auto failed = worker.processClouds(6, 1, 1,
        makeSurface(5, 0.25), makeSurface(5, 0.0), validation(blocked, 1, 0.25));
    ok &= expect(failed.attempted && !failed.accepted && !failed.archiveSucceeded,
        "archive I/O failure must veto a geometrically valid correction");
    return ok;
}
} // namespace

int main(int argc, char **argv)
{
    QCoreApplication application(argc, argv);
    bool ok = testIndependentVetoAndArchive();
    ok &= testIndependentImprovementAndMissingObservation();
    ok &= testPartialSurfaceCoverage();
    ok &= testArchiveFailureAndSessionBudget();
    return ok ? 0 : 1;
}
