/**
 * @file SurfaceReplay.cpp
 * @brief 重算归档云的表面验收与新帧后验；不重新求解 PCL、不访问 RobotSim。
 */
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <stdexcept>

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include "LocalizationSurfaceValidation.h"
#include "OnlineLocalSurfaceVerification.h"

namespace {
namespace loc = i2r_rlc::localization;
namespace surface = loc::surface_validation;
using Check = i2r_rlc::ui::COnlineLocalSurfaceVerification;

/** @brief 读取已保存的 JSON 原始证据，不接受解析失败的空对象。 */
QJsonObject readObject(const char *path)
{
    QFile file(QString::fromUtf8(path));
    if (!file.open(QIODevice::ReadOnly)) { throw std::runtime_error("missing evidence"); }
    QJsonParseError error;
    const auto document = QJsonDocument::fromJson(file.readAll(), &error);
    if (error.error != QJsonParseError::NoError || !document.isObject()) {
        throw std::runtime_error("invalid evidence JSON");
    }
    return document.object();
}

/** @brief 按归档的毫米坐标读取点云，保持顺序与密度。 */
std::vector<loc::Point3D_mm> cloud(const QJsonValue &value)
{
    std::vector<loc::Point3D_mm> result;
    for (const auto &entry : value.toArray()) {
        const auto point = entry.toArray();
        if (point.size() != 3) { throw std::runtime_error("invalid point"); }
        result.push_back({point[0].toDouble(), point[1].toDouble(), point[2].toDouble()});
    }
    return result;
}

/** @brief 将重算数值与历史记录逐项对照，防止错误回放入口产生假结论。 */
void match(double actual, double recorded)
{
    if (!std::isfinite(actual) || std::abs(actual - recorded) > 1.0e-7) {
        throw std::runtime_error("recomputed metric differs from archived runtime");
    }
}

/** @brief 保持生产后验检查器的双向残差及覆盖率字段顺序。 */
Check::Metrics metrics(const surface::Metrics &value)
{
    return {value.realToVirtualMeanMm, value.virtualToRealMeanMm,
            value.realCoverageRatio, value.virtualCoverageRatio};
}
} // namespace

/** @brief 回放原始候选和四份新帧；几何未确认返回 1，证据不一致返回 2。 */
int main(int argc, char **argv)
{
    if (argc < 3 || argc > 5) { return 2; }
    try {
        const auto input = readObject(argv[1]);
        const auto retained = readObject(argv[2]);
        const auto archived = input["surface_validation"].toObject();
        const auto options = input["options"].toObject();
        surface::Request request;
        request.trainingReal_mm = cloud(input["input_real_window_parent_mm"]);
        request.trainingVirtual_mm = cloud(input["solve_virtual_parent_mm"]);
        request.holdoutReal_mm = cloud(archived["holdout_real_parent_mm"]);
        request.holdoutVirtual_mm = cloud(archived["holdout_virtual_parent_mm"]);
        request.options.coverageDistanceMm = options["coverage_distance_mm"].toDouble();
        request.options.minimumBidirectionalCoverage = options["minimum_bidirectional_coverage"].toDouble();
        request.options.maximumResidualMm = options["maximum_residual_mm"].toDouble();
        request.options.minimumImprovementRatio = options["minimum_improvement_ratio"].toDouble();
        request.options.maximumHoldoutWorseningMm = options["maximum_independent_worsening_mm"].toDouble();
        const bool probe = argc >= 4;
        if (probe) {
            request.options.minimumBidirectionalCoverage = std::strtod(argv[3], nullptr);
        }
        const auto candidate = archived["pcl_candidate_virtual_to_real_row_major"].toArray();
        if (candidate.size() != 16) { return 2; }
        for (int index = 0; index < 16; ++index) {
            request.candidateVirtualToReal.rowMajor[static_cast<std::size_t>(index)] = candidate[index].toDouble();
        }
        const auto evaluated = surface::evaluate(request);
        if (!evaluated.accepted || evaluated.observableRank != archived["observable_rank"].toInt()) {
            throw std::runtime_error("write-before verdict differs from archive");
        }
        match(evaluated.trainingAfter.robustSymmetricResidualMm, input["after_residual_mm"].toDouble());
        std::fprintf(stdout, "PREWRITE accepted=%d rank=%d/4\n",
            evaluated.accepted ? 1 : 0, evaluated.observableRank);
        Check::Options checkOptions;
        checkOptions.maximumResidualMm = request.options.maximumResidualMm;
        checkOptions.minimumBidirectionalCoverage = request.options.minimumBidirectionalCoverage;
        checkOptions.maximumWorseningMm = request.options.maximumHoldoutWorseningMm;
        Check check(metrics(evaluated.holdoutAfter), checkOptions);
        const auto samples = retained["post_write_samples"].toArray();
        const bool minimized = argc == 5;
        for (int index = minimized ? 1 : 0; index < samples.size(); ++index) {
            const auto entry = samples[index];
            const auto sample = entry.toObject();
            const auto real = cloud(sample["fresh_real_parent_mm"]);
            const auto reference = cloud(sample["fresh_virtual_parent_mm"]);
            const auto measured = surface::measure(real, reference, request.options);
            const auto paired = surface::compareFreshSurfaceWithCurrentReference(real, reference,
                request.trainingVirtual_mm, evaluated.observableVirtualToReal, request.options);
            match(measured.realCoverageRatio, sample["real_coverage_ratio"].toDouble());
            match(measured.virtualCoverageRatio, sample["virtual_coverage_ratio"].toDouble());
            match(measured.realToVirtualMeanMm, sample["real_to_virtual_mean_mm"].toDouble());
            match(paired.supportedRatio, sample["paired_support_ratio"].toDouble());
            match(paired.currentResidualMm, sample["paired_current_residual_mm"].toDouble());
            const auto frame = sample["frame_id"].toString().toLongLong();
            check.observe(frame, measured.valid, metrics(measured),
                {paired.valid, paired.previousResidualMm, paired.currentResidualMm, paired.supportedRatio});
            if (!probe && static_cast<int>(check.lastObservationStatus()) != sample["observation_status"].toInt()) {
                throw std::runtime_error("post-write observation status differs from archive");
            }
            std::fprintf(stdout, "POST frame=%lld real_coverage=%.6g paired_support=%.6g paired_before=%.6g paired_after=%.6g status=%d decision=%d\n",
                static_cast<long long>(frame), measured.realCoverageRatio, paired.supportedRatio,
                paired.previousResidualMm, paired.currentResidualMm,
                static_cast<int>(check.lastObservationStatus()), static_cast<int>(check.decision()));
        }
        const bool verified = check.decision() == Check::Decision::Verified;
        std::fprintf(stdout, "%s written_pose_geometry_verified=%d archived_retained=%d coverage_limit=%.6g numerical_metrics_match_archive=1\n",
            verified ? "GREEN" : "RED", verified ? 1 : 0,
            retained["retained_without_geometric_verification"].toBool() ? 1 : 0,
            request.options.minimumBidirectionalCoverage);
        return verified ? 0 : 1;
    } catch (const std::exception &error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 2;
    }
}
