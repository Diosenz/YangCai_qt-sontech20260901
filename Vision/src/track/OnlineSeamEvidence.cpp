/**
 * @file OnlineSeamEvidence.cpp
 * @brief 将离线证据策略落实到逐曝光生产 C++ 状态机。
 * @details 母材线和身份得分来自当前原图；历史及仿真只提供关联提案。
 */
#include "track/OnlineSeamEvidence.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace i2r_rlc::vision {
namespace {

bool finitePoint(const cv::Point2d &point)
{
    return std::isfinite(point.x) && std::isfinite(point.y);
}

bool inside(const cv::Point2d &point, const cv::Mat &image)
{
    return finitePoint(point) && point.x >= 0 && point.y >= 0
        && point.x < image.cols && point.y < image.rows;
}

cv::Rect identityRect(const cv::Point2d &point)
{
    return {static_cast<int>(std::lround(point.x)) - 48,
        static_cast<int>(std::lround(point.y)) - 32, 97, 65};
}

bool inside(const cv::Rect &rect, const cv::Mat &image)
{
    return (rect & cv::Rect(0, 0, image.cols, image.rows)) == rect;
}

struct CIdentity {
    bool geometry = false;
    bool appearance = false;
    double score = 0.0;
    cv::Point2d matched{-1, -1};
    std::string debug = "patch_or_expected_missing";
};

/** @brief 限制模板匹配到历史走廊，双侧分别验收，避免反光峰凭亮度换目标。 */
CIdentity inspectIdentity(const cv::Mat &gray, const cv::Mat &patch,
    const cv::Point2d &expected, const cv::Point2d &candidate,
    const cv::Point2d &coarse, bool coarseUsable,
    const COnlineSeamEvidenceOptions &options)
{
    CIdentity result;
    result.debug = "patch_empty=" + std::to_string(patch.empty())
        + ";expected_inside=" + std::to_string(inside(expected, gray));
    if (patch.empty() || !inside(expected, gray)) { return result; }
    const auto expectedRect = identityRect(expected);
    cv::Rect search(expectedRect.x - options.identityCorridorPixel,
        expectedRect.y - options.identityCorridorPixel,
        patch.cols + options.identityCorridorPixel * 2,
        patch.rows + options.identityCorridorPixel * 2);
    search &= cv::Rect(0, 0, gray.cols, gray.rows);
    if (search.width < patch.cols || search.height < patch.rows) {
        result.debug = "search_smaller_than_patch";
        return result;
    }
    cv::Mat scores;
    cv::matchTemplate(gray(search), patch, scores, cv::TM_CCOEFF_NORMED);
    cv::Point best;
    cv::minMaxLoc(scores, nullptr, &result.score, nullptr, &best);
    result.matched = cv::Point2d(best + search.tl() + cv::Point(48, 32));
    const auto excluded = cv::Rect(best.x - 8, best.y - 8, 17, 17)
        & cv::Rect(0, 0, scores.cols, scores.rows);
    scores(excluded).setTo(-1.0);
    double second = -1.0;
    cv::minMaxLoc(scores, nullptr, &second);
    const bool unique = second < options.identityMinPatchScore
        || result.score - second >= options.identityMinUniqueness;
    const bool visible = std::isfinite(result.score)
        && result.score >= options.identityMinPatchScore && unique;
    result.debug = "score=" + std::to_string(result.score)
        + ";second=" + std::to_string(second) + ";unique=" + std::to_string(unique)
        + ";visible=" + std::to_string(visible)
        + ";candidate_inside=" + std::to_string(inside(candidate, gray))
        + ";expected_distance=" + std::to_string(cv::norm(candidate - expected))
        + ";matched_distance=" + std::to_string(cv::norm(candidate - result.matched));
    result.appearance = visible && coarseUsable
        && cv::norm(coarse - result.matched) <= options.identityCorridorPixel;
    if (!visible || !inside(candidate, gray)
        || cv::norm(candidate - expected) > options.identityCorridorPixel
        || cv::norm(candidate - result.matched) > 10.0) { return result; }
    const auto rect = identityRect(candidate);
    result.debug += ";candidate_patch_inside=" + std::to_string(inside(rect, gray));
    if (!inside(rect, gray)) { return result; }
    result.geometry = true;
    for (int side = 0; side < 2; ++side) {
        const cv::Rect section(side == 0 ? 0 : 65, 0, 32, patch.rows);
        cv::Mat score;
        cv::matchTemplate(gray(rect)(section), patch(section), score, cv::TM_CCOEFF_NORMED);
        const double value = score.at<float>(0, 0);
        result.debug += (side == 0 ? ";left_patch=" : ";right_patch=") + std::to_string(value);
        result.geometry = result.geometry && std::isfinite(value)
            && value >= options.identityMinBranchScore;
    }
    return result;
}

struct CBranch {
    bool valid = false;
    double slope = 0.0;
    double intercept = 0.0;
    double verticalOffset = 0.0;
    double targetNormalOffset = 0.0;
    cv::Vec2d normal{0, 0};
    double rhs = 0.0;
    std::string debug = "not_fitted";
};

/** @brief 一列多个窄峰保留为歧义，不能只取更亮条纹；边界截断亮带不作支撑。 */
std::vector<double> narrowPeaks(const cv::Mat &gray, int x, double center, double radius)
{
    std::vector<double> output;
    const int begin = std::max(0, static_cast<int>(std::floor(center - radius)));
    const int end = std::min(gray.rows - 1, static_cast<int>(std::ceil(center + radius)));
    if (begin >= end || x < 0 || x >= gray.cols) { return output; }
    int background = 255;
    for (int y = begin; y <= end; ++y) {
        background = std::min(background, static_cast<int>(gray.at<unsigned char>(y, x)));
    }
    for (int y = begin; y <= end;) {
        if (gray.at<unsigned char>(y, x) < background + 40) { ++y; continue; }
        const int first = y;
        double weighted = 0.0, total = 0.0;
        while (y <= end && gray.at<unsigned char>(y, x) >= background + 40) {
            const double weight = gray.at<unsigned char>(y, x) - background;
            weighted += y * weight;
            total += weight;
            ++y;
        }
        if (y - first <= 8 && first > begin && y <= end && total > 0.0) {
            output.push_back(weighted / total);
        }
    }
    return output;
}

/** @brief 避开中心预焊区域，以真实两侧像素拟合；单侧只约束法向。 */
CBranch fitBranch(const cv::Mat &gray, const cv::Point2d &proposal,
    int sign, const CBranch &history)
{
    CBranch result;
    std::vector<cv::Point2f> points;
    int ambiguous = 0, columns = 0;
    const int first = std::max(0, static_cast<int>(std::ceil(proposal.x + (sign < 0 ? -120 : 40))));
    const int last = std::min(gray.cols - 1, static_cast<int>(std::floor(proposal.x + (sign < 0 ? -40 : 120))));
    for (int x = first; x <= last; ++x) {
        ++columns;
        const double center = proposal.y + (history.valid
            ? history.slope * (x - proposal.x) + history.verticalOffset : 0.0);
        const auto peaks = narrowPeaks(gray, x, center, history.valid ? 12.0 : 80.0);
        if (peaks.size() > 1U) { ++ambiguous; }
        if (peaks.size() == 1U) { points.emplace_back(x, peaks.front()); }
    }
    result.debug = "columns=" + std::to_string(columns) + ";ambiguous=" + std::to_string(ambiguous)
        + ";single_peak=" + std::to_string(points.size())
        + ";span=" + std::to_string(points.empty() ? 0.0 : points.back().x - points.front().x);
    if (columns == 0 || static_cast<double>(ambiguous) / columns > 0.2
        || points.size() < 25U || points.back().x - points.front().x < 40.0) { return result; }
    cv::Vec4f line;
    cv::fitLine(points, line, cv::DIST_HUBER, 0, 0.01, 0.01);
    if (std::abs(line[0]) < 1.0e-6) { return result; }
    result.slope = line[1] / line[0];
    result.intercept = line[3] - result.slope * line[2];
    double square = 0.0;
    for (const auto &point : points) {
        square += std::pow(point.y - result.slope * point.x - result.intercept, 2);
    }
    result.debug += ";rms=" + std::to_string(std::sqrt(square / points.size()))
        + ";slope=" + std::to_string(result.slope)
        + ";history_valid=" + std::to_string(history.valid)
        + ";history_slope=" + std::to_string(history.slope);
    if (std::sqrt(square / points.size()) > 1.5
        || (history.valid && std::abs(result.slope - history.slope) > 0.18)
        || (!history.valid && sign * result.slope >= -0.03)) {
        CBranch rejected;
        rejected.debug = result.debug;
        return rejected;
    }
    const double length = std::sqrt(1.0 + result.slope * result.slope);
    result.normal = {-result.slope / length, 1.0 / length};
    result.targetNormalOffset = history.valid ? history.targetNormalOffset : 0.0;
    result.rhs = result.intercept / length + result.targetNormalOffset;
    result.valid = true;
    return result;
}

struct CMother {
    CBranch left;
    CBranch right;
    bool centerSupported = false;
    int rank = 0;
    cv::Point2d estimate{-1, -1};
};

/** @brief 返回外侧母材约束及中心可见性；即使秩为二，遮挡中心仍只是外推估计。 */
CMother observeMother(const cv::Mat &gray, const cv::Point2d &proposal,
    const cv::Point2d &candidate, const CBranch &leftHistory, const CBranch &rightHistory)
{
    CMother result;
    if (!inside(proposal, gray)) { return result; }
    result.left = fitBranch(gray, proposal, -1, leftHistory);
    result.right = fitBranch(gray, proposal, 1, rightHistory);
    auto &left = result.left;
    auto &right = result.right;
    int supported = 0, columns = 0;
    if (left.valid && right.valid) {
        for (int x = std::max(0, static_cast<int>(proposal.x - 40));
             x <= std::min(gray.cols - 1, static_cast<int>(proposal.x + 40)); ++x) {
            const auto &side = x <= proposal.x ? left : right;
            const double expected = side.slope * x + side.intercept;
            const auto peaks = narrowPeaks(gray, x, expected, 12.0);
            ++columns;
            if (peaks.size() == 1U && std::abs(peaks.front() - expected) <= 2.0) { ++supported; }
        }
    }
    result.centerSupported = columns > 0 && static_cast<double>(supported) / columns >= 0.85;
    if (!leftHistory.valid && !rightHistory.valid && inside(candidate, gray)) {
        left.rhs = left.normal.dot(cv::Vec2d(candidate.x, candidate.y));
        right.rhs = right.normal.dot(cv::Vec2d(candidate.x, candidate.y));
    }
    const double determinant = left.normal[0] * right.normal[1] - left.normal[1] * right.normal[0];
    result.estimate = proposal;
    if (left.valid && right.valid && std::abs(determinant) >= 0.25) {
        result.rank = 2;
        result.estimate.x = (left.rhs * right.normal[1] - left.normal[1] * right.rhs) / determinant;
        result.estimate.y = (left.normal[0] * right.rhs - left.rhs * right.normal[0]) / determinant;
    } else if (left.valid || right.valid) {
        result.rank = 1;
        const auto &side = left.valid ? left : right;
        const double correction = side.rhs - side.normal.dot(cv::Vec2d(proposal.x, proposal.y));
        result.estimate += cv::Point2d(side.normal[0], side.normal[1]) * correction;
    }
    if (!inside(result.estimate, gray) || cv::norm(result.estimate - proposal) > 8.0) { result.rank = 0; }
    return result;
}

/** @brief 仅将独立几何与当前母材关系存为下一曝光关联模型。 */
CBranch learnedBranch(CBranch side, const cv::Point2d &anchor)
{
    side.verticalOffset = side.slope * anchor.x + side.intercept - anchor.y;
    side.targetNormalOffset = -side.verticalOffset / std::sqrt(1 + side.slope * side.slope);
    return side;
}

bool validOptions(const COnlineSeamEvidenceOptions &o)
{
    return o.anchorConfirmationFrames >= 1 && o.recoveryConfirmationFrames >= 1
        && o.identityCorridorPixel >= 1 && o.identityCorridorPixel <= 128
        && std::isfinite(o.identityMinPatchScore) && o.identityMinPatchScore >= 0 && o.identityMinPatchScore <= 1
        && std::isfinite(o.identityMinBranchScore) && o.identityMinBranchScore >= 0 && o.identityMinBranchScore <= 1
        && std::isfinite(o.identityMinUniqueness) && o.identityMinUniqueness >= 0 && o.identityMinUniqueness <= 1
        && std::isfinite(o.recoveryMaxStepPixel) && o.recoveryMaxStepPixel > 0
        && std::isfinite(o.maxContinuousStepPixel) && o.maxContinuousStepPixel > 0
        && std::isfinite(o.maxPredictionMs) && o.maxPredictionMs >= 0
        && std::isfinite(o.maxPredictionTravelMm) && o.maxPredictionTravelMm >= 0;
}

} // namespace

struct COnlineSeamEvidence::CState {
    cv::Mat identityPatch;
    cv::Point2d identityPoint{-1, -1};
    cv::Point2d anchor{-1, -1};
    std::int64_t anchorFrameId = -1;
    std::int64_t lastFrameId = -1;
    std::int64_t anchorTimeMs = -1;
    std::int64_t lastTimeMs = -1;
    int pending = 0;
    bool recovery = false;
    bool structuralCandidateActive = false;
    bool hasIdentityDirections = false;
    double identityLeftSlope = 0.0;
    double identityRightSlope = 0.0;
    cv::Point2d recoveryCandidate{-1, -1};
    std::optional<cv::Point2d> lastOutput;
    bool budgetProvenanceValid = false;
    double travelMm = 0.0;
    std::optional<double> lastProgress;
    std::string progressReference;
    bool mappingValid = false;
    bool referenceConflict = false;
    std::uint64_t revision = 0;
    cv::Point2d reference{-1, -1};
    cv::Point2d estimate{-1, -1};
    CBranch left;
    CBranch right;
    // 连续业务输出与独立实测锚分离；估计不能反向训练可靠身份。
    std::optional<cv::Point2d> continuousPoint;
    std::optional<cv::Point2d> continuousReference;
    std::uint64_t continuousRevision = 0;
};

COnlineSeamEvidence::COnlineSeamEvidence() : m_state(std::make_unique<CState>()) {}
COnlineSeamEvidence::~COnlineSeamEvidence() = default;
void COnlineSeamEvidence::reset() { m_state = std::make_unique<CState>(); }
void COnlineSeamEvidence::notifyGap()
{
    m_state->pending = 0;
    m_state->structuralCandidateActive = false;
    m_state->recovery = m_state->anchorFrameId >= 0;
    m_state->lastOutput.reset();
    // 未处理曝光之间的行程不可推算；只有新的可靠几何才能重新授权预算。
    m_state->budgetProvenanceValid = false;
}

COnlineSeamEvidenceResult COnlineSeamEvidence::evaluate(const cv::Mat &gray,
    std::int64_t frameId, std::uint64_t revision, const COnlineSeamEvidenceContext &context,
    const OnlineSeamRefinementResult &geometry, bool geometryProjected,
    const cv::Point2d &coarse, bool coarseUsable, const COnlineSeamEvidenceOptions &options)
{
    COnlineSeamEvidenceResult result;
    result.enabled = true;
    result.debugOptions = options;
    result.debugDetail = "[DEBUG-kcf-20261026] evaluation_not_completed";
    auto &state = *m_state;
    result.anchorFrameId = state.anchorFrameId;
    if (!validOptions(options)) {
        notifyGap();
        result.message = "证据策略参数无效。";
        result.debugDetail = "[DEBUG-kcf-20261026] invalid_options";
        return result;
    }
    result.freshExposure = frameId >= 0 && frameId > state.lastFrameId;
    state.lastFrameId = std::max(state.lastFrameId, frameId);
    if (!result.freshExposure || gray.empty() || gray.type() != CV_8UC1) {
        notifyGap();
        result.message = "重复/乱序曝光或原图缺失，不刷新锚点和模板。";
        result.debugDetail = "[DEBUG-kcf-20261026] fresh=" + std::to_string(result.freshExposure)
            + ";image_empty=" + std::to_string(gray.empty()) + ";image_type=" + std::to_string(gray.type());
        return result;
    }
    const bool hasProgress = context.weldProgressMm.has_value()
        && std::isfinite(*context.weldProgressMm) && !context.progressReference.empty()
        && !context.progressProvenance.empty()
        && context.progressExposureFrameId == frameId;
    const bool hasReference = context.referencePixel.has_value()
        && finitePoint(*context.referencePixel) && context.referenceExposureFrameId == frameId;
    if (state.anchorFrameId >= 0) {
        state.budgetProvenanceValid = state.budgetProvenanceValid
            && context.timestampMs > state.lastTimeMs && hasProgress
            && context.progressReference == state.progressReference && state.lastProgress.has_value();
        if (hasProgress && state.lastProgress.has_value()
            && context.progressReference == state.progressReference) {
            state.travelMm += std::abs(*context.weldProgressMm - *state.lastProgress);
        }
    }
    state.lastTimeMs = context.timestampMs;
    state.lastProgress = context.weldProgressMm;
    result.travelMm = state.travelMm;
    result.elapsedMs = state.anchorTimeMs >= 0 && context.timestampMs >= state.anchorTimeMs
        ? static_cast<double>(context.timestampMs - state.anchorTimeMs) : -1.0;

    cv::Point2d continuousPrediction = state.continuousPoint.value_or(state.identityPoint);
    if (options.enableContinuousOutput && state.continuousPoint.has_value() && hasReference
        && state.continuousReference.has_value() && revision == state.continuousRevision) {
        const cv::Point2d movement = *context.referencePixel - *state.continuousReference;
        // 只使用同版本相邻曝光的相对运动；模型写回和突跳不直接移动业务点。
        if (cv::norm(movement) <= options.identityCorridorPixel) {
            const double distance = cv::norm(movement);
            continuousPrediction += movement * (distance > options.maxContinuousStepPixel
                ? options.maxContinuousStepPixel / distance : 1.0);
        }
    }
    bool aligned = state.mappingValid && hasReference;
    cv::Point2d proposal = state.identityPoint;
    if (aligned) {
        if (revision == state.revision) {
            proposal = state.estimate + *context.referencePixel - state.reference;
        } else if (context.previousReferencePixel.has_value()
            && finitePoint(*context.previousReferencePixel)
            && context.previousReferenceRevision == state.revision
            && context.previousReferenceExposureFrameId == frameId) {
            proposal = state.estimate + *context.previousReferencePixel - state.reference;
        } else { aligned = false; }
    }
    // 走廊约束补偿后的图像残差，不约束扫描总位移。只有跨出身份模板覆盖的
    // 参考跳变才撤销参考续接；它仍不能否决历史走廊中已被原图确认的目标。
    const bool referenceProposalSupported = aligned && inside(proposal, gray)
        && (identityRect(proposal) & identityRect(state.identityPoint)).area() > 0;
    if (aligned) {
        state.estimate = proposal;
        state.reference = *context.referencePixel;
        state.revision = revision;
        if (!referenceProposalSupported) {
            state.referenceConflict = true;
        }
    }
    if (!referenceProposalSupported) { proposal = state.identityPoint; }
    // 独立原图验收只围绕实测锚/曝光参考关联，不能让续接输出反向移动身份走廊。
    const bool rawGeometry = geometry.valid && geometryProjected && inside(geometry.point, gray);
    if (state.identityPatch.empty() && rawGeometry) { proposal = geometry.point; }
    auto identity = inspectIdentity(gray, state.identityPatch, proposal,
        geometry.point, coarse, coarseUsable, options);
    if (rawGeometry && !identity.geometry
        && cv::norm(proposal - state.identityPoint) > 0.0) {
        // CAD 只提供关联提案。新提案没有图像支撑时，保留旧身份的独立验收机会。
        const auto historical = inspectIdentity(gray, state.identityPatch, state.identityPoint,
            geometry.point, coarse, coarseUsable, options);
        if (historical.geometry) {
            identity = historical;
            proposal = state.identityPoint;
            state.referenceConflict = true;
        }
    }
    // 反光可改变整块灰度外观。当前完整双支几何与历史方向一致、仍在锚的关联
    // 走廊且有最低外观支持时，可由两份新曝光重建身份；估计点从不参与确认。
    const bool structuralIdentity = rawGeometry && !identity.geometry
        && state.anchorFrameId >= 0 && state.hasIdentityDirections && geometry.hasBranchDirections
        && identity.score >= options.identityMinBranchScore
        && cv::norm(geometry.point - proposal) <= options.identityCorridorPixel
        && std::abs(geometry.leftSlope - state.identityLeftSlope) <= 0.25
        && std::abs(geometry.rightSlope - state.identityRightSlope) <= 0.25;
    if (structuralIdentity) {
        identity.geometry = true;
        if (!state.structuralCandidateActive) {
            state.pending = 0;
            state.recovery = true;
        }
    }
    state.structuralCandidateActive = structuralIdentity;
    identity.debug += ";structural_identity=" + std::to_string(structuralIdentity);
    result.identityScore = identity.score;
    bool accepted = rawGeometry && (state.identityPatch.empty() || identity.geometry);
    const auto mother = options.enableMotherConstraints
        ? observeMother(gray, proposal, geometry.point, state.left, state.right) : CMother{};
    result.motherSupportRank = mother.rank;
    result.debugDetail = "[DEBUG-kcf-20261026] geometry_valid=" + std::to_string(geometry.valid)
        + ";geometry_status=" + std::to_string(static_cast<int>(geometry.status))
        + ";projected=" + std::to_string(geometryProjected)
        + ";raw_geometry=" + std::to_string(rawGeometry)
        + ";patch_empty=" + std::to_string(state.identityPatch.empty())
        + ";identity_geometry=" + std::to_string(identity.geometry)
        + ";identity{" + identity.debug + "}"
        + ";mother_enabled=" + std::to_string(options.enableMotherConstraints)
        + ";left_valid=" + std::to_string(mother.left.valid) + ";left{" + mother.left.debug + "}"
        + ";right_valid=" + std::to_string(mother.right.valid) + ";right{" + mother.right.debug + "}"
        + ";center_supported=" + std::to_string(mother.centerSupported)
        + ";mother_rank=" + std::to_string(mother.rank)
        + ";mother_distance=" + std::to_string(cv::norm(mother.estimate - geometry.point));
    if (options.enableMotherConstraints && rawGeometry) {
        accepted = accepted && mother.left.valid && mother.right.valid && mother.centerSupported
            && mother.rank == 2 && cv::norm(mother.estimate - geometry.point) <= 3.0;
    }
    if (!accepted) {
        state.recovery = state.anchorFrameId >= 0;
        state.pending = 0;
    } else {
        const bool trackingEstablishedIdentity = state.anchorFrameId >= 0 && !state.recovery;
        const bool consistent = state.pending == 0
            || cv::norm(geometry.point - state.recoveryCandidate) <= options.identityCorridorPixel;
        // 首次建锚/失测恢复需要多曝光；稳定跟踪已有当前图像身份支持，不能因
        // 绝对位移再次降为第一确认帧，继而把本可更新的锚点冻结在旧位置。
        state.pending = trackingEstablishedIdentity ? options.anchorConfirmationFrames
            : (consistent ? state.pending + 1 : 1);
        state.recoveryCandidate = geometry.point;
        // 第一份几何只建立待确认身份；不会训练 KCF 或建立可靠锚点。
        if (state.identityPatch.empty()) {
            const auto rect = identityRect(geometry.point);
            if (inside(rect, gray)) {
                cv::Scalar mean, deviation;
                cv::meanStdDev(gray(rect), mean, deviation);
                result.debugDetail += ";seed_patch_stddev=" + std::to_string(deviation[0]);
                if (deviation[0] >= 5.0) {
                    state.identityPatch = gray(rect).clone();
                    state.identityPoint = geometry.point;
                }
            }
        }
    }
    const int required = state.recovery ? options.recoveryConfirmationFrames : options.anchorConfirmationFrames;
    result.confirmationCount = state.pending;
    const bool continuation = state.anchorFrameId >= 0 && state.budgetProvenanceValid
        && result.elapsedMs >= 0 && options.maxPredictionMs > 0 && options.maxPredictionTravelMm > 0
        && result.elapsedMs < options.maxPredictionMs && state.travelMm < options.maxPredictionTravelMm
        && aligned && !state.referenceConflict && inside(state.estimate, gray);
    bool transition = false;
    if (accepted && state.pending >= required && !state.identityPatch.empty()) {
        if (!options.enableContinuousOutput && state.recovery && state.lastOutput.has_value()
            && cv::norm(geometry.point - *state.lastOutput) > options.recoveryMaxStepPixel) {
            result.debugDetail += ";recovery_step=" + std::to_string(cv::norm(geometry.point - *state.lastOutput));
            if (continuation) { transition = true; }
            else { accepted = false; }
        } else {
            const auto rect = identityRect(geometry.point);
            if (!inside(rect, gray)) { accepted = false; }
            else {
                state.identityPatch = gray(rect).clone();
                state.identityPoint = state.anchor = geometry.point;
                state.anchorFrameId = frameId;
                state.anchorTimeMs = context.timestampMs;
                state.travelMm = 0.0;
                state.budgetProvenanceValid = context.timestampMs >= 0 && hasProgress;
                state.progressReference = context.progressReference;
                state.mappingValid = hasReference;
                state.referenceConflict = false;
                if (hasReference) {
                    state.reference = *context.referencePixel;
                    state.estimate = geometry.point;
                    state.revision = revision;
                }
                state.recovery = false;
                state.pending = required;
                state.left = learnedBranch(mother.left, geometry.point);
                state.right = learnedBranch(mother.right, geometry.point);
                state.hasIdentityDirections = geometry.hasBranchDirections;
                state.identityLeftSlope = geometry.leftSlope;
                state.identityRightSlope = geometry.rightSlope;
                result.pixelAvailable = result.independentGeometry = result.controlEligible = true;
                result.anchorRefreshed = true;
                result.anchorFrameId = frameId;
                result.elapsedMs = result.travelMm = 0.0;
                result.point = geometry.point;
                result.source = OnlineSeamPointSource::Geometry;
                result.message = "原图几何及目标身份已确认，实测 KCF 可进入控制参考。";
            }
        }
    }
    if (!result.anchorRefreshed) {
        result.message = accepted ? "同目标新曝光确认中。" : "原图几何或母材身份未通过。";
        if (continuation) {
            result.pixelAvailable = true;
            result.point = state.estimate;
            result.source = OnlineSeamPointSource::SimulationPrediction;
            // 恢复候选待确认时不能让外观或母材分支掩盖确认状态。
            if (!accepted && identity.appearance) {
                result.point = identity.matched;
                result.source = OnlineSeamPointSource::CorrectedKcf;
            }
            if (!accepted && options.enableMotherConstraints && mother.rank > 0
                && (state.left.valid || state.right.valid)) {
                result.point = mother.estimate;
                result.source = OnlineSeamPointSource::GuidedEstimate;
            }
            if (transition) {
                const auto delta = geometry.point - *state.lastOutput;
                result.point = *state.lastOutput + delta * (options.recoveryMaxStepPixel / cv::norm(delta));
                result.source = OnlineSeamPointSource::RecoveryTransition;
            }
            result.message += " 有限续接仅供像素显示，不具备独立校验或控制资格。";
        } else {
            result.message += " 续接预算/曝光行程/参考关联缺失或已到限，不输出估计点。";
        }
    }
    if (options.enableContinuousOutput && state.anchorFrameId >= 0) {
        // 原图暂态失测仍输出同一目标。外观只能小幅修正续接，不能跳到反光峰。
        if (!result.pixelAvailable) {
            result.point = continuousPrediction;
            result.source = OnlineSeamPointSource::SimulationPrediction;
            if (identity.appearance && inside(identity.matched, gray)
                && cv::norm(identity.matched - continuousPrediction) <= options.maxContinuousStepPixel) {
                result.point += (identity.matched - continuousPrediction) * 0.25;
                result.source = OnlineSeamPointSource::CorrectedKcf;
            }
            result.pixelAvailable = inside(result.point, gray);
            result.message = "预焊/反光期间连续 KCF 续接，保留估计来源，用于跟随。";
        }
        if (result.pixelAvailable && state.continuousPoint.has_value()) {
            // 检测恢复也受连续步长限制，不能一帧跳回新折点；原始几何另行保留。
            const auto delta = result.point - *state.continuousPoint;
            const double distance = cv::norm(delta);
            if (distance > options.maxContinuousStepPixel) {
                result.point = *state.continuousPoint + delta * (options.maxContinuousStepPixel / distance);
                result.source = OnlineSeamPointSource::RecoveryTransition;
                result.independentGeometry = false;
                result.message = "KCF 恢复平滑过渡，用于跟随，不作为独立实测。";
            }
        }
        result.controlEligible = result.pixelAvailable;
        if (result.pixelAvailable) { state.continuousPoint = result.point; }
        state.continuousReference = hasReference ? context.referencePixel : std::nullopt;
        state.continuousRevision = revision;
    }
    state.lastOutput = result.pixelAvailable ? std::optional<cv::Point2d>(result.point) : std::nullopt;
    result.debugDetail += ";accepted=" + std::to_string(accepted)
        + ";confirmation=" + std::to_string(result.confirmationCount)
        + ";required=" + std::to_string(required)
        + ";has_progress=" + std::to_string(hasProgress) + ";has_reference=" + std::to_string(hasReference)
        + ";aligned=" + std::to_string(aligned) + ";reference_conflict=" + std::to_string(state.referenceConflict)
        + ";continuation=" + std::to_string(continuation)
        + ";control_eligible=" + std::to_string(result.controlEligible);
    return result;
}

} // namespace i2r_rlc::vision
