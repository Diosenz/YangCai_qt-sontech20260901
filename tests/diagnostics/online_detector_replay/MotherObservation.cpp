/**
 * @file MotherObservation.cpp
 * @brief 离线原图母材支撑提取；中心外拟合和法向约束，不输出独立焊点。
 * @details 历史方向/分支偏置只来自已确认几何帧；搜索带跟随当前提案，
 *          不读取场景标签或真值。所有门限为可配置离线候选。
 */
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <vector>
#include <QByteArray>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {
struct COptions {
    double halfSpan = 120;
    double excluded = 40;
    double initialHalfHeight = 80;
    double corridor = 12;
    double minColumns = 25;
    double minSpan = 40;
    double contrast = 40;
    double maxWidth = 8;
    double maxRms = 1.5;
    double maxSlopeChange = 0.18;
    double maxAmbiguity = 0.2;
    double minDeterminant = 0.25;
    double maxCorrection = 8;
    double centerRatio = 0.85;
    double centerDeviation = 2.0;
    double geometryTolerance = 3.0;
};

/** @brief 公开全部候选门限并拒绝未知配置，防止拼写错误悄悄使用默认值。 */
QJsonObject options(const QJsonObject &input, COptions &output)
{
    QJsonObject values{{"search_half_width_px", output.halfSpan},
        {"exclusion_half_width_px", output.excluded},
        {"initial_search_half_height_px", output.initialHalfHeight},
        {"corridor_half_width_px", output.corridor}, {"min_support_columns", output.minColumns},
        {"min_support_span_px", output.minSpan}, {"min_contrast", output.contrast},
        {"max_peak_width_px", output.maxWidth}, {"max_fit_rms_px", output.maxRms},
        {"max_slope_change", output.maxSlopeChange}, {"max_ambiguous_ratio", output.maxAmbiguity},
        {"min_normal_determinant", output.minDeterminant}, {"max_correction_px", output.maxCorrection},
        {"center_required_ratio", output.centerRatio}, {"center_max_deviation_px", output.centerDeviation},
        {"candidate_geometry_tolerance_px", output.geometryTolerance}};
    for (auto it = input.begin(); it != input.end(); ++it) {
        if (!values.contains(it.key()) || !it.value().isDouble()
            || !std::isfinite(it.value().toDouble()) || it.value().toDouble() <= 0 || it.value().toDouble() > 1000000) {
            throw std::runtime_error("unknown or invalid mother candidate option");
        }
        values[it.key()] = it.value();
    }
    output.halfSpan = values["search_half_width_px"].toDouble();
    output.excluded = values["exclusion_half_width_px"].toDouble();
    output.initialHalfHeight = values["initial_search_half_height_px"].toDouble();
    output.corridor = values["corridor_half_width_px"].toDouble();
    output.minColumns = values["min_support_columns"].toDouble();
    output.minSpan = values["min_support_span_px"].toDouble();
    output.contrast = values["min_contrast"].toDouble();
    output.maxWidth = values["max_peak_width_px"].toDouble();
    output.maxRms = values["max_fit_rms_px"].toDouble();
    output.maxSlopeChange = values["max_slope_change"].toDouble();
    output.maxAmbiguity = values["max_ambiguous_ratio"].toDouble();
    output.minDeterminant = values["min_normal_determinant"].toDouble();
    output.maxCorrection = values["max_correction_px"].toDouble();
    output.centerRatio = values["center_required_ratio"].toDouble();
    output.centerDeviation = values["center_max_deviation_px"].toDouble();
    output.geometryTolerance = values["candidate_geometry_tolerance_px"].toDouble();
    if (output.excluded >= output.halfSpan || output.maxAmbiguity > 1
        || output.centerRatio > 1 || output.minDeterminant > 1) {
        throw std::runtime_error("inconsistent mother candidate option ranges");
    }
    return values;
}

struct CSamples {
    std::vector<double> peaks;
    int wide = 0;
};

/** @brief 在原灰度列内寻找窄亮带；多峰不按亮度选赢家。 */
CSamples peaks(const cv::Mat &gray, int x, double center, double radius, const COptions &o)
{
    CSamples result;
    const int begin = std::max(0, static_cast<int>(std::floor(center - radius)));
    const int end = std::min(gray.rows - 1, static_cast<int>(std::ceil(center + radius)));
    if (begin >= end || x < 0 || x >= gray.cols) { return result; }
    int background = 255;
    for (int y = begin; y <= end; ++y) { background = std::min(background, static_cast<int>(gray.at<unsigned char>(y, x))); }
    for (int y = begin; y <= end;) {
        if (gray.at<unsigned char>(y, x) < background + o.contrast) { ++y; continue; }
        const int first = y;
        double weighted = 0, total = 0;
        while (y <= end && gray.at<unsigned char>(y, x) >= background + o.contrast) {
            const double weight = gray.at<unsigned char>(y, x) - background;
            weighted += y * weight;
            total += weight;
            ++y;
        }
        if (y - first > o.maxWidth) { ++result.wide; }
        else if (first > begin && y <= end && total > 0) { result.peaks.push_back(weighted / total); }
    }
    return result;
}

struct CSupport {
    bool valid = false;
    double slope = 0, intercept = 0, rms = 0, offset = 0;
    cv::Vec2d normal{0, 0};
    double rhs = 0;
    std::vector<cv::Point2f> points;
    QJsonObject diagnostic;
};

/** @brief 中心外按历史方向关联像素，拟合后再验收残差、跨度和方向。 */
CSupport support(const cv::Mat &gray, const cv::Point2d &proposal,
    int sign, const QJsonObject &history, const COptions &o)
{
    CSupport result;
    int ambiguity = 0, wide = 0, columns = 0;
    const int first = static_cast<int>(std::ceil(proposal.x + (sign < 0 ? -o.halfSpan : o.excluded)));
    const int last = static_cast<int>(std::floor(proposal.x + (sign < 0 ? -o.excluded : o.halfSpan)));
    for (int x = std::max(0, first); x <= std::min(last, gray.cols - 1); ++x) {
        ++columns;
        const double center = proposal.y + (history.isEmpty() ? 0.0
            : history["slope"].toDouble() * (x - proposal.x) + history["vertical_offset"].toDouble());
        const auto sample = peaks(gray, x, center, history.isEmpty() ? o.initialHalfHeight : o.corridor, o);
        wide += sample.wide;
        if (sample.peaks.size() > 1) { ++ambiguity; }
        if (sample.peaks.size() == 1) { result.points.emplace_back(x, sample.peaks.front()); }
    }
    QJsonArray candidatePixels;
    for (const auto &point : result.points) { candidatePixels.append(QJsonArray{point.x, point.y}); }
    result.diagnostic = {{"points", candidatePixels}, {"point_count", static_cast<int>(result.points.size())},
        {"valid", false}, {"ambiguous_columns", ambiguity},
        {"rejected_wide_peaks", wide}, {"search_columns", columns}};
    const auto reject = [&result](const char *reason) {
        result.diagnostic["reason"] = reason;
        return result;
    };
    if (columns > 0 && static_cast<double>(ambiguity) / columns > o.maxAmbiguity) { return reject("competing_narrow_stripes"); }
    if (result.points.size() < o.minColumns) { return reject("insufficient_narrow_pixel_support"); }
    cv::Vec4f line;
    cv::fitLine(result.points, line, cv::DIST_HUBER, 0, 0.01, 0.01);
    if (std::abs(line[0]) < 1e-6) { return reject("vertical_line_not_supported"); }
    result.slope = line[1] / line[0];
    result.intercept = line[3] - result.slope * line[2];
    double square = 0;
    QJsonArray pixels;
    for (const auto &point : result.points) {
        square += std::pow(point.y - result.slope * point.x - result.intercept, 2);
        pixels.append(QJsonArray{point.x, point.y});
    }
    result.rms = std::sqrt(square / result.points.size());
    result.diagnostic["points"] = pixels;
    result.diagnostic["point_count"] = static_cast<int>(result.points.size());
    result.diagnostic["fit_rms_px"] = result.rms;
    result.diagnostic["slope"] = result.slope;
    result.diagnostic["intercept"] = result.intercept;
    if (result.points.back().x - result.points.front().x < o.minSpan) { return reject("support_span_too_short"); }
    if (result.rms > o.maxRms) { return reject("curved_or_inconsistent_side_support"); }
    if ((!history.isEmpty() && std::abs(result.slope - history["slope"].toDouble()) > o.maxSlopeChange)
        || (history.isEmpty() && sign * result.slope >= -0.03)) { return reject("branch_direction_inconsistent"); }
    const double length = std::sqrt(1 + result.slope * result.slope);
    result.normal = cv::Vec2d(-result.slope / length, 1 / length);
    result.offset = history.isEmpty() ? 0.0 : history["target_normal_offset"].toDouble();
    result.rhs = result.intercept / length + result.offset;
    result.valid = true;
    result.diagnostic["valid"] = true;
    result.diagnostic["normal"] = QJsonArray{result.normal[0], result.normal[1]};
    result.diagnostic["target_rhs"] = result.rhs;
    result.diagnostic["reason"] = "historically_associated_outer_pixel_line";
    return result;
}

/**
 * @brief 用当前像素线与历史目标到线偏置计算部分约束。
 * @details 两法向弱相关时只保留一维，中心不可见时外推仍不是独立测量。
 */
QJsonObject observe(const QJsonObject &request)
{
    COptions o;
    const auto effective = options(request["options"].toObject(), o);
    const auto bytes = QByteArray::fromBase64(request["image_png_base64"].toString().toLatin1());
    const std::vector<unsigned char> encoded(bytes.begin(), bytes.end());
    const auto gray = cv::imdecode(encoded, cv::IMREAD_GRAYSCALE);
    if (gray.empty()) { throw std::runtime_error("mother observer requires actual image pixels"); }
    const auto p = request["proposal"].toArray();
    if (p.size() != 2) { throw std::runtime_error("mother proposal requires two pixels"); }
    const cv::Point2d proposal(p[0].toDouble(), p[1].toDouble());
    if (!std::isfinite(proposal.x) || !std::isfinite(proposal.y)
        || proposal.x < 0 || proposal.y < 0 || proposal.x >= gray.cols || proposal.y >= gray.rows) {
        return {{"support_rank", 0}, {"center_supported", false}, {"learned_model", QJsonObject{}},
            {"estimated_pixel", QJsonValue(QJsonValue::Null)}, {"reasons", QJsonArray{"proposal_outside_image"}},
            {"options", effective}};
    }
    const auto history = request["history"].toObject();
    auto left = support(gray, proposal, -1, history["left"].toObject(), o);
    auto right = support(gray, proposal, 1, history["right"].toObject(), o);
    int good = 0, columns = 0;
    if (left.valid && right.valid) {
        for (int x = std::max(0, static_cast<int>(proposal.x - o.excluded));
             x <= std::min(gray.cols - 1, static_cast<int>(proposal.x + o.excluded)); ++x) {
            const auto &side = x <= proposal.x ? left : right;
            const double expected = side.slope * x + side.intercept;
            const auto sample = peaks(gray, x, expected, o.corridor, o);
            ++columns;
            if (sample.peaks.size() == 1 && std::abs(sample.peaks.front() - expected) <= o.centerDeviation) { ++good; }
        }
    }
    const double centerRatio = columns > 0 ? static_cast<double>(good) / columns : 0;
    QJsonObject model;
    const auto candidate = request["candidate_pixel"].toArray();
    const cv::Point2d anchor = candidate.size() == 2
        ? cv::Point2d(candidate[0].toDouble(), candidate[1].toDouble()) : proposal;
    const auto learned = [&anchor](const CSupport &side) {
        return QJsonObject{{"slope", side.slope},
            {"vertical_offset", side.slope * anchor.x + side.intercept - anchor.y},
            {"target_normal_offset", (anchor.y - side.slope * anchor.x - side.intercept)
                / std::sqrt(1 + side.slope * side.slope)}};
    };
    if (left.valid && right.valid) { model = {{"left", learned(left)}, {"right", learned(right)}}; }
    if (history.isEmpty()) {
        // 尚无真实分支偏置，只用于候选初始验证；不把无锚外推变成焊点。
        left.rhs = left.normal.dot(cv::Vec2d(anchor.x, anchor.y));
        right.rhs = right.normal.dot(cv::Vec2d(anchor.x, anchor.y));
        left.diagnostic["target_rhs"] = left.rhs;
        right.diagnostic["target_rhs"] = right.rhs;
    }
    int rank = 0;
    cv::Point2d estimate = proposal;
    QJsonArray reasons;
    const double determinant = left.normal[0] * right.normal[1] - left.normal[1] * right.normal[0];
    if (left.valid && right.valid && std::abs(determinant) >= o.minDeterminant) {
        rank = 2;
        estimate.x = (left.rhs * right.normal[1] - left.normal[1] * right.rhs) / determinant;
        estimate.y = (left.normal[0] * right.rhs - left.rhs * right.normal[0]) / determinant;
    } else if (left.valid || right.valid) {
        rank = 1;
        const auto &side = left.valid ? left : right;
        const double correction = side.rhs - side.normal.dot(cv::Vec2d(proposal.x, proposal.y));
        estimate += cv::Point2d(side.normal[0], side.normal[1]) * correction;
        reasons.append(left.valid && right.valid ? "near_parallel_rank_one_only" : "single_side_normal_only");
    } else { reasons.append("no_associated_mother_support"); }
    if (cv::norm(estimate - proposal) > o.maxCorrection) {
        rank = 0;
        reasons.append("correction_exceeds_candidate_bound");
    }
    if (!std::isfinite(estimate.x) || !std::isfinite(estimate.y)
        || estimate.x < 0 || estimate.y < 0 || estimate.x >= gray.cols || estimate.y >= gray.rows) {
        rank = 0;
        reasons.append("estimate_outside_image");
    }
    return {{"left", left.diagnostic}, {"right", right.diagnostic}, {"proposal", QJsonArray{proposal.x, proposal.y}},
        {"support_rank", rank}, {"normal_determinant", determinant},
        {"estimated_pixel", rank > 0 ? QJsonValue(QJsonArray{estimate.x, estimate.y}) : QJsonValue(QJsonValue::Null)},
        {"center_supported", centerRatio >= o.centerRatio}, {"center_support_ratio", centerRatio},
        {"learned_model", model}, {"reasons", reasons}, {"options", effective},
        {"tangential_component_predicted", rank == 1}, {"independent_measurement", false}};
}
}

int main()
{
    try {
        const std::string input((std::istreambuf_iterator<char>(std::cin)), std::istreambuf_iterator<char>());
        const auto document = QJsonDocument::fromJson(QByteArray::fromStdString(input));
        if (!document.isObject()) { throw std::runtime_error("invalid mother request"); }
        std::cout << QJsonDocument(observe(document.object())).toJson(QJsonDocument::Compact).constData() << '\n';
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 2;
    }
    return 0;
}
