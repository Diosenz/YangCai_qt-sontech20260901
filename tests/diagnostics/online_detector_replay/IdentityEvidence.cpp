/**
 * @file IdentityEvidence.cpp
 * @brief 离线身份审查的原图外观证据；不训练或替换生产 KCF。
 * @details 以已确认帧的双侧纹理为模板，在历史相对运动走廊内验收。
 *          分数仅为外观证据；走廊外高分候选不能取得同目标身份。
 */
#include <cmath>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <QByteArray>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {
/** @brief 输入仅为公开图像字节；不读取场景标签或评估真值。 */
cv::Mat image(const QJsonObject &input, const char *key)
{
    const auto bytes = QByteArray::fromBase64(input.value(key).toString().toLatin1());
    return cv::imdecode(std::vector<unsigned char>(bytes.begin(), bytes.end()), cv::IMREAD_GRAYSCALE);
}

/** @brief 以完整模板落入图像为前提，禁止边界补像伪造支持。 */
bool inside(const cv::Rect &rect, const cv::Mat &image)
{
    return (rect & cv::Rect(0, 0, image.cols, image.rows)) == rect;
}
}

int main()
{
    try {
        const std::string text((std::istreambuf_iterator<char>(std::cin)), std::istreambuf_iterator<char>());
        const auto input = QJsonDocument::fromJson(QByteArray::fromStdString(text)).object();
        const auto historical = image(input, "historical_image");
        const auto current = image(input, "current_image");
        const auto p = input.value("historical_pixel").toArray();
        const auto e = input.value("expected_pixel").toArray();
        if (historical.empty() || current.empty() || p.size() != 2 || e.size() != 2) {
            throw std::runtime_error("identity image or context unavailable");
        }
        constexpr int halfWidth = 48;
        constexpr int halfHeight = 32;
        const cv::Rect patchRect(std::lround(p[0].toDouble()) - halfWidth,
            std::lround(p[1].toDouble()) - halfHeight, 2 * halfWidth + 1, 2 * halfHeight + 1);
        if (!inside(patchRect, historical)) { throw std::runtime_error("historical patch outside image"); }
        const auto patch = historical(patchRect);
        cv::Scalar mean;
        cv::Scalar deviation;
        cv::meanStdDev(patch, mean, deviation);
        if (deviation[0] < 5.0) { throw std::runtime_error("historical patch has insufficient contrast"); }
        cv::Mat scores;
        cv::matchTemplate(current, patch, scores, cv::TM_CCOEFF_NORMED);
        const int radius = input.value("corridor_pixel").toInt();
        const cv::Point expected(std::lround(e[0].toDouble()) - halfWidth,
            std::lround(e[1].toDouble()) - halfHeight);
        cv::Rect search(expected.x - radius, expected.y - radius, 2 * radius + 1, 2 * radius + 1);
        search &= cv::Rect(0, 0, scores.cols, scores.rows);
        if (search.empty()) { throw std::runtime_error("identity corridor outside image"); }
        double best = 0;
        cv::Point location;
        cv::minMaxLoc(scores(search), nullptr, &best, nullptr, &location);
        location += search.tl();
        const cv::Point matched = location + cv::Point(halfWidth, halfHeight);
        auto alternatives = scores(search).clone();
        const auto local = location - search.tl();
        cv::rectangle(alternatives, cv::Rect(local.x - 8, local.y - 8, 17, 17), cv::Scalar(-1), cv::FILLED);
        double second = -1;
        cv::minMaxLoc(alternatives, nullptr, &second);
        QJsonObject result;
        result["matched_pixel"] = QJsonArray{matched.x, matched.y};
        result["historical_corridor_score"] = best;
        result["second_corridor_score"] = second;
        result["patch_stddev"] = deviation[0];
        const auto candidate = input.value("candidate_pixel").toArray();
        if (candidate.size() == 2) {
            const cv::Rect candidateRect(std::lround(candidate[0].toDouble()) - halfWidth,
                std::lround(candidate[1].toDouble()) - halfHeight, patch.cols, patch.rows);
            if (inside(candidateRect, current)) {
                cv::Mat score;
                cv::matchTemplate(current(candidateRect), patch, score, cv::TM_CCOEFF_NORMED);
                result["candidate_score"] = score.at<float>(0, 0);
                // 双侧分别比较，避免只有一个亮峰获得完整母材形状支持。
                for (int side = 0; side < 2; ++side) {
                    const cv::Rect section(side == 0 ? 0 : 65, 0, 32, patch.rows);
                    cv::matchTemplate(current(candidateRect)(section), patch(section), score, cv::TM_CCOEFF_NORMED);
                    result[side == 0 ? "left_branch_score" : "right_branch_score"] = score.at<float>(0, 0);
                }
            }
        }
        std::cout << QJsonDocument(result).toJson(QJsonDocument::Compact).constData() << '\n';
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 2;
    }
    return 0;
}
