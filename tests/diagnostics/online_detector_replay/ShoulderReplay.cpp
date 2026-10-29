/**
 * @file ShoulderReplay.cpp
 * @brief 重演已归档的生产肩部匹配调用，区分候选选择与上游 MKCF 响应。
 * @details 只用于完整参数可恢复的单次调用；不补造缺失 MKCF 历史或授予控制资格。
 */
#include <cstdio>

#include <QCoreApplication>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <opencv2/imgcodecs.hpp>

// 匿名辅助函数与生产源保持同一编译单元，避免复制一份匹配算法造成诊断失真。
#include "../../../Vision/src/track/LegacyOnlineKcfDetector.cpp"

int main(int argc, char **argv)
{
    QCoreApplication application(argc, argv);
    if (argc != 2) { return 2; }
    QFile file(QString::fromLocal8Bit(argv[1]));
    if (!file.open(QIODevice::ReadOnly)) { return 2; }
    const auto input = QJsonDocument::fromJson(file.readAll()).object();
    const auto identityImage = cv::imread(input["identity_image"].toString().toStdString(), cv::IMREAD_GRAYSCALE);
    const auto current = cv::imread(input["current_image"].toString().toStdString(), cv::IMREAD_GRAYSCALE);
    if (identityImage.empty() || current.empty()) { return 2; }
    const auto point = [&input](const char *key) {
        const auto value = input[key].toArray();
        return cv::Point2d(value[0].toDouble(), value[1].toDouble());
    };
    namespace v = i2r_rlc::vision;
    const auto identity = v::imageIdentityPatch(identityImage, point("identity_point"));
    const auto prediction = point("prediction");
    const auto response = point("response_point");
    const bool appearance = v::completeImageAppearance(current, response, identity);
    const auto center = appearance ? response : prediction;
    auto match = v::matchImageShoulders(current, center, prediction, identity, false);
    if (!match.valid || !v::completeImageAppearance(current, match.point, identity)) {
        match = v::matchImageShoulders(current, center, prediction, identity, true);
    }
    const QJsonObject result{{"valid", match.valid}, {"point", QJsonArray{match.point.x, match.point.y}},
        {"quality", match.quality}, {"center_quality", match.centerQuality},
        {"response_complete", appearance}, {"center", QJsonArray{center.x, center.y}},
        {"response_identity_supported", v::imageIdentityStripeSupported(current, match.point, identity)},
        {"distance_from_response", cv::norm(match.point - response)}};
    const auto encoded = QJsonDocument(result).toJson(QJsonDocument::Compact);
    std::fwrite(encoded.constData(), 1, static_cast<std::size_t>(encoded.size()), stdout);
    return 0;
}
