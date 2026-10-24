/**
 * @file ControlReplay.cpp
 * @brief 离线调用真实跟随器，输出局部参考与预计纠偏；不连接设备。
 * @details 输入已验收的 PositionerTcp 点，仅在本适配器加入一次工装偏移。
 */
#include <cmath>
#include <cstdio>

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include "OnlineCorrectionPathFollower.h"

namespace {
using namespace i2r_rlc;

control::OnlineCorrectionPoint pointOf(const QJsonValue &value)
{
    const auto a = value.toArray();
    return {a[0].toDouble(), a[1].toDouble(), a[2].toDouble(), true};
}

QJsonArray jsonPoint(const control::OnlineCorrectionPoint &point)
{
    return {point.xMm, point.yMm, point.zMm};
}

/** @brief 输入点为工装TCP系；末节偏移只在此处加入一次。 */
control::OnlineCorrectionPoint lastLinkPoint(
    const QJsonValue &value, const vision::PositionerCalibration &calibration)
{
    auto point = pointOf(value);
    point.xMm += calibration.toolTcpInLastLinkMm.x();
    point.yMm += calibration.toolTcpInLastLinkMm.y();
    point.zMm += calibration.toolTcpInLastLinkMm.z();
    return point;
}

/** @brief 独立跟随器实例保持对照公平，输出为首周期本地P，不发送运动。 */
QJsonObject follow(const QJsonArray &points, const QJsonObject &context)
{
    const auto offset = context["tool_tcp_in_last_link_mm"].toArray();
    const auto translation = context["station_base_translation_mm"].toArray();
    vision::PositionerCalibration calibration;
    calibration.valid = context["calibration_valid"].toBool();
    calibration.toolTcpInLastLinkMm = Eigen::Vector3d(
        offset[0].toDouble(), offset[1].toDouble(), offset[2].toDouble());
    calibration.stationBaseInRobotBase.translation() = Eigen::Vector3d(
        translation[0].toDouble(), translation[1].toDouble(), translation[2].toDouble());
    const auto pose = context["robot_pose_mm_deg"].toArray();
    const auto axes = context["axes_deg"].toArray();
    robot_comm::RobotState robot;
    robot.curx_r = std::llround(pose[0].toDouble() * 1000.0);
    robot.cury_r = std::llround(pose[1].toDouble() * 1000.0);
    robot.curz_r = std::llround(pose[2].toDouble() * 1000.0);
    robot.curRx_r = std::llround(pose[3].toDouble() * 10000.0);
    robot.curRy_r = std::llround(pose[4].toDouble() * 10000.0);
    robot.curRz_r = std::llround(pose[5].toDouble() * 10000.0);
    robot.curS1 = std::llround(axes[0].toDouble() * 10000.0);
    robot.curS2 = std::llround(axes[1].toDouble() * 10000.0);
    std::vector<control::OnlineCorrectionPoint> teach;
    for (const auto &point : context["teach_last_link_mm"].toArray())
    {
        teach.push_back(pointOf(point));
    }
    control::COnlineCorrectionPathFollower follower;
    control::OnlineCorrectionOptions options;
    const auto result = follower.updateReference(robot, options, calibration, teach,
        {lastLinkPoint(points.first(), calibration),
         lastLinkPoint(points.last(), calibration), std::nullopt});
    return {{"valid", result.valid}, {"message", QString::fromStdString(result.message)},
        {"first_last_link_mm", jsonPoint(result.diagnostic.first)},
        {"direction_last_link_mm", jsonPoint(result.diagnostic.directionPoint)},
        {"current_last_link_mm", jsonPoint(result.diagnostic.current)},
        {"foot_last_link_mm", jsonPoint(result.diagnostic.foot)},
        {"future_foot_last_link_mm", jsonPoint(result.diagnostic.futureFoot)},
        {"delta_um", QJsonArray{static_cast<double>(result.command.dxUm),
            static_cast<double>(result.command.dyUm), static_cast<double>(result.command.dzUm)}}};
}
} // namespace

/** @brief 一次局部控制快照；不会缓存或发送P命令。 */
int main(int argc, char **argv)
{
    if (argc != 2)
    {
        return 2;
    }
    QFile file(QString::fromLocal8Bit(argv[1]));
    if (!file.open(QIODevice::ReadOnly))
    {
        return 2;
    }
    const auto input = QJsonDocument::fromJson(file.readAll()).object();
    QJsonObject output{{"follow", follow(input["released"].toArray(), input["context"].toObject())}};
    const auto model = input["context"].toObject()["model_positioner_tcp_mm"].toArray();
    if (model.size() >= 2)
    {
        output["model_counterfactual"] = follow(model, input["context"].toObject());
    }
    const auto bytes = QJsonDocument(output).toJson(QJsonDocument::Compact);
    std::fwrite(bytes.constData(), 1, static_cast<std::size_t>(bytes.size()), stdout);
    return 0;
}
