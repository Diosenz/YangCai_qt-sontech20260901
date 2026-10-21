/**
 * @file test_mh12_inverse_kinematics.cpp
 * @brief 用 RobotSim 场景正运动学独立核查 TCP 同步逆解，不连接设备。
 * @details 工具定义取自 test31 的 TCP1011；不读取生产逆解中的工具常量。
 */
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>

#include <Eigen/Geometry>

#include "MotomanMh12InverseKinematics.h"

namespace {

constexpr double PI = 3.14159265358979323846;
using JointArray = std::array<double, 6>;

/** @brief 重建 test31 的 TCP1011 相对软件法兰位姿，长度为 mm。 */
Eigen::Isometry3d sceneToolInFlange()
{
    Eigen::Isometry3d tool = Eigen::Isometry3d::Identity();
    tool.translation() = Eigen::Vector3d(-125.8, 4.7, 490.2);
    tool.linear() = (Eigen::AngleAxisd(169.2 * PI / 180.0, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(-145.2 * PI / 180.0, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(-174.8 * PI / 180.0, Eigen::Vector3d::UnitX())).toRotationMatrix();
    return tool;
}

/**
 * @brief 按场景 MDH 链正解；输入为 SET_Q_VEC 的 S/L/U/R/B/T 弧度值。
 * @details 第二轴有 90 度模型补偿，Link6 后还有 100 mm 软件法兰。
 */
Eigen::Isometry3d sceneTcpFromJoints(const JointArray &joints)
{
    const std::array<double, 6> alpha = {0.0, PI / 2.0, 0.0, PI / 2.0, -PI / 2.0, PI / 2.0};
    const std::array<double, 6> length = {0.0, 155.0, 614.0, 200.0, 0.0, 0.0};
    const std::array<double, 6> offset = {0.0, 0.0, 0.0, 640.0, 0.0, 0.0};
    Eigen::Isometry3d flange = Eigen::Isometry3d::Identity();
    for (std::size_t index = 0; index < joints.size(); ++index) {
        const double theta = joints[index] + (index == 1 ? PI / 2.0 : 0.0);
        flange = flange * Eigen::AngleAxisd(alpha[index], Eigen::Vector3d::UnitX())
            * Eigen::Translation3d(length[index], 0.0, 0.0)
            * Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ())
            * Eigen::Translation3d(0.0, 0.0, offset[index]);
    }
    return flange * Eigen::Translation3d(0.0, 0.0, 100.0) * sceneToolInFlange();
}

/** @brief 将场景正解转为控制柜固定轴 Rz*Ry*Rx 的 TCP 输入。 */
i2r_rlc::digital_twin::MotomanMh12TcpPose tcpPose(const Eigen::Isometry3d &transform)
{
    const Eigen::Vector3d zyx = transform.rotation().eulerAngles(2, 1, 0);
    return {transform.translation().x(), transform.translation().y(), transform.translation().z(),
        zyx[2] * 180.0 / PI, zyx[1] * 180.0 / PI, zyx[0] * 180.0 / PI};
}

void expect(bool condition, const char *message, int &failures)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

/** @brief 多姿态覆盖正常同步、分支复位和无效帧后恢复，避免只比较工具常量。 */
void verifySceneTcpClosure(int &failures)
{
    const std::array<JointArray, 6> samples = {{
        {{0.110, -0.553, -0.074, 0.110, -1.131, -0.561}},
        {{0.2, -0.3, 0.1, 0.4, -0.8, 0.6}},
        {{-0.3, 0.1, -0.2, -0.4, -0.9, -0.7}},
        {{0.5, -0.7, 0.25, 0.5, -1.1, 1.2}},
        {{-0.4, -0.5, 0.5, -0.7, -0.6, -1.0}},
        {{0.1, -0.2, 0.0, 0.2, -1.3, 0.9}}
    }};
    i2r_rlc::digital_twin::CMotomanMh12InverseKinematics solver;
    double maximumPositionErrorMm = 0.0;
    double maximumRotationMatrixError = 0.0;
    for (const auto &joints : samples) {
        const Eigen::Isometry3d expectedTcp = sceneTcpFromJoints(joints);
        const auto input = tcpPose(expectedTcp);
        const auto solution = solver.solve(input);
        expect(solution.succeeded, "reachable scene TCP should have an inverse solution", failures);
        if (!solution.succeeded) {
            continue;
        }
        const Eigen::Isometry3d actualTcp = sceneTcpFromJoints(solution.jointAnglesRad);
        const double positionErrorMm = (actualTcp.translation() - expectedTcp.translation()).norm();
        const double rotationError = (actualTcp.rotation() - expectedTcp.rotation()).norm();
        maximumPositionErrorMm = std::max(maximumPositionErrorMm, positionErrorMm);
        maximumRotationMatrixError = std::max(maximumRotationMatrixError, rotationError);
        expect(positionErrorMm < 1.0e-6, "inverse joints should restore scene TCP position", failures);
        expect(rotationError < 1.0e-8, "inverse joints should restore scene TCP orientation", failures);

        auto invalid = input;
        invalid.xMm = std::numeric_limits<double>::quiet_NaN();
        expect(!solver.solve(invalid).succeeded, "invalid TCP should be rejected", failures);
        const auto recovered = solver.solve(input);
        expect(recovered.succeeded, "invalid input should not prevent subsequent synchronization", failures);
    }
    solver.reset();
    const auto resetSolution = solver.solve(tcpPose(sceneTcpFromJoints(samples.front())));
    expect(resetSolution.succeeded, "reset solver should accept the scene TCP", failures);
    std::cout << "scene TCP closure: max position error=" << maximumPositionErrorMm
              << " mm, max rotation matrix error=" << maximumRotationMatrixError << '\n';
}

/** @brief 可选回放六列 mm/deg TCP 文件；仅核查计算闭环，不产生设备命令。 */
void verifyRecordedTcpClosure(const char *filePath, int &failures)
{
    std::ifstream input(filePath);
    expect(input.is_open(), "recorded TCP file should open", failures);
    i2r_rlc::digital_twin::CMotomanMh12InverseKinematics solver;
    std::string line;
    int count = 0;
    double maximumPositionErrorMm = 0.0;
    double maximumRotationMatrixError = 0.0;
    while (std::getline(input, line)) {
        i2r_rlc::digital_twin::MotomanMh12TcpPose sample;
        std::istringstream fields(line);
        const bool parsed = static_cast<bool>(fields >> sample.xMm >> sample.yMm >> sample.zMm
            >> sample.rxDeg >> sample.ryDeg >> sample.rzDeg);
        expect(parsed, "recorded TCP should contain six numeric fields", failures);
        if (!parsed) {
            continue;
        }
        ++count;
        const auto solution = solver.solve(sample);
        expect(solution.succeeded, "recorded TCP should have a scene-compatible inverse solution", failures);
        if (!solution.succeeded) {
            continue;
        }
        const auto actual = sceneTcpFromJoints(solution.jointAnglesRad);
        const Eigen::Vector3d expectedPosition(sample.xMm, sample.yMm, sample.zMm);
        const Eigen::Matrix3d expectedRotation =
            (Eigen::AngleAxisd(sample.rzDeg * PI / 180.0, Eigen::Vector3d::UnitZ())
             * Eigen::AngleAxisd(sample.ryDeg * PI / 180.0, Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(sample.rxDeg * PI / 180.0, Eigen::Vector3d::UnitX())).toRotationMatrix();
        maximumPositionErrorMm = std::max(maximumPositionErrorMm,
            (actual.translation() - expectedPosition).norm());
        maximumRotationMatrixError = std::max(maximumRotationMatrixError,
            (actual.rotation() - expectedRotation).norm());
    }
    expect(count > 0, "recorded TCP replay should not be empty", failures);
    expect(maximumPositionErrorMm < 1.0e-6, "recorded scene TCP position closure should pass", failures);
    expect(maximumRotationMatrixError < 1.0e-8, "recorded scene TCP rotation closure should pass", failures);
    std::cout << "recorded TCP closure: samples=" << count
              << ", max position error=" << maximumPositionErrorMm
              << " mm, max rotation matrix error=" << maximumRotationMatrixError << '\n';
}

} // namespace

int main(int argc, char *argv[])
{
    int failures = 0;
    verifySceneTcpClosure(failures);
    if (argc == 2) {
        verifyRecordedTcpClosure(argv[1], failures);
    }
    return failures == 0 ? 0 : 1;
}
