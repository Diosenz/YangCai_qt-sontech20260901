/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      MotomanMh12InverseKinematics.cpp
 * @brief     Motoman MH12 真实 TCP 到六轴关节角的解析逆解实现。
 */

#include "MotomanMh12InverseKinematics.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Geometry>

namespace i2r_rlc::digital_twin {
namespace {

using JointArray = std::array<double, 6>;
using SolutionArray = std::array<JointArray, 8>;

constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double MM_TO_M = 0.001;
constexpr double MINIMAL_NUMBER = 1.0e-7;

// 参数来自 SWv1/conf/motoman_mh12.robot，与 SwRobot::inv_kin_6r 的取值一致。
constexpr double LINK2_A_M = 0.155;
constexpr double LINK3_A_M = 0.614;
constexpr double LINK4_A_M = 0.20;
constexpr double LINK4_D_M = 0.64;
constexpr double FLANGE_FRAME_Z_M = 0.1;

constexpr JointArray JOINT_COMPENSATION_RAD = {
    0.0,
    PI / 2.0,
    0.0,
    0.0,
    0.0,
    0.0};
constexpr JointArray JOINT_MIN_RAD = {
    -2.96705973,
    -1.1344640137963142,
    -3.0543261909900767,
    -PI,
    -2.3561944901923448,
    -2.0 * PI};
constexpr JointArray JOINT_MAX_RAD = {
    2.96705973,
    PI,
    4.1887902047863905,
    PI,
    2.3561944901923448,
    2.0 * PI};

// SWv1/CDlgConnectRobotController.cpp 中真实 TCP 到软件法兰的固定工具标定。
constexpr double TOOL_TO_FLANGE_X_M = -0.125136;
constexpr double TOOL_TO_FLANGE_Y_M = 0.004882;
constexpr double TOOL_TO_FLANGE_Z_M = 0.494425;
constexpr double TOOL_TO_FLANGE_RX_RAD = 0.5225 * DEG_TO_RAD;
constexpr double TOOL_TO_FLANGE_RY_RAD = -34.8683 * DEG_TO_RAD;
constexpr double TOOL_TO_FLANGE_RZ_RAD = 0.1727 * DEG_TO_RAD;

enum class SolutionFlag {
    Invalid = -1,
    Regular = 0,
    WristAligned = 1,
    WristOpposed = 2
};

double adjustAngle(double angle)
{
    while (angle > PI) {
        angle -= 2.0 * PI;
    }
    while (angle < -PI) {
        angle += 2.0 * PI;
    }
    return angle;
}

bool finitePose(const MotomanMh12TcpPose &pose)
{
    return std::isfinite(pose.xMm)
        && std::isfinite(pose.yMm)
        && std::isfinite(pose.zMm)
        && std::isfinite(pose.rxDeg)
        && std::isfinite(pose.ryDeg)
        && std::isfinite(pose.rzDeg);
}

Eigen::Isometry3d makePose(
    double xM,
    double yM,
    double zM,
    double rxRad,
    double ryRad,
    double rzRad)
{
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = (
        Eigen::AngleAxisd(rzRad, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(ryRad, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(rxRad, Eigen::Vector3d::UnitX()))
                        .toRotationMatrix();
    pose.translation() = Eigen::Vector3d(xM, yM, zM);
    return pose;
}

bool nonNegativeSquareRoot(double value, double *result)
{
    if (result == nullptr || value < -MINIMAL_NUMBER) {
        return false;
    }
    *result = std::sqrt(std::max(0.0, value));
    return true;
}

double sineCosineError(double angle, double sineValue, double cosineValue)
{
    return std::abs(std::sin(angle) - sineValue)
        + std::abs(std::cos(angle) - cosineValue);
}

double chooseAngle(double sineValue, double cosineValue)
{
    const double first = std::atan2(sineValue, cosineValue);
    const double second = std::atan2(-sineValue, -cosineValue);
    return sineCosineError(first, sineValue, cosineValue)
            <= sineCosineError(second, sineValue, cosineValue)
        ? first
        : second;
}

void calculateAnalyticSolutions(
    const Eigen::Matrix4d &target,
    double currentJoint4Rad,
    SolutionArray *solutions,
    std::array<SolutionFlag, 8> *flags)
{
    if (solutions == nullptr || flags == nullptr) {
        return;
    }

    solutions->fill(JointArray{});
    flags->fill(SolutionFlag::Regular);

    const double targetX = target(0, 3);
    const double targetY = target(1, 3);
    const double targetZ = target(2, 3);
    const double geometryD = LINK4_A_M * LINK4_A_M
        + LINK4_D_M * LINK4_D_M
        - LINK3_A_M * LINK3_A_M;

    (*solutions)[0][0] = std::atan2(targetY, targetX);
    (*solutions)[4][0] = std::atan2(-targetY, -targetX);
    for (std::size_t index = 1; index < 4; ++index) {
        (*solutions)[index][0] = (*solutions)[0][0];
        (*solutions)[index + 4][0] = (*solutions)[4][0];
    }

    for (std::size_t index = 0; index < 8; index += 4) {
        const double theta1 = (*solutions)[index][0];
        const double length = std::cos(theta1) * targetX
            + std::sin(theta1) * targetY
            - LINK2_A_M;
        const double middle = 0.5
            * (length * length + targetZ * targetZ - geometryD)
            / LINK3_A_M;
        double root = 0.0;
        if (!nonNegativeSquareRoot(
                length * length + targetZ * targetZ - middle * middle,
                &root)) {
            for (std::size_t offset = 0; offset < 4; ++offset) {
                (*flags)[index + offset] = SolutionFlag::Invalid;
            }
            continue;
        }

        const double baseAngle = std::atan2(targetZ, length);
        const double branchAngle = std::atan2(root, middle);
        (*solutions)[index][1] = adjustAngle(baseAngle + branchAngle);
        (*solutions)[index + 1][1] = (*solutions)[index][1];
        (*solutions)[index + 2][1] = adjustAngle(baseAngle - branchAngle);
        (*solutions)[index + 3][1] = (*solutions)[index + 2][1];
    }

    for (std::size_t index = 0; index < 8; index += 2) {
        if ((*flags)[index] == SolutionFlag::Invalid) {
            continue;
        }

        const double theta1 = (*solutions)[index][0];
        const double theta2 = (*solutions)[index][1];
        const double length = std::cos(theta1) * targetX
            + std::sin(theta1) * targetY
            - LINK2_A_M;
        const double horizontal = std::cos(theta2) * length
            + std::sin(theta2) * targetZ
            - LINK3_A_M;
        double root = 0.0;
        if (!nonNegativeSquareRoot(
                LINK4_A_M * LINK4_A_M
                    + LINK4_D_M * LINK4_D_M
                    - horizontal * horizontal,
                &root)) {
            (*flags)[index] = SolutionFlag::Invalid;
            (*flags)[index + 1] = SolutionFlag::Invalid;
            continue;
        }

        const double fixedAngle = std::atan2(LINK4_D_M, LINK4_A_M);
        const double branchAngle = std::atan2(root, horizontal);
        (*solutions)[index][2] = adjustAngle(fixedAngle + branchAngle);
        (*solutions)[index + 1][2] = adjustAngle(fixedAngle - branchAngle);

        const double checkValue = std::sin(theta2) * length
            - std::cos(theta2) * targetZ;
        const double firstError = std::abs(
            LINK4_A_M * std::sin((*solutions)[index][2])
            - LINK4_D_M * std::cos((*solutions)[index][2])
            + checkValue);
        const double secondError = std::abs(
            LINK4_A_M * std::sin((*solutions)[index + 1][2])
            - LINK4_D_M * std::cos((*solutions)[index + 1][2])
            + checkValue);
        if (firstError <= MINIMAL_NUMBER) {
            (*solutions)[index + 1][2] = (*solutions)[index][2];
        }
        else if (secondError <= MINIMAL_NUMBER) {
            (*solutions)[index][2] = (*solutions)[index + 1][2];
        }
        else {
            (*flags)[index] = SolutionFlag::Invalid;
            (*flags)[index + 1] = SolutionFlag::Invalid;
        }
    }

    for (std::size_t index = 0; index < 8; index += 2) {
        if ((*flags)[index] == SolutionFlag::Invalid) {
            continue;
        }

        const double theta1 = (*solutions)[index][0];
        const double theta23 = (*solutions)[index][1] + (*solutions)[index][2];
        const double axisProjection = std::cos(theta1) * target(0, 2)
            + std::sin(theta1) * target(1, 2);
        const double sineTheta4Factor = std::sin(theta1) * target(0, 2)
            - std::cos(theta1) * target(1, 2);
        const double cosineTheta4Factor = std::cos(theta23) * axisProjection
            + std::sin(theta23) * target(2, 2);

        if (std::abs(sineTheta4Factor) < MINIMAL_NUMBER
            && std::abs(cosineTheta4Factor) < MINIMAL_NUMBER) {
            const double cosineTheta5 = std::sin(theta23) * axisProjection
                - std::cos(theta23) * target(2, 2);
            if (std::abs(cosineTheta5 - 1.0) < MINIMAL_NUMBER) {
                (*solutions)[index][4] = 0.0;
                (*solutions)[index + 1][4] = 0.0;
                (*flags)[index] = SolutionFlag::WristAligned;
                (*flags)[index + 1] = SolutionFlag::WristAligned;
            }
            else if (std::abs(cosineTheta5 + 1.0) < MINIMAL_NUMBER) {
                (*solutions)[index][4] = PI;
                (*solutions)[index + 1][4] = PI;
                (*flags)[index] = SolutionFlag::WristOpposed;
                (*flags)[index + 1] = SolutionFlag::WristOpposed;
            }
            else {
                (*flags)[index] = SolutionFlag::Invalid;
                (*flags)[index + 1] = SolutionFlag::Invalid;
                continue;
            }

            (*solutions)[index][3] = currentJoint4Rad;
            (*solutions)[index + 1][3] = currentJoint4Rad > 0.0
                ? currentJoint4Rad - PI
                : currentJoint4Rad + PI;
            continue;
        }

        (*solutions)[index][3] = std::atan2(
            sineTheta4Factor,
            cosineTheta4Factor);
        (*solutions)[index + 1][3] = std::atan2(
            -sineTheta4Factor,
            -cosineTheta4Factor);
    }

    for (std::size_t index = 0; index < 8; ++index) {
        if ((*flags)[index] != SolutionFlag::Regular) {
            continue;
        }

        const double theta1 = (*solutions)[index][0];
        const double theta23 = (*solutions)[index][1] + (*solutions)[index][2];
        const double theta4 = (*solutions)[index][3];
        const double projection = std::cos(theta1) * target(0, 2)
            + std::sin(theta1) * target(1, 2);
        const double sineFactor = std::sin(theta1) * target(0, 2)
            - std::cos(theta1) * target(1, 2);
        const double cosineFactor = std::sin(theta23) * projection
            - std::cos(theta23) * target(2, 2);
        const double complementarySineFactor = std::cos(theta23) * projection
            + std::sin(theta23) * target(2, 2);

        double theta5 = 0.0;
        if (std::abs(std::sin(theta4)) < MINIMAL_NUMBER) {
            theta5 = chooseAngle(
                complementarySineFactor / std::cos(theta4),
                cosineFactor);
        }
        else {
            theta5 = chooseAngle(
                sineFactor / std::sin(theta4),
                cosineFactor);
        }
        (*solutions)[index][4] = theta5;
    }

    for (std::size_t index = 0; index < 8; ++index) {
        if ((*flags)[index] == SolutionFlag::Invalid) {
            continue;
        }

        const double theta1 = (*solutions)[index][0];
        const double theta23 = (*solutions)[index][1] + (*solutions)[index][2];
        const double theta4 = (*solutions)[index][3];
        if ((*flags)[index] == SolutionFlag::WristAligned
            || (*flags)[index] == SolutionFlag::WristOpposed) {
            const double sineCombined = std::sin(theta1) * target(0, 0)
                - std::cos(theta1) * target(1, 0);
            const double cosineCombined = std::sin(theta1) * target(0, 1)
                - std::cos(theta1) * target(1, 1);
            const double combined = chooseAngle(sineCombined, cosineCombined);
            (*solutions)[index][5] = adjustAngle(
                (*flags)[index] == SolutionFlag::WristAligned
                    ? combined - theta4
                    : combined + theta4);
            continue;
        }

        const double sineTheta6Factor = std::sin(theta23)
                * (target(0, 1) * std::cos(theta1)
                   + target(1, 1) * std::sin(theta1))
            - std::cos(theta23) * target(2, 1);
        const double cosineTheta6Factor = std::cos(theta23) * target(2, 0)
            - std::sin(theta23)
                * (target(0, 0) * std::cos(theta1)
                   + target(1, 0) * std::sin(theta1));
        (*solutions)[index][5] = chooseAngle(
            sineTheta6Factor / std::sin((*solutions)[index][4]),
            cosineTheta6Factor / std::sin((*solutions)[index][4]));
    }
}

bool withinJointLimits(const JointArray &joints)
{
    for (std::size_t index = 0; index < joints.size(); ++index) {
        if (!std::isfinite(joints[index])
            || joints[index] < JOINT_MIN_RAD[index] - MINIMAL_NUMBER
            || joints[index] > JOINT_MAX_RAD[index] + MINIMAL_NUMBER) {
            return false;
        }
    }
    return true;
}

double jointDistance(const JointArray &lhs, const JointArray &rhs)
{
    double distance = 0.0;
    for (std::size_t index = 0; index < lhs.size(); ++index) {
        distance += std::abs(lhs[index] - rhs[index]);
    }
    return distance;
}

} // namespace

CMotomanMh12InverseKinematics::CMotomanMh12InverseKinematics()
{
    reset();
}

MotomanMh12InverseKinematicsResult CMotomanMh12InverseKinematics::solve(
    const MotomanMh12TcpPose &tcpPose)
{
    MotomanMh12InverseKinematicsResult result;
    if (!finitePose(tcpPose)) {
        result.message = "TCP pose contains a non-finite value";
        return result;
    }

    const Eigen::Isometry3d tcpInRobotBase = makePose(
        tcpPose.xMm * MM_TO_M,
        tcpPose.yMm * MM_TO_M,
        tcpPose.zMm * MM_TO_M,
        tcpPose.rxDeg * DEG_TO_RAD,
        tcpPose.ryDeg * DEG_TO_RAD,
        tcpPose.rzDeg * DEG_TO_RAD);
    const Eigen::Isometry3d toolToFlange = makePose(
        TOOL_TO_FLANGE_X_M,
        TOOL_TO_FLANGE_Y_M,
        TOOL_TO_FLANGE_Z_M,
        TOOL_TO_FLANGE_RX_RAD,
        TOOL_TO_FLANGE_RY_RAD,
        TOOL_TO_FLANGE_RZ_RAD);
    const Eigen::Isometry3d flangeInRobotBase =
        tcpInRobotBase * toolToFlange.inverse();

    Eigen::Isometry3d flangeOffset = Eigen::Isometry3d::Identity();
    flangeOffset.translation().z() = -FLANGE_FRAME_Z_M;
    const Eigen::Matrix4d robotEndTarget =
        (flangeInRobotBase * flangeOffset).matrix();

    SolutionArray solutions{};
    std::array<SolutionFlag, 8> flags{};
    calculateAnalyticSolutions(
        robotEndTarget,
        m_previousInternalJointAnglesRad[3],
        &solutions,
        &flags);

    JointArray selected{};
    double selectedDistance = std::numeric_limits<double>::infinity();
    bool found = false;
    for (std::size_t solutionIndex = 0;
         solutionIndex < solutions.size();
         ++solutionIndex) {
        if (flags[solutionIndex] == SolutionFlag::Invalid) {
            continue;
        }

        for (double joint6Offset : {0.0, -2.0 * PI, 2.0 * PI}) {
            JointArray candidate = solutions[solutionIndex];
            candidate[5] += joint6Offset;
            if (!withinJointLimits(candidate)) {
                continue;
            }

            const double distance = jointDistance(
                candidate,
                m_previousInternalJointAnglesRad);
            if (distance < selectedDistance) {
                selected = candidate;
                selectedDistance = distance;
                found = true;
            }
        }
    }

    if (!found) {
        result.message = "TCP pose has no MH12 solution within joint limits";
        return result;
    }

    m_previousInternalJointAnglesRad = selected;
    for (std::size_t index = 0; index < selected.size(); ++index) {
        result.jointAnglesRad[index] = selected[index]
            - JOINT_COMPENSATION_RAD[index];
    }
    result.succeeded = true;
    return result;
}

void CMotomanMh12InverseKinematics::reset()
{
    m_previousInternalJointAnglesRad = JOINT_COMPENSATION_RAD;
}

} // namespace i2r_rlc::digital_twin
