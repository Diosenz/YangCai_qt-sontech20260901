/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_vision_penzui_station_projection.cpp
 * @brief       vision penzui station projection 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "geometry/PositionerKinematics.h"
#include "geometry/WeldImageToSpaceProjector.h"
#include "geometry/WeldProjectionContextBuilder.h"

namespace {

namespace fs = std::filesystem;
constexpr double kPi = 3.14159265358979323846;

double rad(double degree)
{
    return degree * kPi / 180.0;
}

std::map<std::string, double> readSensorValues(const fs::path &path)
{
    std::map<std::string, double> values;
    std::ifstream input(path);
    std::string line;
    while (std::getline(input, line)) {
        std::istringstream stream(line);
        std::string key;
        double value = 0.0;
        if (stream >> key >> value) {
            values[key] = value;
        }
    }
    return values;
}

i2r_rlc::vision::WeldSensorCalibration makeSensor(
    const std::map<std::string, double> &v)
{
    i2r_rlc::vision::WeldSensorCalibration sensor;
    sensor.focalLengthM = v.at("f/m");
    sensor.kappa = v.at("Kappa");
    sensor.sxM = v.at("sx/m");
    sensor.syM = v.at("sy/m");
    sensor.cxPixel = v.at("cx");
    sensor.cyPixel = v.at("cy");
    sensor.laserPlaneA = v.at("a");
    sensor.laserPlaneB = v.at("b");
    sensor.laserPlaneC = v.at("c");
    sensor.cameraXInToolM = v.at("pose_CinT[0]/m");
    sensor.cameraYInToolM = v.at("pose_CinT[1]/m");
    sensor.cameraZInToolM = v.at("pose_CinT[2]/m");
    sensor.cameraRxInToolDeg = v.at("pose_CinT[3]/deg");
    sensor.cameraRyInToolDeg = v.at("pose_CinT[4]/deg");
    sensor.cameraRzInToolDeg = v.at("pose_CinT[5]/deg");
    sensor.valid = true;
    return sensor;
}

std::vector<Eigen::Vector2d> readImagePoints(const fs::path &path)
{
    std::vector<Eigen::Vector2d> points;
    std::ifstream input(path);
    double x = 0.0;
    double y = 0.0;
    while (input >> x >> y) {
        points.emplace_back(x, y);
    }
    return points;
}

std::vector<i2r_rlc::robot_comm::RobotState> readRobotStates(const fs::path &path)
{
    std::vector<i2r_rlc::robot_comm::RobotState> states;
    std::ifstream input(path);
    int index = 0;
    double x, y, z, rx, ry, rz, s1, s2;
    while (input >> index >> x >> y >> z >> rx >> ry >> rz >> s1 >> s2) {
        i2r_rlc::robot_comm::RobotState state;
        state.curx_r = std::lround(x * 1000.0);
        state.cury_r = std::lround(y * 1000.0);
        state.curz_r = std::lround(z * 1000.0);
        state.curRx_r = std::lround(rx * 10000.0);
        state.curRy_r = std::lround(ry * 10000.0);
        state.curRz_r = std::lround(rz * 10000.0);
        state.curS1 = std::lround(s1 * 10000.0);
        state.curS2 = std::lround(s2 * 10000.0);
        states.push_back(state);
    }
    return states;
}

Eigen::Vector3d independentLegacyProjection(
    const Eigen::Vector2d &pixel,
    const i2r_rlc::vision::WeldSensorCalibration &s,
    const i2r_rlc::vision::PositionerCalibration &p,
    const i2r_rlc::robot_comm::RobotState &r)
{
    double u = s.sxM * (pixel.x() - s.cxPixel);
    double v = s.syM * (pixel.y() - s.cyPixel);
    const double distortion =
        1.0 / (1.0 + s.kappa * (u * u + v * v));
    u *= distortion;
    v *= distortion;
    const double scale = -1.0
        / (s.laserPlaneA * u + s.laserPlaneB * v
           + s.laserPlaneC * s.focalLengthM);
    const Eigen::Vector3d cameraMm(
        scale * u * 1000.0,
        scale * v * 1000.0,
        scale * s.focalLengthM * 1000.0);

    const Eigen::Affine3d cameraInTool =
        Eigen::Translation3d(
            s.cameraXInToolM * 1000.0,
            s.cameraYInToolM * 1000.0,
            s.cameraZInToolM * 1000.0)
        * (Eigen::AngleAxisd(rad(s.cameraRxInToolDeg), Eigen::Vector3d::UnitX())
           * Eigen::AngleAxisd(rad(s.cameraRyInToolDeg), Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(rad(s.cameraRzInToolDeg), Eigen::Vector3d::UnitZ()));
    const Eigen::Affine3d toolInRobot =
        Eigen::Translation3d(
            r.curx_r / 1000.0, r.cury_r / 1000.0, r.curz_r / 1000.0)
        * (Eigen::AngleAxisd(rad(r.curRz_r / 10000.0), Eigen::Vector3d::UnitZ())
           * Eigen::AngleAxisd(rad(r.curRy_r / 10000.0), Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(rad(r.curRx_r / 10000.0), Eigen::Vector3d::UnitX()));
    const Eigen::Affine3d robotToStation =
        Eigen::Translation3d(
            -p.toolTcpInLastLinkMm.x(),
            -p.toolTcpInLastLinkMm.y(),
            -p.toolTcpInLastLinkMm.z())
        * Eigen::Affine3d(
            (Eigen::AngleAxisd(rad(-r.curS2 / 10000.0), Eigen::Vector3d::UnitZ())
             * Eigen::AngleAxisd(rad(-r.curS1 / 10000.0), Eigen::Vector3d::UnitY()))
                .toRotationMatrix())
        * p.stationBaseInRobotBase;
    return robotToStation * toolInRobot * cameraInTool * cameraMm;
}

} // namespace

int main()
{
    const fs::path root = fs::path(I2R_RLC_PROJECT_SOURCE_DIR);
    const fs::path replay = root / "data" / "replay" / "penzui";
    const auto sensor = makeSensor(readSensorValues(replay / "SENSORPARA.txt"));
    const auto positioner =
        i2r_rlc::vision::CPositionerCalibrationParser::parseFile(
            (replay / "RBCALIB.DAT").string());
    const auto pixels = readImagePoints(replay / "simulatedWeldpt.txt");
    const auto states = readRobotStates(replay / "robotPos.txt");
    if (!positioner.valid || states.size() != 702U || pixels.size() < states.size()) {
        std::cerr << "invalid penzui replay inputs\n";
        return 1;
    }

    fs::create_directories(root / "output" / "reports");
    std::ofstream report(
        root / "output" / "reports" / "penzui_station_trajectory.csv");
    report << "frame,pixel_x,pixel_y,station_x_mm,station_y_mm,station_z_mm,error_mm\n";

    std::vector<Eigen::Vector3d> stationPoints;
    double maxFormulaError = 0.0;
    for (std::size_t index = 0; index < states.size(); ++index) {
        const auto context = i2r_rlc::vision::CWeldProjectionContextBuilder::build(
            sensor, positioner, states[index]);
        const auto projected =
            i2r_rlc::vision::CWeldImageToSpaceProjector::projectImagePoint(
                i2r_rlc::vision::WeldImagePoint{
                    pixels[index].x(), pixels[index].y(), true},
                context.context,
                i2r_rlc::vision::WeldProjectionTargetFrame::PositionerTcp);
        if (!context.valid || !projected.valid || !projected.spacePoint.valid) {
            std::cerr << "invalid projection at frame " << index << '\n';
            return 2;
        }
        const Eigen::Vector3d actual(
            projected.spacePoint.xMm,
            projected.spacePoint.yMm,
            projected.spacePoint.zMm);
        const Eigen::Vector3d expected = independentLegacyProjection(
            pixels[index], sensor, positioner, states[index]);
        const double error = (actual - expected).norm();
        maxFormulaError = std::max(maxFormulaError, error);
        stationPoints.push_back(actual);
        report << index << ',' << pixels[index].x() << ',' << pixels[index].y()
               << ',' << actual.x() << ',' << actual.y() << ',' << actual.z()
               << ',' << error << '\n';
    }

    std::vector<double> steps;
    for (std::size_t index = 1; index < stationPoints.size(); ++index) {
        steps.push_back((stationPoints[index] - stationPoints[index - 1]).norm());
    }
    std::sort(steps.begin(), steps.end());
    const double p95 = steps[static_cast<std::size_t>(
        std::floor(0.95 * static_cast<double>(steps.size() - 1U)))];
    const double maxStep = steps.back();
    if (maxFormulaError > 1e-6 || p95 > 2.0 || maxStep > 6.0) {
        std::cerr << "projection acceptance failed: formula=" << maxFormulaError
                  << " p95=" << p95 << " max=" << maxStep << '\n';
        return 3;
    }

    std::cout << "PASS: penzui station projection frames=" << stationPoints.size()
              << " formula_max_mm=" << maxFormulaError
              << " step_p95_mm=" << p95
              << " step_max_mm=" << maxStep << '\n';
    return 0;
}
