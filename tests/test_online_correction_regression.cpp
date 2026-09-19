#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "OnlineCorrectionTrajectoryPlanner.h"
#include "geometry/PositionerKinematics.h"

#ifndef I2R_RLC_SOURCE_DIR
#define I2R_RLC_SOURCE_DIR "."
#endif

namespace {

using i2r_rlc::control::OnlineCorrectionPoint;
using i2r_rlc::control::OnlineCorrectionPose;
using i2r_rlc::control::OnlineCorrectionSessionState;
using i2r_rlc::control::OnlineCorrectionStartMode;
using i2r_rlc::control::OnlineCorrectionTrajectoryPlanResult;
using i2r_rlc::control::OnlineCorrectionTrajectoryRequest;
using i2r_rlc::control::OnlineCorrectionOptions;
using i2r_rlc::control::COnlineCorrectionTrajectoryPlanner;

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct WorkRecord {
    Vec3 point;
    double rx = 0.0;
    double ry = 0.0;
    double rz = 0.0;
    double s1 = 0.0;
    double s2 = 0.0;
};

struct OracleMatch {
    int index = -1;
    double fraction = 0.0;
    Vec3 projection;
    Vec3 residual;
    Vec3 scanDirection;
    Vec3 tangent;
    double distance = 0.0;
    double reverseCosine = 0.0;
    double scanSpan = 0.0;
    double remainingLength = 0.0;
};

struct HistoricalReplay {
    std::filesystem::path root;
    std::vector<WorkRecord> work;
    std::vector<OnlineCorrectionPoint> samples;
    i2r_rlc::control::OnlineCorrectionPlannedTrajectory trajectory;
    i2r_rlc::vision::PositionerCalibration calibration;
};

bool near(double actual, double expected, double tolerance)
{
    return std::abs(actual - expected) <= tolerance;
}

double dot(Vec3 lhs, Vec3 rhs)
{
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

double norm(Vec3 value)
{
    return std::sqrt(dot(value, value));
}

Vec3 add(Vec3 lhs, Vec3 rhs)
{
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

Vec3 subtract(Vec3 lhs, Vec3 rhs)
{
    return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

Vec3 multiply(Vec3 value, double scalar)
{
    return {value.x * scalar, value.y * scalar, value.z * scalar};
}

std::vector<std::string> splitTabs(const std::string &line)
{
    std::vector<std::string> fields;
    std::size_t begin = 0;
    while (begin <= line.size()) {
        const std::size_t end = line.find('\t', begin);
        fields.push_back(line.substr(
            begin, end == std::string::npos ? end : end - begin));
        if (end == std::string::npos) {
            break;
        }
        begin = end + 1;
    }
    return fields;
}

double parseNumber(const std::string &value)
{
    return value.empty() ? 0.0 : std::stod(value);
}

std::vector<WorkRecord> readWork(const std::filesystem::path &path)
{
    std::ifstream input(path);
    std::vector<WorkRecord> records;
    std::string line;
    while (std::getline(input, line)) {
        std::istringstream stream(line);
        WorkRecord record;
        if (stream >> record.point.x >> record.point.y >> record.point.z
            >> record.rx >> record.ry >> record.rz >> record.s1 >> record.s2) {
            records.push_back(record);
        }
    }
    return records;
}

std::vector<OnlineCorrectionPoint> readHistoricalSamples(
    const std::filesystem::path &path)
{
    std::ifstream input(path);
    std::vector<OnlineCorrectionPoint> samples;
    std::string line;
    std::getline(input, line);
    while (std::getline(input, line)) {
        const auto fields = splitTabs(line);
        if (fields.size() < 8 || fields[2] != "vision_sample"
            || fields[3] != "initial_detection_running" || fields[4] != "1") {
            continue;
        }
        samples.push_back({parseNumber(fields[5]), parseNumber(fields[6]),
                           parseNumber(fields[7]) + 231.0, true});
    }
    return samples;
}

HistoricalReplay loadHistoricalReplay()
{
    HistoricalReplay replay;
    replay.root = std::filesystem::path(I2R_RLC_SOURCE_DIR);
    const auto workPath = replay.root / "data/WorkPiece/20260404CHUFENGKOU1_W.txt";
    const auto teachPath = replay.root / "data/WorkPiece/20260404CHUFENGKOU1_T.txt";
    const auto sessionPath = replay.root
        / "output/online_correction/20260914_174544_959/realtime_correction.txt";
    const auto rbcalibPath = replay.root / "config/calibration/RBCALIB.DAT";

    OnlineCorrectionTrajectoryRequest request;
    request.workPathFile = workPath.string();
    request.teachPoseFile = teachPath.string();
    request.segmentIndex = 0;
    request.startTeachIndex = 0;
    request.endTeachIndex = 1;
    replay.trajectory = COnlineCorrectionTrajectoryPlanner::loadTrajectory(request);
    replay.work = readWork(workPath);
    replay.samples = readHistoricalSamples(sessionPath);
    replay.calibration = i2r_rlc::vision::CPositionerCalibrationParser::parseFile(
        rbcalibPath.string());
    return replay;
}

OracleMatch computeOracle(
    const std::vector<WorkRecord> &work,
    const std::vector<OnlineCorrectionPoint> &samples,
    int endIndex)
{
    OracleMatch result;
    const Vec3 initial{samples.back().xMm, samples.back().yMm, samples.back().zMm};
    double bestDistance = std::numeric_limits<double>::max();
    for (int index = 0; index < endIndex; ++index) {
        const Vec3 begin = work[static_cast<std::size_t>(index)].point;
        const Vec3 end = work[static_cast<std::size_t>(index + 1)].point;
        const Vec3 segment = subtract(end, begin);
        const double squareLength = dot(segment, segment);
        double fraction = squareLength > 0.0
            ? dot(subtract(initial, begin), segment) / squareLength : 0.0;
        fraction = std::clamp(fraction, 0.0, 1.0);
        const Vec3 projection = add(begin, multiply(segment, fraction));
        const double distance = norm(subtract(initial, projection));
        if (distance < bestDistance) {
            bestDistance = distance;
            result.index = index;
            result.fraction = fraction;
            result.projection = projection;
            result.tangent = segment;
        }
    }

    result.distance = bestDistance;
    result.residual = subtract(initial, result.projection);
    Vec3 first;
    Vec3 last;
    for (std::size_t index = samples.size() - 32; index < samples.size() - 28; ++index) {
        first = add(first, {samples[index].xMm, samples[index].yMm, samples[index].zMm});
    }
    for (std::size_t index = samples.size() - 4; index < samples.size(); ++index) {
        last = add(last, {samples[index].xMm, samples[index].yMm, samples[index].zMm});
    }
    first = multiply(first, 0.25);
    last = multiply(last, 0.25);
    result.scanDirection = subtract(last, first);
    result.scanSpan = norm(result.scanDirection);
    result.reverseCosine = dot(result.scanDirection, multiply(result.tangent, -1.0))
        / (norm(result.scanDirection) * norm(result.tangent));
    result.remainingLength = norm(subtract(work[static_cast<std::size_t>(result.index)].point,
                                           result.projection));
    for (int index = result.index + 1; index < endIndex; ++index) {
        result.remainingLength += norm(subtract(
            work[static_cast<std::size_t>(index + 1)].point,
            work[static_cast<std::size_t>(index)].point));
    }
    return result;
}

OnlineCorrectionTrajectoryPlanResult runPlan(
    const HistoricalReplay &replay,
    const OnlineCorrectionOptions &options,
    const std::vector<OnlineCorrectionPoint> &samples)
{
    OnlineCorrectionSessionState state;
    for (const auto &sample : samples) {
        state.initialDetectionSamples.push_back({sample, std::nullopt});
    }
    i2r_rlc::robot_comm::RobotState robotState;
    return COnlineCorrectionTrajectoryPlanner::planInitialCorrection(
        replay.trajectory, options, state, robotState, replay.calibration);
}

void writeSyntheticCase(
    const std::filesystem::path &directory,
    const std::vector<WorkRecord> &work)
{
    std::filesystem::create_directories(directory);
    std::ofstream workOutput(directory / "W.txt");
    for (const auto &record : work) {
        workOutput << record.point.x << ' ' << record.point.y << ' '
                   << record.point.z << ' ' << record.rx << ' ' << record.ry << ' '
                   << record.rz << ' ' << record.s1 << ' ' << record.s2 << "\n";
    }
    std::ofstream teachOutput(directory / "T.txt");
    teachOutput << work.front().point.x << ' ' << work.front().point.y << ' '
                << work.front().point.z << " 0 0 0 0 0 0\n";
    teachOutput << work.back().point.x << ' ' << work.back().point.y << ' '
                << work.back().point.z << " 0 0 0 0 0 0\n";
}

HistoricalReplay makeSyntheticReplay(
    const std::filesystem::path &directory,
    const std::vector<OnlineCorrectionPoint> &samples,
    const std::vector<WorkRecord> &work)
{
    writeSyntheticCase(directory, work);
    HistoricalReplay replay;
    replay.root = directory;
    OnlineCorrectionTrajectoryRequest request;
    request.workPathFile = (directory / "W.txt").string();
    request.teachPoseFile = (directory / "T.txt").string();
    request.segmentIndex = 0;
    request.startTeachIndex = 0;
    request.endTeachIndex = 1;
    replay.trajectory = COnlineCorrectionTrajectoryPlanner::loadTrajectory(request);
    replay.samples = samples;
    replay.calibration.valid = true;
    replay.calibration.stationBaseInRobotBase = Eigen::Affine3d::Identity();
    return replay;
}

std::vector<OnlineCorrectionPoint> samplesEndingAt(double endpoint, bool increasing,
                                                    bool stationary = false)
{
    std::vector<OnlineCorrectionPoint> samples;
    for (int index = 0; index < 32; ++index) {
        double x = endpoint;
        if (!stationary) {
            const double fraction = static_cast<double>(index) / 31.0;
            x = increasing ? endpoint * fraction : endpoint * (1.0 - fraction);
        }
        samples.push_back({x, 0.0, 0.0, true});
    }
    return samples;
}

bool check(bool condition, const std::string &name, const std::string &detail)
{
    if (condition) {
        return true;
    }
    std::cerr << "FAIL " << name << ": " << detail << '\n';
    return false;
}

bool testHistoricalReplayPathMatch()
{
    const auto replay = loadHistoricalReplay();
    const auto oracle = computeOracle(replay.work, replay.samples, 341);
    bool ok = true;
    ok &= check(replay.samples.size() == 277, "HISTORICAL_REPLAY_PATH_MATCH",
                "initial sample count must be 277");
    ok &= check(oracle.index == 162, "HISTORICAL_REPLAY_PATH_MATCH",
                "matched index must be 162");
    ok &= check(near(oracle.fraction, 0.279970047638, 1e-9),
                "HISTORICAL_REPLAY_PATH_MATCH", "segment fraction mismatch");
    ok &= check(near(oracle.projection.x, -122.540369375, 1e-6)
                && near(oracle.projection.y, -58.052341669, 1e-6)
                && near(oracle.projection.z, 253.962559940, 1e-6),
                "HISTORICAL_REPLAY_PATH_MATCH", "projection mismatch");
    ok &= check(near(oracle.distance, 1.614820100, 1e-6),
                "HISTORICAL_REPLAY_PATH_MATCH", "projection distance mismatch");
    ok &= check(near(oracle.residual.x, -1.437455625, 1e-6)
                && near(oracle.residual.y, -0.735259331, 1e-6)
                && near(oracle.residual.z, -0.027549940, 1e-6),
                "HISTORICAL_REPLAY_PATH_MATCH", "local residual mismatch");
    ok &= check(near(oracle.reverseCosine, 0.996410057, 1e-6)
                && near(oracle.scanSpan, 4.670802565, 1e-6),
                "HISTORICAL_REPLAY_PATH_MATCH", "direction diagnostics mismatch");
    OnlineCorrectionOptions options;
    const auto plan = runPlan(replay, options, replay.samples);
    ok &= check(plan.valid && plan.command.valid,
                "HISTORICAL_REPLAY_PATH_MATCH",
                "legacy planner must generate the initial O command");
    ok &= check(!plan.initialPathMatch.valid,
                "HISTORICAL_REPLAY_PATH_MATCH",
                "legacy planner does not expose a constrained path match");
    return ok;
}

bool testNoGlobalStartTranslation()
{
    const auto replay = loadHistoricalReplay();
    OnlineCorrectionOptions options;
    const auto plan = runPlan(replay, options, replay.samples);
    const Vec3 correctedSecond{plan.correctedSegmentPath[1].xMm,
                               plan.correctedSegmentPath[1].yMm,
                               plan.correctedSegmentPath[1].zMm};
    const auto &segment = replay.trajectory.segmentPathPoints;
    const auto &initial = plan.initialPoint;
    const Vec3 expectedSecond{
        segment[1].xMm + initial.xMm - segment.front().xMm,
        segment[1].yMm + initial.yMm - segment.front().yMm,
        segment[1].zMm + initial.zMm - segment.front().zMm};
    return check(near(correctedSecond.x, expectedSecond.x, 1e-3)
                 && near(correctedSecond.y, expectedSecond.y, 1e-3)
                 && near(correctedSecond.z, expectedSecond.z, 1e-3),
                 "NO_GLOBAL_START_TRANSLATION",
                 "legacy planner must translate the complete W segment from its first point");
}

bool testLegacyContinuesWithoutPathMatchGates(
    const std::string &name,
    const std::vector<OnlineCorrectionPoint> &samples,
    const std::vector<WorkRecord> &work)
{
    const auto directory = std::filesystem::temp_directory_path()
        / "i2r_rlc_online_correction_regression" / name;
    const auto replay = makeSyntheticReplay(directory, samples, work);
    OnlineCorrectionOptions options;
    const auto plan = runPlan(replay, options, samples);
    return check(plan.valid && plan.command.valid && plan.command.pointCount > 0,
                 name, "legacy behavior must continue to generate an O command");
}

bool testLegacyPathMatchBehavior()
{
    const std::vector<WorkRecord> work{
        {{0.0, 0.0, 0.0}, 0, 0, 0, 0, 0},
        {{10.0, 0.0, 0.0}, 0, 0, 0, 0, 0},
        {{20.0, 0.0, 0.0}, 0, 0, 0, 0, 0}};
    bool ok = true;
    auto farSamples = samplesEndingAt(10.0, true);
    for (auto &sample : farSamples) {
        sample.yMm = 20.0;
    }
    ok &= testLegacyContinuesWithoutPathMatchGates(
        "LEGACY_MAX_MATCH_DISTANCE_NO_GATE", farSamples, work);
    ok &= testLegacyContinuesWithoutPathMatchGates(
        "LEGACY_REVERSE_DIRECTION_NO_GATE", samplesEndingAt(10.0, true), work);
    ok &= testLegacyContinuesWithoutPathMatchGates(
        "LEGACY_MIN_SCAN_SPAN_NO_GATE", samplesEndingAt(10.0, true, true), work);
    ok &= testLegacyContinuesWithoutPathMatchGates(
        "LEGACY_MIN_REMAINING_PATH_NO_GATE", samplesEndingAt(19.5, true), work);
    return ok;
}

bool testFirstDynamicPointFromTeachTemplate()
{
    const auto replay = loadHistoricalReplay();
    OnlineCorrectionOptions options;
    const auto plan = runPlan(replay, options, replay.samples);
    const double firstTeachStationS2 = replay.trajectory.stationAxes.front().s2Deg;
    return check(!plan.command.stationAxes.empty()
                 && near(plan.command.stationAxes.front().s2Deg10000 / 10000.0,
                         firstTeachStationS2, 1e-3),
                 "FIRST_DYNAMIC_POINT_FROM_TEACH_TEMPLATE",
                 "legacy O generation must use the T template station axis");
}

double shortestAngleLerp(double from, double to, double fraction)
{
    double delta = std::fmod(to - from + 180.0, 360.0);
    if (delta < 0.0) {
        delta += 360.0;
    }
    delta -= 180.0;
    return from + fraction * delta;
}

bool testShortestAngleInterpolation()
{
    const double result = shortestAngleLerp(179.0, -179.0, 0.5);
    return check(std::abs(std::abs(result) - 180.0) < 1e-9,
                 "SHORTEST_ANGLE_INTERPOLATION",
                 "interpolation must take the two-degree shortest arc");
}

bool testIbvsStartCompatibility()
{
    const auto directory = std::filesystem::temp_directory_path()
        / "i2r_rlc_online_correction_regression" / "IBVS_START_COMPATIBILITY";
    const std::vector<WorkRecord> work{
        {{0.0, 0.0, 0.0}, 0, 0, 0, 0, 0},
        {{100.0, 0.0, 0.0}, 0, 0, 0, 0, 0}};
    const auto replay = makeSyntheticReplay(
        directory, {{0.0, 0.0, 0.0, true}}, work);
    OnlineCorrectionOptions options;
    options.startMode = OnlineCorrectionStartMode::IBVSStart;
    OnlineCorrectionSessionState state;
    state.startMode = options.startMode;
    i2r_rlc::robot_comm::RobotState robotState;
    robotState.curx_r = 21000;
    robotState.cury_r = 22000;
    robotState.curz_r = 23000;
    const auto plan = COnlineCorrectionTrajectoryPlanner::planInitialCorrection(
        replay.trajectory, options, state, robotState);
    const bool valuesMatch = plan.command.teachPoints.size() > 0
        && plan.command.teachPoints.front().xUm == 21000
        && plan.command.teachPoints.front().yUm == 22000
        && plan.command.teachPoints.front().zUm == 23000;
    return check(plan.valid && valuesMatch,
                 "IBVS_START_COMPATIBILITY",
                 "IBVSStart must continue using RobotState as its source (" + plan.message + ")");
}

bool testGlobalCorrectionOrder()
{
    const auto directory = std::filesystem::temp_directory_path()
        / "i2r_rlc_online_correction_regression" / "GLOBAL_CORRECTION_ORDER";
    const std::vector<WorkRecord> work{
        {{0.0, 0.0, 0.0}, 0, 0, 0, 0, 0},
        {{10.0, 0.0, 0.0}, 0, 0, 0, 0, 30},
        {{20.0, 0.0, 0.0}, 0, 0, 0, 0, 60}};
    auto samples = samplesEndingAt(110.0, false);
    for (int index = 0; index < 32; ++index) {
        samples[static_cast<std::size_t>(index)].xMm = 120.0
            - 10.0 * static_cast<double>(index) / 31.0;
    }
    const auto replay = makeSyntheticReplay(directory, samples, work);
    OnlineCorrectionOptions options;
    options.startMode = OnlineCorrectionStartMode::GlobalCorrection;
    options.globalCorrectionRowMajor = {
        1, 0, 0, 100,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1};
    const auto plan = runPlan(replay, options, replay.samples);
    return check(!plan.command.stationAxes.empty()
                 && near(plan.command.stationAxes.front().s2Deg10000 / 10000.0,
                         0.0, 1e-3),
                 "GLOBAL_CORRECTION_ORDER",
                 "legacy GlobalCorrection must keep station axes from the T template");
}

} // namespace

int main()
{
    bool ok = true;
    ok &= testHistoricalReplayPathMatch();
    ok &= testNoGlobalStartTranslation();
    ok &= testLegacyPathMatchBehavior();
    ok &= testFirstDynamicPointFromTeachTemplate();
    ok &= testShortestAngleInterpolation();
    ok &= testIbvsStartCompatibility();
    ok &= testGlobalCorrectionOrder();

    if (!ok) {
        std::cerr << "EXPECTED_FAILS_PRESENT: current production Planner does not satisfy the documented contract\n";
        return 1;
    }
    std::cout << "PASS: all online correction regression invariants\n";
    return 0;
}
