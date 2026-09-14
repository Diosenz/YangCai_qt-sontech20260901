/*******************************************************************************
 * @file        test_automatic_calibration_pose.cpp
 * @brief       自动标定位姿文件解析与修正测试入口
 * @details     验证旧 CalibPose.txt 模板、CalibPose_new.txt 写出和 O 命令 DTO。
 ******************************************************************************/
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "AutomaticCalibrationPoseController.h"

namespace fs = std::filesystem;

namespace {

const char *kLegacyCalibPoseText =
    "864455 -29634 -110860 1776237 2708 26367 0 0\n"
    "884821 -42547 -114302 1786071 7769 28627 0 0\n"
    "865099 -43237 -110901 1790147 9881 29639 0 0\n"
    "840393 -41925 -110813 1795704 31491 29994 0 0\n"
    "839961 -29986 -110856 1795608 31535 30169 0 0\n"
    "887966 -30843 -120565 -1743345 -2294 30761 0 0\n"
    "888212 -17614 -119582 -1780279 -44731 41844 0 0\n"
    "866887 -16283 -117745 1791931 -6151 30855 0 0\n"
    "841265 -15888 -121217 -1761634 41381 19735 0 0\n"
    "874799 -27918 -104829 1666757 111463 16329 0 0\n"
    "857064 -37890 -122301 -1583053 38563 63442 0 0\n"
    "868949 -25248 -111138 -1795172 242328 44372 0 0\n"
    "870687 -31223 -105641 1723389 -159039 38182 0 0\n"
    "870150 -29289 -111430 1751183 -20647 -93694 0 0\n"
    "865724 -31643 -107169 1751177 -20668 214009 0 0\n"
    "862640 -21265 -109348 1751246 -20642 -150437 0 0\n"
    "872307 -22003 -108161 1770136 -25401 139571 0 0\n"
    "863465 -23441 -110165 -1781918 5648 238027 0 0\n"
    "882114 -20850 -113672 1751225 -20691 312051 0 0\n";

bool expectTrue(bool condition, const std::string &message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool expectLong(long actual, long expected, const std::string &message)
{
    if (actual != expected) {
        std::cerr << "FAIL: " << message << ", actual=" << actual
                  << ", expected=" << expected << '\n';
        return false;
    }
    return true;
}

fs::path makeCaseRoot(const std::string &name)
{
    const fs::path root =
        fs::temp_directory_path() / ("i2r_rlc_auto_calib_pose_" + name);
    fs::remove_all(root);
    fs::create_directories(root);
    return root;
}

void writeText(const fs::path &path, const std::string &text)
{
    fs::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::trunc);
    output << text;
}

std::vector<std::string> readLines(const fs::path &path)
{
    std::ifstream input(path);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(input, line)) {
        lines.push_back(line);
    }
    return lines;
}

std::string makeSixFieldPoseText(const int count)
{
    std::string text;
    for (int index = 0; index < count; ++index) {
        text += std::to_string(1000 + index) + " "
            + std::to_string(2000 + index) + " "
            + std::to_string(3000 + index) + " "
            + std::to_string(4000 + index) + " "
            + std::to_string(5000 + index) + " "
            + std::to_string(6000 + index) + "\n";
    }
    return text;
}

i2r_rlc::robot_comm::RobotState makeCurrentRobotState()
{
    i2r_rlc::robot_comm::RobotState state;
    state.curx_r = 900000;
    state.cury_r = -10000;
    state.curz_r = -90000;
    state.curRx_r = 111;
    state.curRy_r = 222;
    state.curRz_r = 123456;
    return state;
}

bool testLegacyEightFieldTemplate()
{
    const fs::path root = makeCaseRoot("legacy_8");
    const fs::path filePath = root / "CalibPose.txt";
    writeText(filePath, kLegacyCalibPoseText);

    i2r_rlc::control::CAutomaticCalibrationPoseController controller;
    const auto result = controller.makeTemplateCommand(filePath.string());

    bool ok = true;
    ok &= expectTrue(result.succeeded, "legacy 8-field CalibPose should parse");
    ok &= expectTrue(result.poses.size() == 19U, "legacy CalibPose should contain 19 poses");
    ok &= expectLong(result.poses.front().xUm, 864455, "first x should match legacy file");
    ok &= expectLong(result.poses.front().s1Deg10000, 0, "first s1 should match legacy file");
    ok &= expectLong(result.poses.back().rzDeg10000, 312051, "last rz should match legacy file");
    ok &= expectTrue(result.robotCommand.valid, "template command should be valid");
    ok &= expectLong(result.robotCommand.pointCount, 19, "template command point count");
    ok &= expectLong(result.robotCommand.offlineSignal, 4, "template command offline signal");
    ok &= expectTrue(result.robotCommand.stationAxes.size() == 19U,
                     "8-field template should keep station axes");
    return ok;
}

bool testSixFieldPoseFile()
{
    const fs::path root = makeCaseRoot("six_field");
    const fs::path filePath = root / "CalibPose_new.txt";
    writeText(filePath, makeSixFieldPoseText(19));

    i2r_rlc::control::CAutomaticCalibrationPoseController controller;
    const auto result = controller.loadPoseFile(filePath.string());

    bool ok = true;
    ok &= expectTrue(result.succeeded, "6-field CalibPose_new should parse");
    ok &= expectTrue(result.poses.size() == 19U, "6-field file should contain 19 poses");
    ok &= expectTrue(!result.poses.front().stationAxesValid,
                     "6-field file should not expose station axes");
    return ok;
}

bool testCorrectionAndWrite()
{
    const fs::path root = makeCaseRoot("correction");
    const fs::path templatePath = root / "CalibPose.txt";
    const fs::path correctedPath =
        root / "Data" / "CALID" / "4 TWO_STEP" / "CalibPose_new.txt";
    writeText(templatePath, kLegacyCalibPoseText);

    i2r_rlc::control::CAutomaticCalibrationPoseController controller;
    const auto result = controller.makeCorrectedCommand(
        templatePath.string(),
        correctedPath.string(),
        makeCurrentRobotState());

    bool ok = true;
    ok &= expectTrue(result.succeeded, "correction should succeed");
    ok &= expectTrue(fs::exists(correctedPath), "CalibPose_new.txt should be written");
    ok &= expectLong(result.poses.front().xUm, 900000, "corrected first x should match current");
    ok &= expectLong(result.poses.front().yUm, -10000, "corrected first y should match current");
    ok &= expectLong(result.poses.front().zUm, -90000, "corrected first z should match current");
    ok &= expectLong(result.poses.front().rxDeg10000, 1776237, "corrected first rx should keep template");
    ok &= expectLong(result.poses.front().ryDeg10000, 2708, "corrected first ry should keep template");
    ok &= expectLong(result.poses.front().rzDeg10000, 123456, "corrected first rz should match current");
    ok &= expectLong(result.poses[1].xUm, 920366, "corrected second x should keep template offset");
    ok &= expectLong(result.poses[1].rzDeg10000, 125716, "corrected second rz should keep template offset");
    ok &= expectTrue(result.robotCommand.valid, "corrected command should be valid");
    ok &= expectLong(result.robotCommand.pointCount, 19, "corrected command point count");
    ok &= expectLong(result.robotCommand.offlineSignal, 4, "corrected command offline signal");
    ok &= expectTrue(result.robotCommand.stationAxes.empty(),
                     "corrected command should mirror 6-field CalibPose_new station axes");

    const std::vector<std::string> lines = readLines(correctedPath);
    ok &= expectTrue(lines.size() == 19U, "CalibPose_new should contain 19 lines");
    ok &= expectTrue(
        lines.front() == "900000 -10000 -90000 1776237 2708 123456",
        "CalibPose_new first line should use 6-field legacy format");
    return ok;
}

bool testInvalidFilesFail()
{
    const fs::path root = makeCaseRoot("invalid");
    const fs::path shortPath = root / "short.txt";
    const fs::path missingColumnPath = root / "missing_column.txt";
    const fs::path invalidPath = root / "invalid.txt";
    writeText(shortPath, makeSixFieldPoseText(18));
    writeText(missingColumnPath, "1 2 3 4 5\n");
    writeText(invalidPath, "1 2 3 4 5 bad\n");

    i2r_rlc::control::CAutomaticCalibrationPoseController controller;
    const auto shortResult = controller.loadPoseFile(shortPath.string());
    const auto missingColumnResult =
        controller.loadPoseFile(missingColumnPath.string());
    const auto invalidResult = controller.loadPoseFile(invalidPath.string());

    bool ok = true;
    ok &= expectTrue(!shortResult.succeeded, "18-line CalibPose should fail");
    ok &= expectTrue(!missingColumnResult.succeeded, "5-field CalibPose should fail");
    ok &= expectTrue(!invalidResult.succeeded, "non-integer CalibPose should fail");
    return ok;
}

} // namespace

int main()
{
    bool ok = true;
    ok &= testLegacyEightFieldTemplate();
    ok &= testSixFieldPoseFile();
    ok &= testCorrectionAndWrite();
    ok &= testInvalidFilesFail();

    if (!ok) {
        return 1;
    }

    std::cout << "PASS: automatic calibration pose tests\n";
    return 0;
}
