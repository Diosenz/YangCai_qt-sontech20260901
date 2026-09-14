/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_sensor_calibration_parser.cpp
 * @brief       sensor calibration parser 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <cmath>
#include <iostream>

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>

#include "AppPaths.h"
#include "ConfigRepository.h"
#include "SensorCalibrationParser.h"

namespace {

using i2r_rlc::ui::AppPaths;
using i2r_rlc::ui::ConfigRepository;
using i2r_rlc::ui::SensorCalibrationParser;

constexpr double kTolerance = 1e-12;

QString validSensorParaText()
{
    return QString::fromUtf8(R"(Calib_Method 4
f/m  0.012139
Kappa  -2614.568100
sx/m 0.0000037500219435
sy/m 0.0000037500000000
cx 675.819942
cy 494.484186
a  -0.192572
b  12.570285
c  -4.622321
pose_CinT[0]/m  -0.009732
pose_CinT[1]/m  -0.076993
pose_CinT[2]/m  -0.222147
pose_CinT[3]/deg  352.448772
pose_CinT[4]/deg  4.839075
pose_CinT[5]/deg  358.728728
pose_CinT[6]  1652807680.000000
More Info Please refer to data\CALID\4 TWO_STEP\CalibResult.txt
Tue Jul  4 09:55:06 2023
UNKNOWN_LINE should be ignored
)");
}

bool expectTrue(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }

    return true;
}

bool expectNear(double actual, double expected, const char* message)
{
    if (std::abs(actual - expected) > kTolerance) {
        std::cerr << "FAIL: " << message << ", actual=" << actual << ", expected=" << expected << '\n';
        return false;
    }

    return true;
}

bool writeTextFile(const QString& filePath, const QString& content)
{
    QFileInfo fileInfo(filePath);
    QDir().mkpath(fileInfo.absolutePath());

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        std::cerr << "FAIL: cannot write file " << filePath.toStdString() << '\n';
        return false;
    }

    file.write(content.toUtf8());
    return true;
}

QString makeTestRoot(const QString& caseName)
{
    QDir root(QDir::current().filePath(QStringLiteral("Testing/Temporary/i2r_rlc_config_tests/%1").arg(caseName)));
    if (root.exists()) {
        root.removeRecursively();
    }
    if (!QDir().mkpath(root.absolutePath())) {
        std::cerr << "FAIL: cannot create test root " << root.absolutePath().toStdString() << '\n';
        return {};
    }
    return root.absolutePath();
}

bool testParseCompleteSensorPara()
{
    SensorCalibrationParser parser;
    const auto result = parser.parseText(validSensorParaText(), QStringLiteral("memory:SENSORPARA.txt"));

    bool ok = true;
    ok &= expectTrue(result.ok, "complete SENSORPARA.txt should parse");
    ok &= expectTrue(result.data.calibMethod == QStringLiteral("4"), "Calib_Method should be parsed");
    ok &= expectNear(result.data.focalLengthM, 0.012139, "focalLengthM");
    ok &= expectNear(result.data.kappa, -2614.568100, "kappa");
    ok &= expectNear(result.data.sxM, 0.0000037500219435, "sxM");
    ok &= expectNear(result.data.syM, 0.0000037500000000, "syM");
    ok &= expectNear(result.data.cxPixel, 675.819942, "cxPixel");
    ok &= expectNear(result.data.cyPixel, 494.484186, "cyPixel");
    ok &= expectNear(result.data.laserPlaneA, -0.192572, "laserPlaneA");
    ok &= expectNear(result.data.laserPlaneB, 12.570285, "laserPlaneB");
    ok &= expectNear(result.data.laserPlaneC, -4.622321, "laserPlaneC");
    ok &= expectNear(result.data.poseCinTXM, -0.009732, "poseCinTXM");
    ok &= expectNear(result.data.poseCinTYM, -0.076993, "poseCinTYM");
    ok &= expectNear(result.data.poseCinTZM, -0.222147, "poseCinTZM");
    ok &= expectNear(result.data.poseCinTRxDeg, 352.448772, "poseCinTRxDeg");
    ok &= expectNear(result.data.poseCinTRyDeg, 4.839075, "poseCinTRyDeg");
    ok &= expectNear(result.data.poseCinTRzDeg, 358.728728, "poseCinTRzDeg");
    ok &= expectNear(result.data.poseCinTReserved, 1652807680.000000, "poseCinTReserved");
    return ok;
}

bool testMissingRequiredFieldFails()
{
    SensorCalibrationParser parser;
    QString text = validSensorParaText();
    text.replace(QStringLiteral("cy 494.484186\n"), QString());

    const auto result = parser.parseText(text, QStringLiteral("memory:missing"));
    bool ok = true;
    ok &= expectTrue(!result.ok, "missing required field should fail");
    ok &= expectTrue(result.errorMessage.contains(QStringLiteral("cy")), "missing field error should mention cy");
    return ok;
}

bool testInvalidNumericValueFails()
{
    SensorCalibrationParser parser;
    QString text = validSensorParaText();
    text.replace(QStringLiteral("f/m  0.012139"), QStringLiteral("f/m not-a-number"));

    const auto result = parser.parseText(text, QStringLiteral("memory:invalid"));
    bool ok = true;
    ok &= expectTrue(!result.ok, "invalid numeric value should fail");
    ok &= expectTrue(result.errorMessage.contains(QStringLiteral("f/m")), "invalid numeric error should mention f/m");
    return ok;
}

bool testConfigRepositoryPrefersPrimaryPath()
{
    const QString tempPath = makeTestRoot(QStringLiteral("prefers_primary"));
    if (!expectTrue(!tempPath.isEmpty(), "temporary dir should be valid")) {
        return false;
    }

    const AppPaths paths = AppPaths::fromRuntimeRoot(tempPath);
    QString errorMessage;
    QString legacyText = validSensorParaText();
    legacyText.replace(QStringLiteral("Calib_Method 4"), QStringLiteral("Calib_Method legacy"));
    bool ok = true;
    ok &= expectTrue(paths.ensureRuntimeDirectories(&errorMessage), "runtime directories should be created");
    ok &= writeTextFile(paths.sensorCalibrationFilePath(), validSensorParaText());
    ok &= writeTextFile(paths.legacySensorCalibrationFilePath(), legacyText);

    const ConfigRepository repository(paths);
    const QString foundPath = repository.findSensorCalibrationFile();
    const auto result = repository.loadSensorCalibration();

    ok &= expectTrue(QFileInfo(foundPath).canonicalFilePath() == QFileInfo(paths.sensorCalibrationFilePath()).canonicalFilePath(),
        "primary calibration path should be preferred");
    ok &= expectTrue(result.ok, "primary calibration file should load");
    ok &= expectTrue(result.data.calibMethod == QStringLiteral("4"), "primary calibration data should be used");
    return ok;
}

bool testConfigRepositoryUsesLegacyFallback()
{
    const QString tempPath = makeTestRoot(QStringLiteral("legacy_fallback"));
    if (!expectTrue(!tempPath.isEmpty(), "temporary dir should be valid")) {
        return false;
    }

    const AppPaths paths = AppPaths::fromRuntimeRoot(tempPath);
    QString errorMessage;
    QString legacyText = validSensorParaText();
    legacyText.replace(QStringLiteral("Calib_Method 4"), QStringLiteral("Calib_Method 7"));
    bool ok = true;
    ok &= expectTrue(paths.ensureRuntimeDirectories(&errorMessage), "runtime directories should be created");
    ok &= writeTextFile(paths.legacySensorCalibrationFilePath(), legacyText);

    const ConfigRepository repository(paths);
    const QString foundPath = repository.findSensorCalibrationFile();
    const auto result = repository.loadSensorCalibration();

    ok &= expectTrue(QFileInfo(foundPath).canonicalFilePath() == QFileInfo(paths.legacySensorCalibrationFilePath()).canonicalFilePath(),
        "legacy calibration path should be used when primary is missing");
    ok &= expectTrue(result.ok, "legacy calibration file should load");
    ok &= expectTrue(result.data.calibMethod == QStringLiteral("7"), "legacy calibration data should be used");
    return ok;
}

bool testConfigRepositoryMissingFileFails()
{
    const QString tempPath = makeTestRoot(QStringLiteral("missing_file"));
    if (!expectTrue(!tempPath.isEmpty(), "temporary dir should be valid")) {
        return false;
    }

    const AppPaths paths = AppPaths::fromRuntimeRoot(tempPath);
    QString errorMessage;
    bool ok = true;
    ok &= expectTrue(paths.ensureRuntimeDirectories(&errorMessage), "runtime directories should be created");

    const ConfigRepository repository(paths);
    const auto result = repository.loadSensorCalibration();

    ok &= expectTrue(repository.findSensorCalibrationFile().isEmpty(), "missing calibration file should not resolve");
    ok &= expectTrue(!result.ok, "missing calibration file should fail");
    ok &= expectTrue(result.errorMessage.contains(QStringLiteral("SENSORPARA.txt")), "missing file error should mention SENSORPARA.txt");
    return ok;
}

bool testPositionerCalibrationUsesFixedDefaultPath()
{
    const QString tempPath = makeTestRoot(QStringLiteral("rbcalib_default"));
    const AppPaths paths = AppPaths::fromRuntimeRoot(tempPath);
    QString errorMessage;
    bool ok = paths.ensureRuntimeDirectories(&errorMessage);
    ok &= expectTrue(
        paths.rbcalibFilePath().endsWith(
            QStringLiteral("config/calibration/RBCALIB.DAT")),
        "RBCALIB should use the fixed calibration path");
    ok &= expectTrue(
        paths.calibPoseTemplateFilePath().endsWith(
            QStringLiteral("config/calibration/CalibPose.txt")),
        "CalibPose template should use the fixed calibration path");
    ok &= expectTrue(
        paths.calibPoseCorrectedFilePath().endsWith(
            QStringLiteral("Data/CALID/4 TWO_STEP/CalibPose_new.txt")),
        "CalibPose_new should use the runtime CALID output path");
    ok &= writeTextFile(
        paths.rbcalibFilePath(),
        QStringLiteral("header\n////SRANG 10,20,30,1,2,3\n"));
    const ConfigRepository repository(paths);
    const auto loaded = repository.loadPositionerCalibration();
    ok &= expectTrue(loaded.valid, "valid default RBCALIB should load");
    ok &= expectTrue(
        std::abs(loaded.toolTcpInLastLinkMm.z() - 231.0) < 1e-9,
        "SWv1 positioner tool TCP offset should be restored");

    ok &= writeTextFile(paths.rbcalibFilePath(), QStringLiteral("missing marker\n"));
    const auto invalid = repository.loadPositionerCalibration();
    ok &= expectTrue(!invalid.valid, "RBCALIB without SRANG should fail");
    return ok;
}

bool testWorkpieceProfilesLoadAndResolveTeachFiles()
{
    const QString tempPath = makeTestRoot(QStringLiteral("workpiece_profiles"));
    const AppPaths paths = AppPaths::fromRuntimeRoot(tempPath);
    QString errorMessage;
    bool ok = true;
    ok &= expectTrue(paths.ensureRuntimeDirectories(&errorMessage), "runtime directories should be created");
    ok &= writeTextFile(
        paths.workpieceSelectionFilePath(),
        QString::fromUtf8(
            "# comment\n"
            "penzui|喷嘴|models/weldment/penzuicone.weldment|LOCJOB|ONLINEJOB|OFFDETJOB|20240828PENZUI|20250519PENZUIOFF\n"
            "\n"
            "zhongji|中集平板工件|models/weldment/zhongjiplane.weldment|LOC2|ONLINE2|OFFDET2|20250304ZHONGJIFANGBAN|20250304ZHONGJIFANGBANOFF\n"));

    const ConfigRepository repository(paths);
    const auto result = repository.loadWorkpieceProfiles();

    ok &= expectTrue(result.ok, "valid workpiece profiles should load");
    ok &= expectTrue(result.profiles.size() == 2, "two workpiece profiles should be loaded");
    ok &= expectTrue(result.profiles.at(0).displayName == QStringLiteral("喷嘴"),
                     "Chinese workpiece display name should be preserved");
    ok &= expectTrue(result.profiles.at(0).onlineJbi == QStringLiteral("ONLINEJOB"),
                     "legacy 8-column profile should keep InitType online JBI");
    ok &= expectTrue(result.profiles.at(0).onlineGlobalCorrectionJbi == QStringLiteral("ONLINEJOB"),
                     "legacy 8-column profile should reuse online JBI for GlobalCorrection");
    ok &= expectTrue(result.profiles.at(0).onlineIbvsStartJbi == QStringLiteral("ONLINEJOB"),
                     "legacy 8-column profile should reuse online JBI for IBVSStart");
    ok &= expectTrue(
        repository.workpieceOnlineWorkPathFile(result.profiles.at(0)).endsWith(
            QStringLiteral("data/WorkPiece/20240828PENZUI_W.txt")),
        "online workpiece _W path should be derived from online teach base name");
    ok &= expectTrue(
        repository.workpieceOfflineCorrectionTeachPoseFile(result.profiles.at(1)).endsWith(
            QStringLiteral("data/WorkPiece/20250304ZHONGJIFANGBANOFF_T.txt")),
        "offline correction _T path should be derived from offline teach base name");
    return ok;
}

bool testWorkpieceProfilesLoadStartModeSpecificOnlineJobs()
{
    const QString tempPath = makeTestRoot(QStringLiteral("workpiece_profiles_start_mode_jobs"));
    const AppPaths paths = AppPaths::fromRuntimeRoot(tempPath);
    QString errorMessage;
    bool ok = true;
    ok &= expectTrue(paths.ensureRuntimeDirectories(&errorMessage), "runtime directories should be created");
    ok &= writeTextFile(
        paths.workpieceSelectionFilePath(),
        QString::fromUtf8(
            "penzui|喷嘴|models/weldment/penzuicone.weldment|LOCJOB|ONLINE_INIT|ONLINE_GLOBAL|ONLINE_IBVS|OFFDETJOB|20240828PENZUI|20250519PENZUIOFF\n"));

    const ConfigRepository repository(paths);
    const auto result = repository.loadWorkpieceProfiles();

    ok &= expectTrue(result.ok, "10-column workpiece profile should load");
    ok &= expectTrue(result.profiles.size() == 1, "one workpiece profile should be loaded");
    ok &= expectTrue(result.profiles.at(0).onlineJbi == QStringLiteral("ONLINE_INIT"),
                     "InitType online JBI should be parsed from column 5");
    ok &= expectTrue(result.profiles.at(0).onlineGlobalCorrectionJbi == QStringLiteral("ONLINE_GLOBAL"),
                     "GlobalCorrection online JBI should be parsed from column 6");
    ok &= expectTrue(result.profiles.at(0).onlineIbvsStartJbi == QStringLiteral("ONLINE_IBVS"),
                     "IBVSStart online JBI should be parsed from column 7");
    ok &= expectTrue(result.profiles.at(0).offlineCorrectionJbi == QStringLiteral("OFFDETJOB"),
                     "offline correction JBI should move to column 8 in 10-column format");
    return ok;
}

bool testWorkpieceProfilesRejectInvalidRows()
{
    const QString tempPath = makeTestRoot(QStringLiteral("workpiece_invalid"));
    const AppPaths paths = AppPaths::fromRuntimeRoot(tempPath);
    QString errorMessage;
    bool ok = true;
    ok &= expectTrue(paths.ensureRuntimeDirectories(&errorMessage), "runtime directories should be created");
    ok &= writeTextFile(
        paths.workpieceSelectionFilePath(),
        QStringLiteral("bad|坏行|models/x.weldment|LOC|ONLINE|OFFDET|NAME|NAME_W.txt\n"));

    const ConfigRepository repository(paths);
    const auto result = repository.loadWorkpieceProfiles();
    ok &= expectTrue(!result.ok, "workpiece profile with _W suffix should fail");
    ok &= expectTrue(result.message.contains(QStringLiteral("_W/_T")),
                     "workpiece suffix error should mention _W/_T");
    return ok;
}

} // namespace

/**
 * @brief 配置和标定解析轻量级测试入口。
 *
 * 该测试不依赖 Qt Widgets，不访问旧工程目录，只验证第四阶段的数据与配置支撑能力。
 */
int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    bool ok = true;
    ok &= testParseCompleteSensorPara();
    ok &= testMissingRequiredFieldFails();
    ok &= testInvalidNumericValueFails();
    ok &= testConfigRepositoryPrefersPrimaryPath();
    ok &= testConfigRepositoryUsesLegacyFallback();
    ok &= testConfigRepositoryMissingFileFails();
    ok &= testPositionerCalibrationUsesFixedDefaultPath();
    ok &= testWorkpieceProfilesLoadAndResolveTeachFiles();
    ok &= testWorkpieceProfilesLoadStartModeSpecificOnlineJobs();
    ok &= testWorkpieceProfilesRejectInvalidRows();

    if (!ok) {
        return 1;
    }

    std::cout << "PASS: sensor calibration parser and config repository tests\n";
    return 0;
}
