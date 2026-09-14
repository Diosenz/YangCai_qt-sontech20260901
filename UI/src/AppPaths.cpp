/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        AppPaths.cpp
 * @brief       运行时路径管理实现
 * @details     从应用目录或显式根目录推导配置、数据和输出路径。
 ******************************************************************************/
#include <utility>

#include <QCoreApplication>
#include <QFileInfo>
#include <QList>

#include "AppPaths.h"

namespace i2r_rlc::ui {

AppPaths AppPaths::fromDefaultLocation()
{
    return AppPaths(discoverRuntimeRootFromApplicationDir());
}

AppPaths AppPaths::fromRuntimeRoot(const QString& runtimeRootPath)
{
    return AppPaths(QDir(runtimeRootPath));
}

AppPaths::AppPaths(QDir runtimeRoot)
    : m_runtimeRoot(std::move(runtimeRoot))
{
}

QDir AppPaths::runtimeRootDir() const
{
    return m_runtimeRoot;
}

QDir AppPaths::configDir() const
{
    return childDir(QStringLiteral("config"));
}

QDir AppPaths::appConfigDir() const
{
    return childDir(QStringLiteral("config/app"));
}

QDir AppPaths::cameraConfigDir() const
{
    return childDir(QStringLiteral("config/camera"));
}

QDir AppPaths::robotConfigDir() const
{
    return childDir(QStringLiteral("config/robot"));
}

QDir AppPaths::digitalTwinConfigDir() const
{
    return childDir(QStringLiteral("config/digital_twin"));
}

QDir AppPaths::calibrationDir() const
{
    return childDir(QStringLiteral("config/calibration"));
}

QDir AppPaths::calibrationDataDir() const
{
    return childDir(QStringLiteral("Data/CALID"));
}

QDir AppPaths::workpiecesDir() const
{
    return childDir(QStringLiteral("config/workpieces"));
}

QString AppPaths::workpieceSelectionFilePath() const
{
    return workpiecesDir().filePath(QStringLiteral("workpiece_selection.txt"));
}

QDir AppPaths::workflowsDir() const
{
    return childDir(QStringLiteral("config/workflows"));
}

QDir AppPaths::jobsDir() const
{
    return childDir(QStringLiteral("jobs"));
}

QDir AppPaths::modelsDir() const
{
    return childDir(QStringLiteral("models"));
}

QDir AppPaths::weldmentModelsDir() const
{
    return childDir(QStringLiteral("models/weldment"));
}

QDir AppPaths::ibvsModelsDir() const
{
    return childDir(QStringLiteral("models/ibvs"));
}

QDir AppPaths::legacyDataDir() const
{
    return childDir(QStringLiteral("data/legacy"));
}

QDir AppPaths::workpieceTrajectoryDir() const
{
    return childDir(QStringLiteral("data/WorkPiece"));
}

QDir AppPaths::teachTrajectoryCaptureDir() const
{
    return childDir(QStringLiteral("data/trajectory_capture"));
}

QDir AppPaths::replayDataDir() const
{
    return childDir(QStringLiteral("data/replay"));
}

QDir AppPaths::logsDir() const
{
    return childDir(QStringLiteral("logs"));
}

QDir AppPaths::outputDir() const
{
    return childDir(QStringLiteral("output"));
}

QDir AppPaths::outputImagesDir() const
{
    return childDir(QStringLiteral("output/images"));
}

QDir AppPaths::outputPointCloudsDir() const
{
    return childDir(QStringLiteral("output/point_clouds"));
}

QDir AppPaths::outputTracesDir() const
{
    return childDir(QStringLiteral("output/traces"));
}

QDir AppPaths::outputReportsDir() const
{
    return childDir(QStringLiteral("output/reports"));
}

QString AppPaths::sensorCalibrationFilePath() const
{
    return calibrationDir().filePath(QStringLiteral("SENSORPARA.txt"));
}

QString AppPaths::rbcalibFilePath() const
{
    return calibrationDir().filePath(QStringLiteral("RBCALIB.DAT"));
}

QString AppPaths::calibPoseTemplateFilePath() const
{
    return calibrationDir().filePath(QStringLiteral("CalibPose.txt"));
}

QString AppPaths::calibPoseCorrectedFilePath() const
{
    return childDir(QStringLiteral("Data/CALID/4 TWO_STEP"))
        .filePath(QStringLiteral("CalibPose_new.txt"));
}

QString AppPaths::legacySensorCalibrationFilePath() const
{
    return childDir(QStringLiteral("data/legacy/Robotleadingdata/BASIC")).filePath(QStringLiteral("SENSORPARA.txt"));
}

bool AppPaths::ensureRuntimeDirectories(QString* errorMessage) const
{
    const QList<QDir> dirs = {
        configDir(),
        appConfigDir(),
        cameraConfigDir(),
        robotConfigDir(),
        digitalTwinConfigDir(),
        calibrationDir(),
        calibrationDataDir(),
        childDir(QStringLiteral("Data/CALID/4 TWO_STEP")),
        workpiecesDir(),
        workflowsDir(),
        jobsDir(),
        modelsDir(),
        weldmentModelsDir(),
        ibvsModelsDir(),
        legacyDataDir(),
        childDir(QStringLiteral("data/legacy/Robotleadingdata/BASIC")),
        workpieceTrajectoryDir(),
        teachTrajectoryCaptureDir(),
        replayDataDir(),
        logsDir(),
        outputDir(),
        outputImagesDir(),
        outputPointCloudsDir(),
        outputTracesDir(),
        outputReportsDir()
    };

    for (const auto& dir : dirs) {
        if (!ensureDir(dir, errorMessage)) {
            return false;
        }
    }

    return true;
}

QDir AppPaths::runtimeRoot() const
{
    return runtimeRootDir();
}

QDir AppPaths::calibrationConfigDir() const
{
    return calibrationDir();
}

QString AppPaths::legacySensorParaPath() const
{
    return legacySensorCalibrationFilePath();
}

QString AppPaths::sensorParaPath() const
{
    return sensorCalibrationFilePath();
}

QDir AppPaths::discoverRuntimeRootFromApplicationDir()
{
    QDir dir(QCoreApplication::applicationDirPath());

    for (int depth = 0; depth < 8; ++depth) {
        if (QFileInfo::exists(dir.filePath(QStringLiteral("config/calibration/SENSORPARA.txt")))
            || QFileInfo::exists(dir.filePath(QStringLiteral("CMakeLists.txt")))) {
            return dir;
        }

        if (!dir.cdUp()) {
            break;
        }
    }

    return QDir(QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("runtime_data")));
}

QDir AppPaths::childDir(const QString& relativePath) const
{
    return QDir(m_runtimeRoot.filePath(relativePath));
}

bool AppPaths::ensureDir(const QDir& dir, QString* errorMessage) const
{
    if (dir.exists()) {
        return true;
    }

    if (QDir().mkpath(dir.absolutePath())) {
        return true;
    }

    if (errorMessage) {
        *errorMessage = QStringLiteral("无法创建目录：%1").arg(dir.absolutePath());
    }
    return false;
}

} // namespace i2r_rlc::ui
