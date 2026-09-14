/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        AppPaths.h
 * @brief       运行时路径管理
 * @details     集中管理配置、数据、日志和输出目录，避免业务代码拼接路径。
 ******************************************************************************/
#pragma once

#include <QDir>
#include <QString>

namespace i2r_rlc::ui {

/**
 * @brief 运行时路径集中管理。
 *
 * 本类不依赖当前工作目录。默认会从应用程序所在目录向上查找包含运行时配置
 * 的目录；也可以由用户显式传入运行时根目录。输入配置目录和运行输出目录
 * 都通过这里统一获取，避免业务代码拼接路径。
 */
class AppPaths {
public:
    /**
     * @brief 基于应用程序目录创建路径集合。
     *
     * 如果应用程序位于构建目录下，会向上查找包含 `CMakeLists.txt` 或 `config`
     * 的项目目录；找不到时使用应用程序目录下的 `runtime_data`。
     */
    static AppPaths fromDefaultLocation();

    /**
     * @brief 基于显式运行时根目录创建路径集合。
     */
    static AppPaths fromRuntimeRoot(const QString& runtimeRootPath);

    /**
     * @brief 使用显式运行时根目录创建路径集合。
     */
    explicit AppPaths(QDir runtimeRoot);

    /// @brief 运行时数据根目录。
    QDir runtimeRootDir() const;

    /// @brief 输入配置根目录：`config/`。
    QDir configDir() const;

    /// @brief 输入配置目录：`config/app/`。
    QDir appConfigDir() const;

    /// @brief 输入配置目录：`config/camera/`。
    QDir cameraConfigDir() const;

    /// @brief 输入配置目录：`config/robot/`。
    QDir robotConfigDir() const;

    /// @brief 输入配置目录：`config/digital_twin/`。
    QDir digitalTwinConfigDir() const;

    /// @brief 输入配置目录：`config/calibration/`。
    QDir calibrationDir() const;

    /// @brief 运行时标定输出目录：`Data/CALID/`。
    QDir calibrationDataDir() const;

    /// @brief 输入配置目录：`config/workpieces/`。
    QDir workpiecesDir() const;

    /// @brief 工件选择配置文件路径：`config/workpieces/workpiece_selection.txt`。
    QString workpieceSelectionFilePath() const;

    /// @brief 输入配置目录：`config/workflows/`。
    QDir workflowsDir() const;

    /// @brief 输入配置目录：`jobs/`。
    QDir jobsDir() const;

    /// @brief 输入资产根目录：`models/`。
    QDir modelsDir() const;

    /// @brief 输入资产目录：`models/weldment/`。
    QDir weldmentModelsDir() const;

    /// @brief 输入资产目录：`models/ibvs/`。
    QDir ibvsModelsDir() const;

    /// @brief legacy 兼容输入目录：`data/legacy/`。
    QDir legacyDataDir() const;

    /// @brief 工件轨迹目录：`data/WorkPiece/`。
    QDir workpieceTrajectoryDir() const;

    /// @brief 示教轨迹临时记录目录：`data/trajectory_capture/`。
    QDir teachTrajectoryCaptureDir() const;

    /// @brief 离线回放输入目录：`data/replay/`。
    QDir replayDataDir() const;

    /// @brief 运行输出目录：`logs/`。
    QDir logsDir() const;

    /// @brief 运行输出根目录：`output/`。
    QDir outputDir() const;

    /// @brief 运行输出目录：`output/images/`。
    QDir outputImagesDir() const;

    /// @brief 运行输出目录：`output/point_clouds/`。
    QDir outputPointCloudsDir() const;

    /// @brief 运行输出目录：`output/traces/`。
    QDir outputTracesDir() const;

    /// @brief 运行输出目录：`output/reports/`。
    QDir outputReportsDir() const;

    /// @brief 新版传感器标定文件路径：`config/calibration/SENSORPARA.txt`。
    QString sensorCalibrationFilePath() const;

    /// @brief 变位机标定文件路径：`config/calibration/RBCALIB.DAT`。
    QString rbcalibFilePath() const;

    /// @brief 自动标定模板位姿路径：`config/calibration/CalibPose.txt`。
    QString calibPoseTemplateFilePath() const;

    /// @brief 自动标定修正位姿输出路径：`Data/CALID/4 TWO_STEP/CalibPose_new.txt`。
    QString calibPoseCorrectedFilePath() const;

    /// @brief legacy 兼容传感器标定文件路径：`data/legacy/Robotleadingdata/BASIC/SENSORPARA.txt`。
    QString legacySensorCalibrationFilePath() const;

    /**
     * @brief 创建必要的运行时目录。
     *
     * 会创建输入配置目录和运行输出目录。该函数不访问旧工程目录。
     */
    bool ensureRuntimeDirectories(QString* errorMessage = nullptr) const;

    /**
     * @brief 兼容旧接口：运行时数据根目录。
     */
    QDir runtimeRoot() const;

    /**
     * @brief 兼容旧接口：标定配置目录。
     */
    QDir calibrationConfigDir() const;

    /**
     * @brief 兼容旧接口：legacy SENSORPARA.txt 路径。
     */
    QString legacySensorParaPath() const;

    /**
     * @brief 兼容旧接口：新版 SENSORPARA.txt 路径。
     */
    QString sensorParaPath() const;

private:
    static QDir discoverRuntimeRootFromApplicationDir();

    QDir childDir(const QString& relativePath) const;
    bool ensureDir(const QDir& dir, QString* errorMessage) const;

    QDir m_runtimeRoot;
};

} // namespace i2r_rlc::ui
