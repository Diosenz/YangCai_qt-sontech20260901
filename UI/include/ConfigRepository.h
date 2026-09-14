/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        ConfigRepository.h
 * @brief       运行时配置仓库
 * @details     封装工件配置、传感器标定和变位机标定文件读取入口。
 ******************************************************************************/
#pragma once

#include <QString>
#include <QVector>

#include "AppPaths.h"
#include "geometry/PositionerKinematics.h"
#include "SensorCalibrationParser.h"

namespace i2r_rlc::ui {

/**
 * @brief 旧式“当前工件”下拉框的一条配置。
 *
 * 所有轨迹名均使用不带 `_W/_T` 后缀的基名；运行时按流程自动补后缀。
 * JBI 名对应旧工程 `WPOnlineJBIList / WPOfflineJBIList /
 * WPOfflineCalJBIList`，用于通过 J 协议启动控制柜程序。
 * `onlineJbi` 是 InitType 起始方式使用的在线程序；另外两个字段分别供
 * 全局修正矩阵起始和 IBVS 起始方式使用。旧 8 列配置会自动把三者填为同一值。
 * 在线纠偏和离线纠偏使用不同轨迹基名，兼容旧 `COnlineCor::cfilename`
 * 与 `COffLineCor::sfilename` 分别配置的行为。
 */
struct WorkpieceProfile {
    QString id;
    QString displayName;
    QString weldmentModel;
    QString offlineLocateJbi;
    QString onlineJbi;
    QString onlineGlobalCorrectionJbi;
    QString onlineIbvsStartJbi;
    QString offlineCorrectionJbi;
    QString onlineTeachBaseName;
    QString offlineCorrectionTeachBaseName;
};

/**
 * @brief 工件配置加载结果。
 */
struct WorkpieceProfileLoadResult {
    bool ok = false;
    QVector<WorkpieceProfile> profiles;
    QString filePath;
    QString message;
};

/**
 * @brief 配置仓库。
 *
 * 负责通过 `AppPaths` 查找配置文件，并把解析工作委托给 UI 模块内的轻量解析器。
 * UI 和业务流程不应直接拼接路径或解析 `SENSORPARA.txt`。
 */
class ConfigRepository {
public:
    explicit ConfigRepository(AppPaths paths);

    /// @brief 返回运行时数据根目录说明。
    QString runtimeRootText() const;

    /// @brief 返回标定文件候选位置说明。
    QString calibrationSearchText() const;

    /**
     * @brief 确保运行时目录存在。
     */
    bool ensureRuntimeDirectories(QString* errorMessage = nullptr) const;

    /**
     * @brief 查找传感器标定文件。
     *
     * 优先查找 `config/calibration/SENSORPARA.txt`，不存在时再查找
     * `data/legacy/Robotleadingdata/BASIC/SENSORPARA.txt`。
     */
    QString findSensorCalibrationFile() const;

    /**
     * @brief 加载并解析传感器标定文件。
     *
     * 返回结构化结果，不弹窗，不访问旧工程目录。
     */
    SensorCalibrationParseResult loadSensorCalibration() const;

    /**
     * @brief 从固定路径 `config/calibration/RBCALIB.DAT` 加载变位机标定。
     */
    vision::PositionerCalibration loadPositionerCalibration() const;

    /**
     * @brief 加载旧式“当前工件”下拉框配置。
     */
    WorkpieceProfileLoadResult loadWorkpieceProfiles() const;

    /// @brief 根据在线纠偏示教基名生成 `_W.txt` 路径。
    QString workpieceOnlineWorkPathFile(const WorkpieceProfile& profile) const;

    /// @brief 根据在线纠偏示教基名生成 `_T.txt` 路径。
    QString workpieceOnlineTeachPoseFile(const WorkpieceProfile& profile) const;

    /// @brief 根据离线纠偏示教基名生成 `_W.txt` 路径。
    QString workpieceOfflineCorrectionWorkPathFile(const WorkpieceProfile& profile) const;

    /// @brief 根据离线纠偏示教基名生成 `_T.txt` 路径。
    QString workpieceOfflineCorrectionTeachPoseFile(const WorkpieceProfile& profile) const;

    /// @brief 获取路径管理对象。
    const AppPaths& paths() const;

private:
    AppPaths m_paths;
    SensorCalibrationParser m_sensorCalibrationParser;
};

} // namespace i2r_rlc::ui
