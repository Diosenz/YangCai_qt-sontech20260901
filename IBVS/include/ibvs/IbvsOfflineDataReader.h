/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsOfflineDataReader.h
 * @brief     IBVS 文本数据读取入口。
 * @details   本文件定义特征、ROI、位姿、速度和初始雅可比的序列化读取接口。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 8 日
 */

#ifndef I2R_RLC_IBVS_IBVS_OFFLINE_DATA_READER_H
#define I2R_RLC_IBVS_IBVS_OFFLINE_DATA_READER_H

#include <string>

#include "ibvs/IbvsTypes.h"

namespace i2r_rlc::ibvs {

/**
 * @brief 带结构化错误状态的数据序列。
 */
template<class T>
struct IbvsDataSeries
{
    IbvsError error;
    std::vector<T> values;
};

/**
 * @brief 初始雅可比文本文件读取结果。
 */
struct IbvsJacobianFileResult
{
    IbvsError error;
    IbvsJacobianMatrix jacobian = IbvsJacobianMatrix::Zero();
};

/**
 * @brief 读取 IBVS 文本数据序列。
 */
class IbvsOfflineDataReader
{
public:
    /**
     * @brief 读取图像特征坐标序列。
     */
    static IbvsDataSeries<IbvsFeatureVector> readFeatureCoordinates(
        const std::string& path);

    /**
     * @brief 读取机器人平移增量序列，单位为 m。
     */
    static IbvsDataSeries<Eigen::Vector3d> readRobotTranslations(
        const std::string& path);

    /**
     * @brief 读取相机 6 维位姿序列。
     */
    static IbvsDataSeries<IbvsPoseVector> readCameraPoses(
        const std::string& path);

    /**
     * @brief 读取 4x3 初始雅可比矩阵。
     */
    static IbvsJacobianFileResult readInitialJacobian(
        const std::string& path);

    /**
     * @brief 读取 ROI 序列，自动丢弃无效 ROI。
     */
    static IbvsDataSeries<PixelRoi> readRois(const std::string& path);

    /**
     * @brief 读取相机平移速度序列，单位为 m/s。
     */
    static IbvsDataSeries<IbvsVelocityVector> readVelocitySeries(
        const std::string& path);
};

} // namespace i2r_rlc::ibvs

#endif // I2R_RLC_IBVS_IBVS_OFFLINE_DATA_READER_H
