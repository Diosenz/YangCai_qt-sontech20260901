/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsVelocitySolver.h
 * @brief     IBVS 平移速度求解器。
 * @details   本文件定义由图像特征误差和雅可比矩阵求解相机平移速度的接口。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 8 日
 */

#ifndef I2R_RLC_IBVS_IBVS_VELOCITY_SOLVER_H
#define I2R_RLC_IBVS_IBVS_VELOCITY_SOLVER_H

#include "ibvs/IbvsTypes.h"

namespace i2r_rlc::ibvs {

/**
 * @brief 使用 WLS 和 LM 阻尼伪逆求解 IBVS 平移速度。
 */
class IbvsVelocitySolver
{
public:
    IbvsVelocitySolver();
    explicit IbvsVelocitySolver(const IbvsControlParameters& parameters);

    /**
     * @brief 更新速度控制参数，并在之后的求解中立即使用新参数。
     */
    void setControlParameters(const IbvsControlParameters& parameters);

    /**
     * @brief 返回当前速度控制参数。
     */
    const IbvsControlParameters& controlParameters() const;

    /**
     * @brief 设置期望 4 维图像特征。
     */
    void setDesiredFeature(const IbvsFeatureVector& desiredFeature);

    /**
     * @brief 返回当前期望 4 维图像特征。
     */
    const IbvsFeatureVector& desiredFeature() const;

    /**
     * @brief 根据当前 4 维图像特征和 4x3 雅可比求解 3 维平移速度。
     *
     * 速度单位为 m/s；输入特征单位为 pixel。异常数值条件不会抛异常，
     * 而是返回零速度和结构化错误状态。
     */
    IbvsVelocitySolveResult computeCameraVelocity(
        const IbvsJacobianMatrix& jacobian,
        const IbvsFeatureVector& currentFeature) const;

    /**
     * @brief 计算当前特征相对期望特征的误差，单位为 pixel。
     */
    Eigen::Vector4d featureError(const IbvsFeatureVector& currentFeature) const;

    /**
     * @brief 返回图像特征误差的欧氏范数，单位为 pixel。
     */
    double errorNorm(const IbvsFeatureVector& currentFeature) const;

    /**
     * @brief 判断当前特征误差是否低于停止阈值。
     */
    bool isConverged(const IbvsFeatureVector& currentFeature) const;

private:
    IbvsControlParameters m_parameters;
};

} // namespace i2r_rlc::ibvs

#endif // I2R_RLC_IBVS_IBVS_VELOCITY_SOLVER_H
