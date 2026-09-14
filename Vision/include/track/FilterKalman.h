/*******************************************************************************
 *       ______   ______   _______         __                  __
 *      /      | /      \ /       \       /  |                /  |
 *      $$$$$$/ /$$$$$$  |$$$$$$$  |      $$ |        ______  $$ |____
 *        $$ |  $$____$$ |$$ |__$$ |      $$ |       /      \ $$      \
 *        $$ |   /    $$/ $$    $$<       $$ |       $$$$$$  |$$$$$$$  |
 *        $$ |  /$$$$$$/  $$$$$$$  |      $$ |       /    $$ |$$ |  $$ |
 *       _$$ |_ $$ |_____ $$ |  $$ |      $$ |_____ /$$$$$$$ |$$ |__$$ |
 *      / $$   |$$       |$$ |  $$ |      $$       |$$    $$ |$$    $$
 *      $$$$$$/ $$$$$$$$/ $$/   $$/       $$$$$$$$/  $$$$$$$/ $$$$$$$/
 *
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        FilterKalman.h
 * @brief       旧工程 Kalman 滤波器迁移接口
 * @details     使用 libicp 的 RMatrix::Matrix 复刻旧 Filter_Kalman 的矩阵更新语义。
 *
 * @version     1.1
 * @author      LT
 * @date        2026 年 6 月 6 日
 *
 * @par         修改日志
 * <table>
 * <tr><th>Version  <th>Author      <th>Date        <th>Description         </tr>
 * <tr><td>1.0      <td>LT          <td>260606      <td>创建文件            </tr>
 * <tr><td>1.1      <td>LT          <td>260606      <td>初步完成            </tr>
 * </table>
 *
 ******************************************************************************/
#ifndef I2R_RLC_VISION_TRACK_CFILTER_KALMAN_H
#define I2R_RLC_VISION_TRACK_CFILTER_KALMAN_H

#include "libicp/matrix.h"

namespace i2r_rlc::vision {

/**
 * @brief RMatrix 版 Kalman 滤波器。
 *
 * 矩阵字段保持可直接读写，便于 KCF 按旧工程流程设置 `X/Y/u/cR`。
 */
class CFilterKalman final {
public:
    /**
     * @brief 构造旧工程默认 6 维状态、3 维观测的 Kalman 滤波器。
     *
     * 默认状态按 `[x, y, z, vx, vy, vz]` 理解，位置单位由调用侧决定；
     * KCF 跟踪中使用 pixel。
     */
    CFilterKalman();

    /**
     * @brief 构造指定维度的方阵 Kalman 滤波器。
     * @param dimension 状态、观测、控制矩阵的维度，必须为正数。
     */
    explicit CFilterKalman(int dimension);

    /**
     * @brief 返回一次滤波后的副本，保留旧 `Kalfilter(filter)` 语义。
     * @param filter 输入滤波器副本，函数在副本上执行预测和更新。
     * @return 更新后的滤波器副本。
     */
    CFilterKalman Kalfilter(CFilterKalman filter);

    /// @brief 就地执行一次 Kalman 滤波。
    void Kalfilter();

    /// @brief 状态转移矩阵。
    RMatrix::Matrix m_A;
    /// @brief 状态协方差矩阵。
    RMatrix::Matrix m_P;
    /// @brief Kalman 增益矩阵。
    RMatrix::Matrix m_K;
    /// @brief 观测矩阵。
    RMatrix::Matrix m_H;
    /// @brief 过程噪声协方差矩阵。
    RMatrix::Matrix m_Q;
    /// @brief 观测噪声协方差矩阵。
    RMatrix::Matrix m_R;
    /// @brief 状态向量；KCF 场景中位置单位为 pixel。
    RMatrix::Matrix m_X;
    /// @brief 观测向量；KCF 场景中位置单位为 pixel。
    RMatrix::Matrix m_Y;
    /// @brief 控制矩阵。
    RMatrix::Matrix m_B;
    /// @brief 控制输入向量。
    RMatrix::Matrix m_u;

    /// @brief 过程噪声缩放系数。
    double m_cQ = 3e-2;
    /// @brief 当前观测噪声缩放系数，会随响应峰值动态调整。
    double m_cR = 1e-5;
    /// @brief 观测噪声缩放系数上限。
    double m_cRMaximum = 1e-3;
    /// @brief 观测噪声缩放系数下限。
    double m_cRMinimum = 5e-6;
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_TRACK_CFILTER_KALMAN_H
