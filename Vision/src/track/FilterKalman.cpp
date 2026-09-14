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
 * @file        FilterKalman.cpp
 * @brief       Kalman 滤波器迁移实现
 * @details     复刻旧 Filter_Kalman 的矩阵初始化和 Kalfilter 更新公式。
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
#include "track/FilterKalman.h"

namespace i2r_rlc::vision {
namespace {

/// @brief 默认 6 维匀速模型状态转移矩阵，状态顺序为 `[x,y,z,vx,vy,vz]`。
constexpr double kDefaultTransition[6 * 6] = {
    1, 0, 0, 1, 0, 0,
    0, 1, 0, 0, 1, 0,
    0, 0, 1, 0, 0, 1,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1
};

/// @brief 默认 3 维位置观测矩阵，只观测 `[x,y,z]`。
constexpr double kDefaultObserve[3 * 6] = {
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0
};

} // namespace

CFilterKalman::CFilterKalman()
{
    m_A = RMatrix::Matrix(6, 6, kDefaultTransition);
    m_P = RMatrix::Matrix(6, 6);
    m_H = RMatrix::Matrix(3, 6, kDefaultObserve);
    m_K = RMatrix::Matrix(6, 6);
    m_Q = RMatrix::Matrix(6, 6);
    m_R = RMatrix::Matrix(3, 3);
    m_B = RMatrix::Matrix(6, 6);
    m_u = RMatrix::Matrix(6, 1);
}

CFilterKalman::CFilterKalman(int dimension)
{
    m_A = RMatrix::Matrix(dimension, dimension);
    m_P = RMatrix::Matrix(dimension, dimension);
    m_H = RMatrix::Matrix(dimension, dimension);
    m_K = RMatrix::Matrix(dimension, dimension);
    m_Q = RMatrix::Matrix(dimension, dimension);
    m_R = RMatrix::Matrix(dimension, dimension);
    m_B = RMatrix::Matrix(dimension, dimension);
    m_u = RMatrix::Matrix(dimension, 1);
}

CFilterKalman CFilterKalman::Kalfilter(CFilterKalman filter)
{
    filter.m_Q.eye();
    filter.m_R.eye();
    filter.m_Q = filter.m_Q * m_cQ;
    filter.m_R = filter.m_R * m_cR;
    RMatrix::Matrix predict = filter.m_A * filter.m_X + filter.m_B * filter.m_u;
    filter.m_P = filter.m_A * filter.m_P * (~filter.m_A) + filter.m_Q;

    RMatrix::Matrix temp = filter.m_H * filter.m_P * (~filter.m_H) + filter.m_R;
    temp.inv();
    filter.m_K = filter.m_P * (~filter.m_H) * temp;
    filter.m_X = predict + filter.m_K * (filter.m_Y - filter.m_H * predict);
    filter.m_P = filter.m_P - filter.m_K * filter.m_H * filter.m_P;
    return filter;
}

void CFilterKalman::Kalfilter()
{
    m_Q.eye();
    m_R.eye();
    m_Q = m_Q * m_cQ;
    m_R = m_R * m_cR;
    RMatrix::Matrix predict = m_A * m_X + m_B * m_u;
    m_P = m_A * m_P * (~m_A) + m_Q;

    RMatrix::Matrix temp = m_H * m_P * (~m_H) + m_R;
    temp.inv();
    m_K = m_P * (~m_H) * temp;
    m_X = predict + m_K * (m_Y - m_H * predict);
    m_P = m_P - m_K * m_H * m_P;
}

} // namespace i2r_rlc::vision
