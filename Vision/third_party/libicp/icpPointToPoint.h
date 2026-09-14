/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

Authors: Andreas Geiger

libicp is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libicp is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libicp; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

/**
 * @file icpPointToPoint.h
 * @brief libicp 点到点配准派生类。
 * @details 保留旧工程迁入的 point-to-point、scale、affine 和 BDD 分支接口，
 *          仅补充文档和 include 规范，不改变算法行为。
 */

#ifndef I2R_RLC_VISION_ICP_LIBICP_ICP_POINT_TO_POINT_H
#define I2R_RLC_VISION_ICP_LIBICP_ICP_POINT_TO_POINT_H

#include "icp.h"

/**
 * @brief 点到点 ICP 实现。
 *
 * 该类通过 kd-tree 最近邻建立模板点到模型点的对应关系，并由私有 fitStep
 * 系列函数求解每次迭代的 R/t/scale/affine 更新。
 */
class CIcpPointToPoint : public CIcp {

public:

  /**
   * @brief 构造点到点 ICP。
   * @param M 模型点连续数组，布局为 `[x,y,(z)]`。
   * @param M_num 模型点数量，至少 5 个。
   * @param m_dim 点维度，只支持 2 或 3。
   * @param m_maxIter 最大迭代次数。
   * @param m_minDelta 参数收敛阈值。
   */
  CIcpPointToPoint (double *M,const int32_t M_num,const int32_t m_dim,int32_t m_maxIter,double m_minDelta) : CIcp(M,M_num,m_dim,m_maxIter,m_minDelta) {}
  /// @brief 默认析构，资源由基类管理。
  virtual ~CIcpPointToPoint () {}
  /// @brief 保持旋转约束开关，沿用旧工程控制逻辑。
  void keepRotation();
  /// @brief 保持连接约束开关，沿用旧工程控制逻辑。
  void keepConnect();
  /// @brief 移除失败匹配点开关，沿用旧工程控制逻辑。
  void keepRemoveFailPoint();
private:

  /// @brief 单步点到点刚体最小二乘拟合。
  double fitStep (double *T,const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t,const std::vector<int32_t> &active);
  //double fitStepRemoveR(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const std::vector<int32_t> &active,const int &m_keepRotation);
  /// @brief 基于当前 R/t 和内点距离筛选有效模板点索引。
  std::vector<int32_t> getInliers (double *T,const int32_t T_num,const 	RMatrix::Matrix &R,const 	RMatrix::Matrix &t,const double indist);
  /// @brief 单步仿射拟合，输出线性部分到 R、平移到 t。
  double fitStepAffine(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const std::vector<int32_t> &active);

  /// @brief 单步双向距离约束仿射拟合。
  double fitStepAffineBDD(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const std::vector<int32_t> &active);

  /// @brief 计算双向距离约束匹配的最大/最小/平均/均方根残差。
  void EvaluateMatchResultBDD(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const std::vector<int32_t> &active, double &MaxDist, double &MinDist, double &MeanDist, double &MsrDist);

  /// @brief 单步双向距离约束刚体拟合。
  double fitStepBDD(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const std::vector<int32_t> &active);
  /// @brief 单步带尺度点到点拟合，保留 20220416 旧工程扩展。
  double fitStep_scale(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, RMatrix::Matrix&S, const std::vector<int32_t> &active);
  /// @brief 是否保持旋转，0 表示关闭。
  int m_keepRotation=0;
  /// @brief 是否启用连接约束，0 表示关闭。
  int m_keepConnectFlag = 0;
  /// @brief 是否移除失败匹配点，0 表示关闭。
  int m_keepRemoveFailPointFlag = 0;
};

#endif // I2R_RLC_VISION_ICP_LIBICP_ICP_POINT_TO_POINT_H


