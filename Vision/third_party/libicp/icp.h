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
 * @file icp.h
 * @brief libicp 基类接口。
 * @details 该文件来自旧工程迁入的 Andreas Geiger libicp。当前工程仅补充
 *          许可证可见性、Doxygen 注释和 include 顺序，不改变 ICP 算法、宏值或 ABI。
 */

#ifndef I2R_RLC_VISION_ICP_LIBICP_ICP_H
#define I2R_RLC_VISION_ICP_LIBICP_ICP_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
#include <vector>

#include "kdtree.h"
#include "matrix.h"

/**
 * @brief libicp 迭代配准基类。
 *
 * 旧库约定模型点集为 `M`，模板/源点集为 `T`，拟合关系为 `M = R * T + t`。
 * 坐标单位由调用方保证一致，新 Vision 封装中使用 mm。
 */
class CIcp {

public:

  /**
   * @brief 构造 ICP 模型点云和 kd-tree。
   * @param M 模型点连续数组，布局为 `[x,y,(z)]`，单位由调用方保证一致。
   * @param M_num 模型点数量，至少 5 个。
   * @param m_dim 点维度，只支持 2 或 3。
   * @param m_maxIter 最大迭代次数。
   * @param m_minDelta 收敛阈值，表示旋转/平移参数变化最小量。
   */
  CIcp (double *M,const int32_t M_num,const int32_t m_dim,int32_t m_maxIter,double m_minDelta);
  
  /// @brief 释放内部 kd-tree 和模型缓存。
  virtual ~CIcp ();
  
  /**
   * @brief 设置最大迭代次数。
   * @param val 最大迭代次数。
   */
  void setMaxIterations   (int32_t val) { m_maxIter  = val; }
  
  /**
   * @brief 设置旋转/平移参数变化收敛阈值。
   * @param val 最小参数变化量。
   */
  void setMinDeltaParam   (double  val) { m_minDelta = val; }
  
  /**
   * @brief 点到点或点到面派生类通用刚体拟合入口。
   * @param T 模板/源点连续数组，布局为 `[x,y,(z)]`。
   * @param T_num 模板点数量，至少 5 个。
   * @param R 输入初始旋转，输出最终旋转。
   * @param t 输入初始平移，输出最终平移，单位与点坐标一致。
   * @param indist 内点距离阈值；小于等于 0 时使用全部点。
   */
  void fit(double *T,const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t,const double indist);
  //void fitRemoveR(double * T, const int32_t T_num, RMatrix::Matrix & R, RMatrix::Matrix & t, const double indist, const int m_keepRotation);

  /**
   * @brief 带尺度矩阵估计的点到点拟合入口。
   * @param T 模板/源点连续数组。
   * @param T_num 模板点数量。
   * @param R 输入输出旋转矩阵。
   * @param t 输入输出平移向量。
   * @param S 输出尺度矩阵。
   * @param indist 内点距离阈值。
   */
  void fit_scale(double * T, const int32_t T_num, RMatrix::Matrix & R, RMatrix::Matrix & t, RMatrix::Matrix & S, const double indist);

  /**
   * @brief 仿射 ICP 拟合并输出距离统计。
   * @param T 模板/源点连续数组。
   * @param T_num 模板点数量。
   * @param R 输入输出仿射线性部分。
   * @param t 输入输出平移向量。
   * @param indist 内点距离阈值。
   * @param MaxDist 输出最大残差距离。
   * @param MinDist 输出最小残差距离。
   * @param MeanDist 输出平均残差距离。
   * @param MsrDist 输出均方根残差距离。
   */
  void fitAffine(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const double indist, double &MaxDist, double &MinDist, double &MeanDist, double &MsrDist);

  /**
   * @brief 双向距离约束仿射 ICP 拟合。
   * @param T 模板/源点连续数组。
   * @param T_num 模板点数量。
   * @param R 输入输出仿射线性部分。
   * @param t 输入输出平移向量。
   * @param indist 内点距离阈值。
   * @param MaxDist 输出最大残差距离。
   * @param MinDist 输出最小残差距离。
   * @param MeanDist 输出平均残差距离。
   * @param MsrDist 输出均方根残差距离。
   */
  void fitAffineBDD(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const double indist, double &MaxDist, double &MinDist, double &MeanDist, double &MsrDist);

  /**
   * @brief 双向距离约束刚体 ICP 拟合。
   * @param T 模板/源点连续数组。
   * @param T_num 模板点数量。
   * @param R 输入输出旋转矩阵。
   * @param t 输入输出平移向量。
   * @param indist 内点距离阈值。
   */
  void fitBDD(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const double indist);
private:
  
  /// @brief 执行刚体 ICP 迭代，直到达到最大迭代数或参数变化收敛。
  void fitIterate(double *T,const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t,const std::vector<int32_t> &active);

  //void fitIterateRemoveR(double * T, const int32_t T_num, RMatrix::Matrix & R, RMatrix::Matrix & t, const std::vector<int32_t>& active, const int m_keepRotation);

  void fitIterate_scale(double * T, const int32_t T_num, RMatrix::Matrix & R, RMatrix::Matrix & t, RMatrix::Matrix & S, const std::vector<int32_t>& active);
  
  /// @brief 派生类实现单步拟合，并返回本轮参数变化量。
  virtual double               fitStep(double *T,const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t,const std::vector<int32_t> &active) = 0;

  //virtual double               fitStepRemoveR(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const std::vector<int32_t> &active,const int& m_keepRotation) = 0;

  virtual double    fitStep_scale(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, RMatrix::Matrix&S, const std::vector<int32_t> &active) = 0;

  virtual std::vector<int32_t> getInliers(double *T,const int32_t T_num,const 	RMatrix::Matrix &R,const 	RMatrix::Matrix &t,const double indist) = 0;
  /// @brief 执行仿射 ICP 迭代并更新距离统计。
  void fitIterateAffine(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, std::vector<int32_t> &active, double &MaxDist, double &MinDist, double &MeanDist, double &MsrDist);

  /// @brief 执行双向距离约束仿射 ICP 迭代并更新距离统计。
  void fitIterateAffineBDD(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, std::vector<int32_t> &active, double &MaxDist, double &MinDist, double &MeanDist, double &MsrDist);

  /// @brief 执行双向距离约束刚体 ICP 迭代。
  void fitIterateBDD(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const std::vector<int32_t> &active);

  /// @brief 派生类实现单步仿射拟合。
  virtual double               fitStepAffine(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const std::vector<int32_t> &active) = 0;

  /// @brief 派生类实现单步双向距离仿射拟合。
  virtual double               fitStepAffineBDD(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const std::vector<int32_t> &active) = 0;

  /// @brief 派生类根据双向距离约束评估匹配残差统计。
  virtual void               EvaluateMatchResultBDD(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const std::vector<int32_t> &active, double &MaxDist, double &MinDist, double &MeanDist, double &MsrDist) = 0;

  /// @brief 派生类实现单步双向距离刚体拟合。
  virtual double               fitStepBDD(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const std::vector<int32_t> &active) = 0;
  
protected:
  
  // kd tree of model points
  kdtree::KDTree*     m_tree;
  kdtree::KDTreeArray m_data;
  
  int32_t m_dim;       // dimensionality of model + template data (2 or 3)
  int32_t m_maxIter;  // max number of iterations
  int32_t m_currentIter;
  double  m_minDelta; // min parameter delta
};

#endif // I2R_RLC_VISION_ICP_LIBICP_ICP_H


