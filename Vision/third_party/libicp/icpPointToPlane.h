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
 * @file icpPointToPlane.h
 * @brief libicp 点到面配准派生类。
 * @details 该文件保留旧工程迁入的 point-to-plane ICP 和模型法向估计实现。
 *          当前工程仅补充文档和 include 规范，不改变算法行为。
 */

#ifndef I2R_RLC_VISION_ICP_LIBICP_ICP_POINT_TO_PLANE_H
#define I2R_RLC_VISION_ICP_LIBICP_ICP_POINT_TO_PLANE_H

#include "icp.h"

/**
 * @brief 点到面 ICP 实现。
 *
 * 构造时从模型点云估计法向，迭代时使用模板点到模型切平面的距离建立线性方程。
 */
class CIcpPointToPlane : public CIcp {

public:
  
  /**
   * @brief 构造点到面 ICP 并预计算模型法向。
   * @param M 模型点连续数组，布局为 `[x,y,(z)]`。
   * @param M_num 模型点数量，至少 5 个。
   * @param m_dim 点维度，只支持 2 或 3。
   * @param m_maxIter 最大迭代次数。
   * @param m_minDelta 参数收敛阈值。
   * @param num_neighbors 法向估计使用的近邻数量，单位为点数。
   * @param flatness 法向估计的平坦度阈值。
   */
  CIcpPointToPlane (double *M,const int32_t M_num,const int32_t m_dim,int32_t m_maxIter,double m_minDelta,const int32_t num_neighbors=10,const double flatness=5.0) : CIcp(M,M_num,m_dim,m_maxIter,m_minDelta) {
    m_modelNormals = computeNormals(num_neighbors,flatness);
  }

  /// @brief 释放预计算的模型法向缓冲区。
  virtual ~CIcpPointToPlane () {
    free(m_modelNormals);
  }

private:


	/// @brief 单步点到面最小二乘拟合。
	double fitStep(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const std::vector<int32_t> &active);
	
	/// @brief 基于当前 R/t 和内点距离筛选有效模板点索引。
	std::vector<int32_t> getInliers(double *T, const int32_t T_num, const RMatrix::Matrix &R, const RMatrix::Matrix &t, const double indist);

    /// @brief 点到面分支不估计尺度，兼容基类虚函数接口。
    double fitStep_scale(
        double *T,
        const int32_t T_num,
        RMatrix::Matrix &R,
        RMatrix::Matrix &t,
        RMatrix::Matrix &S,
        const std::vector<int32_t> &active)
    {
        (void)S;
        return fitStep(T, T_num, R, t, active);
    }

    /// @brief 点到面分支不支持真实仿射，兼容基类虚函数接口。
    double fitStepAffine(
        double *T,
        const int32_t T_num,
        RMatrix::Matrix &R,
        RMatrix::Matrix &t,
        const std::vector<int32_t> &active)
    {
        return fitStep(T, T_num, R, t, active);
    }

    /// @brief 点到面分支不支持真实 affineBDD，兼容基类虚函数接口。
    double fitStepAffineBDD(
        double *T,
        const int32_t T_num,
        RMatrix::Matrix &R,
        RMatrix::Matrix &t,
        const std::vector<int32_t> &active)
    {
        return fitStep(T, T_num, R, t, active);
    }

    /// @brief 用点到面单步误差填充 BDD 统计输出，兼容基类虚函数接口。
    void EvaluateMatchResultBDD(
        double *T,
        const int32_t T_num,
        RMatrix::Matrix &R,
        RMatrix::Matrix &t,
        const std::vector<int32_t> &active,
        double &MaxDist,
        double &MinDist,
        double &MeanDist,
        double &MsrDist)
    {
        const double delta = fitStep(T, T_num, R, t, active);
        MaxDist = delta;
        MinDist = delta;
        MeanDist = delta;
        MsrDist = delta;
    }

    /// @brief 点到面 BDD 入口复用点到面单步拟合。
    double fitStepBDD(
        double *T,
        const int32_t T_num,
        RMatrix::Matrix &R,
        RMatrix::Matrix &t,
        const std::vector<int32_t> &active)
    {
        return fitStep(T, T_num, R, t, active);
    }

	/// @brief 根据近邻点估计单个模型点法向。
	void computeNormal(const kdtree::KDTreeResultVector &neighbors, double *m_modelNormals, const double flatness);
	/// @brief 估计全部模型点法向，返回需由析构释放的连续缓冲区。
	double* computeNormals(const int32_t num_neighbors, const double flatness);

	/// @brief 模型点法向连续缓冲区，布局为 `[nx,ny,(nz)]`。
	double *m_modelNormals;
};

#endif // I2R_RLC_VISION_ICP_LIBICP_ICP_POINT_TO_PLANE_H


