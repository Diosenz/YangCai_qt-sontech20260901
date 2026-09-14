/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

Authors: Andreas Geiger

matrix is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

matrix is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
matrix; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA
*/

/**
 * @file matrix.h
 * @brief libicp 自带轻量矩阵类。
 * @details 该文件来自 Andreas Geiger libicp/matrix，实现旧 ICP 内核依赖的
 *          动态矩阵、线性方程、LU 和 SVD 运算。当前工程只补文档和 include
 *          顺序，不改变矩阵存储布局或公开字段。
 */

#ifndef I2R_RLC_VISION_ICP_LIBICP_MATRIX_H
#define I2R_RLC_VISION_ICP_LIBICP_MATRIX_H

#ifndef _MSC_VER
#include <stdint.h>
#else
typedef signed __int8     int8_t;
typedef signed __int16    int16_t;
typedef signed __int32    int32_t;
typedef signed __int64    int64_t;
typedef unsigned __int8   uint8_t;
typedef unsigned __int16  uint16_t;
typedef unsigned __int32  uint32_t;
typedef unsigned __int64  uint64_t;
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
#include <vector>

using namespace std;

/// @brief 旧矩阵打印接口使用的双换行流宏。
#define endll endl << endl // double end line definition

//typedef double double;      // double precision
//typedef float  FLOAT;    // single precision

namespace RMatrix
{

	/**
	 * @brief libicp 内部使用的动态 double 矩阵。
	 *
	 * 矩阵采用 `val[row][col]` 二级指针存储，公开字段是旧库 ABI 的一部分。
	 * 维度单位为元素个数；角度相关静态函数使用弧度。
	 */
	class Matrix {

	public:

		/// @brief 构造 0x0 空矩阵。
		Matrix();
		/// @brief 构造 `m x n` 零初始化矩阵。
		Matrix(const int32_t m, const int32_t n);
		/// @brief 从连续数组构造 `m x n` 矩阵，数组按行优先读取。
		Matrix(const int32_t m, const int32_t n, const double* val_);
		/// @brief 深拷贝构造矩阵。
		Matrix(const Matrix &M);
		/// @brief 释放矩阵内存。
		~Matrix();

		/// @brief 赋值并深拷贝矩阵内容。
		Matrix& operator= (const Matrix &M);

		/// @brief 将子矩阵复制到外部连续数组，默认范围表示整矩阵。
		void getData(double* val_, int32_t i1 = 0, int32_t j1 = 0, int32_t i2 = -1, int32_t j2 = -1);

		/// @brief 获取当前矩阵的子矩阵。
		Matrix getMat(int32_t i1, int32_t j1, int32_t i2 = -1, int32_t j2 = -1);
		/// @brief 从指定行列开始写入子矩阵。
		void   setMat(const Matrix &M, const int32_t i, const int32_t j);

		/// @brief 将子矩阵设置为标量；结束索引为 -1 时表示直到末尾。
		void setVal(double s, int32_t i1 = 0, int32_t j1 = 0, int32_t i2 = -1, int32_t j2 = -1);

		/// @brief 将对角线区间设置为标量；结束索引为 -1 时表示整个对角线。
		void setDiag(double s, int32_t i1 = 0, int32_t i2 = -1);

		/// @brief 将所有元素清零。
		void zero();

		/// @brief 按列索引提取子矩阵。
		Matrix extractCols(vector<int> idx);

		/// @brief 创建 `m x m` 单位矩阵。
		static Matrix eye(const int32_t m);
		/// @brief 将当前方阵设置为单位矩阵。
		void          eye();

		/// @brief 创建全 1 矩阵。
		static Matrix ones(const int32_t m, const int32_t n);

		/// @brief 根据向量元素创建对角矩阵。
		static Matrix diag(const Matrix &M);

		/// @brief 按列优先顺序重排矩阵元素。
		static Matrix reshape(const Matrix &M, int32_t m, int32_t n);

		/// @brief 创建绕 X 轴旋转矩阵，角度单位为 rad。
		static Matrix rotMatX(const double &angle);
		/// @brief 创建绕 Y 轴旋转矩阵，角度单位为 rad。
		static Matrix rotMatY(const double &angle);
		/// @brief 创建绕 Z 轴旋转矩阵，角度单位为 rad。
		static Matrix rotMatZ(const double &angle);

		// simple arithmetic operations
		Matrix  operator+ (const Matrix &M); // add matrix
		Matrix  operator- (const Matrix &M); // subtract matrix
		Matrix  operator* (const Matrix &M); // multiply with matrix
		Matrix  operator* (const double &s);  // multiply with scalar
		Matrix  operator/ (const Matrix &M); // divide elementwise by matrix (or vector)
		Matrix  operator/ (const double &s);  // divide by scalar
		Matrix  operator- ();                // negative matrix
		Matrix  operator~ ();                // transpose
		double   l2norm();                   // euclidean norm (vectors) / frobenius norm (matrices)
		double   mean();                     // mean of all elements in matrix

		// complex arithmetic operations
		static Matrix cross(const Matrix &a, const Matrix &b);    // cross product of two vectors
		static Matrix inv(const Matrix &M);                       // invert matrix M
		bool   inv();                                             // invert this matrix
		double  det();                                             // returns determinant of matrix
		bool   solve(const Matrix &M, double eps = 1e-20);            // solve linear system M*x=B, replaces *this and M
		bool   lu(int32_t *idx, double &d, double eps = 1e-20);        // replace *this by lower upper decomposition
		void   svd(Matrix &U, Matrix &W, Matrix &V);                 // singular value decomposition *this = U*diag(W)*V^T

		// print matrix to stream
		friend ostream& operator<< (ostream& out, const Matrix& M);

		// direct data access
		double   **val;
		int32_t   m, n;

	private:

		void allocateMemory(const int32_t m_, const int32_t n_);
		void releaseMemory();
		inline double pythag(double a, double b);

	};
}

#endif // I2R_RLC_VISION_ICP_LIBICP_MATRIX_H


