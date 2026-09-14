/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

Authors: Andreas Geiger

openMP support by Manolis Lourakis, Foundation for Research & Technology - Hellas, Heraklion, Greece

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
 * @file icpPointToPoint.cpp
 * @brief libicp 点到点、尺度、仿射和 BDD 分支实现。
 * @details 旧工程迁入文件；当前仅补充文档和 include 顺序，不改变最小二乘公式。
 */

#ifdef _OPENMP
#include <omp.h>
#endif

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Geometry>

#include "icpPointToPoint.h"

using namespace std;

namespace {

/**
 * @brief 旧 BDD/连接约束分支使用的三维点缓存。
 *
 * 坐标单位与输入点云一致；Vision 封装调用时为 mm。
 */
struct Point3Df {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

/// @brief 旧工程三维几何分支使用的圆周率常量。
constexpr double kPi = 3.141592653589793238462643383279502884;

} // namespace

// Also see (3d part): "Least-Squares Fitting of Two 3-D Point Sets" (Arun, Huang and Blostein)
//double CIcpPointToPoint::fitStep(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const std::vector<int32_t> &active) {
//
//	int i;
//	int nact = (int)active.size();
//
//	// init matrix for point correspondences
//	RMatrix::Matrix p_m(nact, m_dim); // model
//	RMatrix::Matrix p_t(nact, m_dim); // template
//
//	// init mean
//	RMatrix::Matrix mu_m(1, m_dim);
//	RMatrix::Matrix mu_t(1, m_dim);
//
//	// dimensionality 2
//	if (m_dim == 2) {
//
//		// extract matrix and translation vector
//		double r00 = R.val[0][0]; double r01 = R.val[0][1];
//		double r10 = R.val[1][0]; double r11 = R.val[1][1];
//		double t0 = t.val[0][0]; double t1 = t.val[1][0];
//		double mum0 = 0.0, mum1 = 0.0;
//		double mut0 = 0.0, mut1 = 0.0;
//
//		// establish correspondences
//#pragma omp parallel for private(i) default(none) shared(T,active,nact,p_m,p_t,r00,r01,r10,r11,t0,t1) reduction(+:mum0,mum1, mut0,mut1) // schedule (dynamic,2)
//		for (i = 0; i < nact; i++) {
//			// kd tree query + result
//			std::vector<float>         query(m_dim);
//			kdtree::KDTreeResultVector result;
//
//			// get index of active point
//			int32_t idx = active[i];
//
//			// transform point according to R|t
//			query[0] = (float)(r00*T[idx * 2 + 0] + r01 * T[idx * 2 + 1] + t0);
//			query[1] = (float)(r10*T[idx * 2 + 0] + r11 * T[idx * 2 + 1] + t1);
//
//			// search nearest neighbor
//			m_tree->n_nearest(query, 1, result);
//
//			// set model point
//			p_m.val[i][0] = m_tree->the_data[result[0].idx][0]; mum0 += p_m.val[i][0];
//			p_m.val[i][1] = m_tree->the_data[result[0].idx][1]; mum1 += p_m.val[i][1];
//
//			// set template point
//			p_t.val[i][0] = query[0]; mut0 += p_t.val[i][0];
//			p_t.val[i][1] = query[1]; mut1 += p_t.val[i][1];
//		}
//		mu_m.val[0][0] = mum0;
//		mu_m.val[0][1] = mum1;
//
//		mu_t.val[0][0] = mut0;
//		mu_t.val[0][1] = mut1;
//
//		// dimensionality 3
//	}
//	else {
//
//		// extract matrix and translation vector
//		double r00 = R.val[0][0]; double r01 = R.val[0][1]; double r02 = R.val[0][2];
//		double r10 = R.val[1][0]; double r11 = R.val[1][1]; double r12 = R.val[1][2];
//		double r20 = R.val[2][0]; double r21 = R.val[2][1]; double r22 = R.val[2][2];
//		double t0 = t.val[0][0]; double t1 = t.val[1][0]; double t2 = t.val[2][0];
//		double mum0 = 0.0, mum1 = 0.0, mum2 = 0.0;
//		double mut0 = 0.0, mut1 = 0.0, mut2 = 0.0;
//
//		// establish correspondences
//#pragma omp parallel for private(i) default(none) shared(T,active,nact,p_m,p_t,r00,r01,r02,r10,r11,r12,r20,r21,r22,t0,t1,t2) reduction(+:mum0,mum1,mum2, mut0,mut1,mut2) // schedule (dynamic,2)
//		for (i = 0; i < nact; i++) {
//			// kd tree query + result
//			std::vector<float>         query(m_dim);
//			kdtree::KDTreeResultVector result;
//
//			// get index of active point
//			int32_t idx = active[i];
//
//			// transform point according to R|t
//			query[0] = (float)(r00*T[idx * 3 + 0] + r01 * T[idx * 3 + 1] + r02 * T[idx * 3 + 2] + t0);
//			query[1] = (float)(r10*T[idx * 3 + 0] + r11 * T[idx * 3 + 1] + r12 * T[idx * 3 + 2] + t1);
//			query[2] = (float)(r20*T[idx * 3 + 0] + r21 * T[idx * 3 + 1] + r22 * T[idx * 3 + 2] + t2);
//
//			// search nearest neighbor
//			m_tree->n_nearest(query, 1, result);
//
//			// set model point
//			p_m.val[i][0] = m_tree->the_data[result[0].idx][0]; mum0 += p_m.val[i][0];
//			p_m.val[i][1] = m_tree->the_data[result[0].idx][1]; mum1 += p_m.val[i][1];
//			p_m.val[i][2] = m_tree->the_data[result[0].idx][2]; mum2 += p_m.val[i][2];
//
//			// set template point
//			p_t.val[i][0] = query[0]; mut0 += p_t.val[i][0];
//			p_t.val[i][1] = query[1]; mut1 += p_t.val[i][1];
//			p_t.val[i][2] = query[2]; mut2 += p_t.val[i][2];
//		}
//		mu_m.val[0][0] = mum0;
//		mu_m.val[0][1] = mum1;
//		mu_m.val[0][2] = mum2;
//
//		mu_t.val[0][0] = mut0;
//		mu_t.val[0][1] = mut1;
//		mu_t.val[0][2] = mut2;
//	}
//
//	// subtract mean
//	mu_m = mu_m / (double)active.size();
//	mu_t = mu_t / (double)active.size();
//
// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
//	if (m_keepConnectFlag == 1)
//	{
//		mu_m.val[0][0] = p_m.val[0][0];
//		mu_m.val[0][1] = p_m.val[0][1];
//		mu_m.val[0][2] = p_m.val[0][2];
//
//		mu_t.val[0][0] = p_t.val[0][0];
//		mu_t.val[0][1] = p_t.val[0][1];
//		mu_t.val[0][2] = p_t.val[0][2];
//	}
//
// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
//	RMatrix::Matrix q_m = p_m - RMatrix::Matrix::ones(active.size(), 1)*mu_m;
//	RMatrix::Matrix q_t = p_t - RMatrix::Matrix::ones(active.size(), 1)*mu_t;
//
//	// compute relative rotation matrix R and translation vector t
//	RMatrix::Matrix H = ~q_t*q_m;
//	RMatrix::Matrix U, W, V;
//	H.svd(U, W, V);
//	RMatrix::Matrix R_ = V * ~U;
//
//	// fix improper matrix problem
//	if (R_.det() < 0) {
//		RMatrix::Matrix B = RMatrix::Matrix::eye(m_dim);
//		B.val[m_dim - 1][m_dim - 1] = R_.det();
//		R_ = V * B*~U;
//	}
//
//		if (m_keepRotation == 1)
//		{
//		//1.0
//		//R_ = RMatrix::Matrix::eye(m_dim);
//
// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
//		//double* Rvalue = new double[m_dim*m_dim];
//		//RMatrix::Matrix tempR = R_ * R;
//		//tempR.getData(Rvalue);
//		//cv::Mat temp_matrix(m_dim, m_dim, CV_64FC1);
//		//for (int i = 0;i < m_dim;++i) {
//		//	for (int j = 0;j < m_dim;++j) {
//		//		temp_matrix.at<double>(i, j) = Rvalue[m_dim*i + j];
//		//	}
//		//}
//		//std::vector<double> rotationVector;
//		//cv::Rodrigues(temp_matrix, rotationVector);
//		//double rotationVectorAngle = 0;
//		//for (int i = 0;i < m_dim;++i)
//		//{
//		//	rotationVectorAngle += pow(rotationVector[i], 2);
//		//}
//		//rotationVectorAngle = sqrt(rotationVectorAngle);
//		//const double MAX_ROTATION_ANGLE = 0.15;
//		//if (rotationVectorAngle > MAX_ROTATION_ANGLE)
//		//{
//		//	for (int i = 0;i < m_dim;++i)
//		//	{
//		//		rotationVector[i] *= double(MAX_ROTATION_ANGLE / rotationVectorAngle);
//		//	}
//		//}
//		//cv::Mat afterMatrix(3, 3, CV_64FC1);
//		//cv::Rodrigues(rotationVector, afterMatrix);
//		//for (int i = 0;i < m_dim;++i)
//		//{
//		//	for (int j = 0;j < m_dim;++j)
//		//	{
//		//		Rvalue[m_dim*i + j] = afterMatrix.at<double>(i, j);
//		//	}
//		//}
//		//tempR = RMatrix::Matrix(m_dim, m_dim, Rvalue);
//		//R_ = tempR * RMatrix::Matrix::inv(R);
//
// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
//			Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
//			double* Rvalue = new double[m_dim*m_dim];
//			RMatrix::Matrix tempR = R_ ;
//			tempR.getData(Rvalue);
//			for (int i = 0;i < m_dim;++i) {
//				for (int j = 0;j < m_dim;++j) {
//					rotation_matrix(i, j) = Rvalue[m_dim*i + j];
//				}
//			}
//			Eigen::Vector3d euler_angles;
//			euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
//			euler_angles = euler_angles / kPi * 180.0;
//			const int allowAngleDegree = 3;
//			for (int i = 0;i < m_dim;++i) {
//				if (euler_angles[i] > 90 && euler_angles[i] < 180)
//				{
//					if (i == 0)
//						euler_angles[i] = -180 + euler_angles[i];
//					else if (i == 1)
//						euler_angles[i] = -180 + euler_angles[i];
//					else
//						euler_angles[i] = -180 + euler_angles[i];
//				}
//				if (euler_angles[i]<-90 && euler_angles[i]>-180)
//					euler_angles[i] = 180 + euler_angles[i];
//				if (abs(euler_angles[i]) > allowAngleDegree) {
//					if (euler_angles[i] > 0)
//						euler_angles[i] = allowAngleDegree;
//					else
//						euler_angles[i] = -allowAngleDegree;
//				}
//			}
// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
//			euler_angles[1] = euler_angles[2] = 0;
//				euler_angles = euler_angles * kPi / 180.0;
//			Eigen::Matrix3d after_matrix;
//			after_matrix = Eigen::AngleAxisd(euler_angles[0], Eigen::Vector3d::UnitZ())*
//				Eigen::AngleAxisd(euler_angles[1], Eigen::Vector3d::UnitY())*
//				Eigen::AngleAxisd(euler_angles[2], Eigen::Vector3d::UnitX());
//			for (int i = 0;i < m_dim;++i)
//			{
//				for (int j = 0;j < m_dim;++j)
//				{
//					Rvalue[m_dim*i + j] = after_matrix(i, j);
//				}
//			}
//			tempR = RMatrix::Matrix(m_dim, m_dim, Rvalue);
//			R_ = tempR;
//			delete[]Rvalue;
//		}
//
//	RMatrix::Matrix t_ = ~mu_m - R_ * ~mu_t;
//	// compose: R|t = R_|t_ * R|t
//	R = R_ * R;
//	t = R_ * t + t_;
//
//	// return max delta in parameters
//	if (m_dim == 2) return max((R_ - RMatrix::Matrix::eye(2)).l2norm(), t_.l2norm());
//	else        return max((R_ - RMatrix::Matrix::eye(3)).l2norm(), t_.l2norm());
//}

// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
double CIcpPointToPoint::fitStep(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const std::vector<int32_t> &active) {

	int i;
	int nact = (int)active.size();

	// init matrix for point correspondences
	RMatrix::Matrix p_m(nact, m_dim); // model
	RMatrix::Matrix p_t(nact, m_dim); // template

	// init mean
	RMatrix::Matrix mu_m(1, m_dim);
	RMatrix::Matrix mu_t(1, m_dim);
	vector<Point3Df> tempModelPoints;
	vector<Point3Df> tempTemplatePoints;
	vector<double> distanceGroup;
	int sucessPointCount = 0;
	// dimensionality 2
	if (m_dim == 2) {

		// extract matrix and translation vector
		double r00 = R.val[0][0]; double r01 = R.val[0][1];
		double r10 = R.val[1][0]; double r11 = R.val[1][1];
		double t0 = t.val[0][0]; double t1 = t.val[1][0];
		double mum0 = 0.0, mum1 = 0.0;
		double mut0 = 0.0, mut1 = 0.0;

		// establish correspondences
#pragma omp parallel for private(i) default(none) shared(T,active,nact,p_m,p_t,r00,r01,r10,r11,t0,t1) reduction(+:mum0,mum1, mut0,mut1) // schedule (dynamic,2)
		for (i = 0; i < nact; i++) {
			// kd tree query + result
			std::vector<float>         query(m_dim);
			kdtree::KDTreeResultVector result;

			// get index of active point
			int32_t idx = active[i];

			// transform point according to R|t
			query[0] = (float)(r00*T[idx * 2 + 0] + r01 * T[idx * 2 + 1] + t0);
			query[1] = (float)(r10*T[idx * 2 + 0] + r11 * T[idx * 2 + 1] + t1);

			// search nearest neighbor
			m_tree->n_nearest(query, 1, result);

			// set model point
			p_m.val[i][0] = m_tree->the_data[result[0].idx][0]; mum0 += p_m.val[i][0];
			p_m.val[i][1] = m_tree->the_data[result[0].idx][1]; mum1 += p_m.val[i][1];

			// set template point
			p_t.val[i][0] = query[0]; mut0 += p_t.val[i][0];
			p_t.val[i][1] = query[1]; mut1 += p_t.val[i][1];
		}
		mu_m.val[0][0] = mum0;
		mu_m.val[0][1] = mum1;

		mu_t.val[0][0] = mut0;
		mu_t.val[0][1] = mut1;

		// dimensionality 3
	}
	else {

		// extract matrix and translation vector
		double r00 = R.val[0][0]; double r01 = R.val[0][1]; double r02 = R.val[0][2];
		double r10 = R.val[1][0]; double r11 = R.val[1][1]; double r12 = R.val[1][2];
		double r20 = R.val[2][0]; double r21 = R.val[2][1]; double r22 = R.val[2][2];
		double t0 = t.val[0][0]; double t1 = t.val[1][0]; double t2 = t.val[2][0];
		double mum0 = 0.0, mum1 = 0.0, mum2 = 0.0;
		double mut0 = 0.0, mut1 = 0.0, mut2 = 0.0;

		// establish correspondences
#pragma omp parallel for private(i) default(none) shared(T,active,nact,p_m,p_t,r00,r01,r02,r10,r11,r12,r20,r21,r22,t0,t1,t2) reduction(+:mum0,mum1,mum2, mut0,mut1,mut2) // schedule (dynamic,2)
		for (i = 0; i < nact; i++) {
			// kd tree query + result
			std::vector<float>         query(m_dim);
			kdtree::KDTreeResultVector result;

			// get index of active point
			int32_t idx = active[i];

			// transform point according to R|t
			query[0] = (float)(r00*T[idx * 3 + 0] + r01 * T[idx * 3 + 1] + r02 * T[idx * 3 + 2] + t0);
			query[1] = (float)(r10*T[idx * 3 + 0] + r11 * T[idx * 3 + 1] + r12 * T[idx * 3 + 2] + t1);
			query[2] = (float)(r20*T[idx * 3 + 0] + r21 * T[idx * 3 + 1] + r22 * T[idx * 3 + 2] + t2);

			// search nearest neighbor
			m_tree->n_nearest(query, 1, result);

			if (m_keepRemoveFailPointFlag == 1)
			{
				Point3Df tempM, tempT;
				tempM.x = m_tree->the_data[result[0].idx][0];
				tempM.y = m_tree->the_data[result[0].idx][1];
				tempM.z = m_tree->the_data[result[0].idx][2];
				tempT.x = query[0];
				tempT.y = query[1];
				tempT.z = query[2];
				double tempDis = 0;
				tempDis = sqrt(pow(tempM.x - tempT.x, 2) + pow(tempM.y - tempT.y, 2) + pow(tempM.z - tempT.z, 2));
				distanceGroup.push_back(tempDis);
				tempModelPoints.push_back(tempM);
				tempTemplatePoints.push_back(tempT);

			}
			else {
				// set model point
				p_m.val[i][0] = m_tree->the_data[result[0].idx][0]; mum0 += p_m.val[i][0];
				p_m.val[i][1] = m_tree->the_data[result[0].idx][1]; mum1 += p_m.val[i][1];
				p_m.val[i][2] = m_tree->the_data[result[0].idx][2]; mum2 += p_m.val[i][2];

				// set template point
				p_t.val[i][0] = query[0]; mut0 += p_t.val[i][0];
				p_t.val[i][1] = query[1]; mut1 += p_t.val[i][1];
				p_t.val[i][2] = query[2]; mut2 += p_t.val[i][2];
			}
		}

		if (m_keepRemoveFailPointFlag == 1)
		{
			// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
			auto tempDistanceGroup = distanceGroup;
			sort(distanceGroup.begin(), distanceGroup.end());
			double midDistanceValue = distanceGroup[(distanceGroup.size() - 1) / 2];
			double distanceAllowRange =3;
			for (int i = 0;i < distanceGroup.size();++i)
			{
				if (tempDistanceGroup[i] < midDistanceValue + distanceAllowRange)
				{
					mum0 += tempModelPoints[i].x;
					mum1 += tempModelPoints[i].y;
					mum2 += tempModelPoints[i].z;

					mut0 += tempTemplatePoints[i].x;
					mut1 += tempTemplatePoints[i].y;
					mut2 += tempTemplatePoints[i].z;
					sucessPointCount++;
				}
				else {
					tempModelPoints[i].x = 0;
					tempModelPoints[i].y = 0;
					tempModelPoints[i].z = 0;

					tempTemplatePoints[i].x = 0;
					tempTemplatePoints[i].y = 0;
					tempTemplatePoints[i].z = 0;
				}
			}
		}

			mu_m.val[0][0] = mum0;
			mu_m.val[0][1] = mum1;
			mu_m.val[0][2] = mum2;

			mu_t.val[0][0] = mut0;
			mu_t.val[0][1] = mut1;
			mu_t.val[0][2] = mut2;
	}
	
	if (m_keepRemoveFailPointFlag==1)
	{
		p_m = RMatrix::Matrix(sucessPointCount, m_dim); // model
		p_t = RMatrix::Matrix(sucessPointCount, m_dim); // template

		int tempCounter = 0;
		for (int i = 0;i < sucessPointCount;++i)
		{
			if (tempModelPoints[tempCounter].x != 0 && tempModelPoints[tempCounter].y != 0 && tempModelPoints[tempCounter].z != 0) {
				p_m.val[i][0] = tempModelPoints[tempCounter].x;
				p_m.val[i][1] = tempModelPoints[tempCounter].y;
				p_m.val[i][2] = tempModelPoints[tempCounter].z;

				p_t.val[i][0] = tempTemplatePoints[tempCounter].x;
				p_t.val[i][1] = tempTemplatePoints[tempCounter].y;
				p_t.val[i][2] = tempTemplatePoints[tempCounter].z;
			}
			else {
				i--;
			}
			tempCounter++;
		}
		// subtract mean
		mu_m = mu_m / sucessPointCount;
		mu_t = mu_t / sucessPointCount;
	}
	else {
		// subtract mean
		mu_m = mu_m / (double)active.size();
		mu_t = mu_t / (double)active.size();
	}


	// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
	if (m_keepConnectFlag == 1)
	{
		mu_m.val[0][0] = p_m.val[0][0];
		mu_m.val[0][1] = p_m.val[0][1];
		mu_m.val[0][2] = p_m.val[0][2];

		mu_t.val[0][0] = p_t.val[0][0];
		mu_t.val[0][1] = p_t.val[0][1];
		mu_t.val[0][2] = p_t.val[0][2];
	}

	// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
	RMatrix::Matrix q_m;
	RMatrix::Matrix q_t;
	if (m_keepRemoveFailPointFlag == 1)
	{

		q_m = p_m - RMatrix::Matrix::ones(sucessPointCount, 1)*mu_m;
		q_t = p_t - RMatrix::Matrix::ones(sucessPointCount, 1)*mu_t;
	}
	else {
		 q_m= p_m - RMatrix::Matrix::ones(active.size(), 1)*mu_m;
		q_t= p_t - RMatrix::Matrix::ones(active.size(), 1)*mu_t;
	}

	// compute relative rotation matrix R and translation vector t
	RMatrix::Matrix H = ~q_t*q_m;
	RMatrix::Matrix U, W, V;
	H.svd(U, W, V);
	RMatrix::Matrix R_ = V * ~U;

	// fix improper matrix problem
	if (R_.det() < 0) {
		RMatrix::Matrix B = RMatrix::Matrix::eye(m_dim);
		B.val[m_dim - 1][m_dim - 1] = R_.det();
		R_ = V * B*~U;
	}

	if (m_keepRotation == 1)
	{
		//1.0
		//R_ = RMatrix::Matrix::eye(m_dim);

		// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
		//double* Rvalue = new double[m_dim*m_dim];
		//RMatrix::Matrix tempR = R_ * R;
		//tempR.getData(Rvalue);
		//cv::Mat temp_matrix(m_dim, m_dim, CV_64FC1);
		//for (int i = 0;i < m_dim;++i) {
		//	for (int j = 0;j < m_dim;++j) {
		//		temp_matrix.at<double>(i, j) = Rvalue[m_dim*i + j];
		//	}
		//}
		//std::vector<double> rotationVector;
		//cv::Rodrigues(temp_matrix, rotationVector);
		//double rotationVectorAngle = 0;
		//for (int i = 0;i < m_dim;++i)
		//{
		//	rotationVectorAngle += pow(rotationVector[i], 2);
		//}
		//rotationVectorAngle = sqrt(rotationVectorAngle);
		//const double MAX_ROTATION_ANGLE = 0.15;
		//if (rotationVectorAngle > MAX_ROTATION_ANGLE)
		//{
		//	for (int i = 0;i < m_dim;++i)
		//	{
		//		rotationVector[i] *= double(MAX_ROTATION_ANGLE / rotationVectorAngle);
		//	}
		//}
		//cv::Mat afterMatrix(3, 3, CV_64FC1);
		//cv::Rodrigues(rotationVector, afterMatrix);
		//for (int i = 0;i < m_dim;++i)
		//{
		//	for (int j = 0;j < m_dim;++j)
		//	{
		//		Rvalue[m_dim*i + j] = afterMatrix.at<double>(i, j);
		//	}
		//}
		//tempR = RMatrix::Matrix(m_dim, m_dim, Rvalue);
		//R_ = tempR * RMatrix::Matrix::inv(R);

		// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
		Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
		double* Rvalue = new double[m_dim*m_dim];
		RMatrix::Matrix tempR = R_;
		tempR.getData(Rvalue);
		for (int i = 0;i < m_dim;++i) {
			for (int j = 0;j < m_dim;++j) {
				rotation_matrix(i, j) = Rvalue[m_dim*i + j];
			}
		}
		Eigen::Vector3d euler_angles;
		euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
		euler_angles = euler_angles / kPi * 180.0;
		const int allowAngleDegree = 3;
		for (int i = 0;i < m_dim;++i) {
			if (euler_angles[i] > 90 && euler_angles[i] < 180)
			{
				if (i == 0)
					euler_angles[i] = -180 + euler_angles[i];
				else if (i == 1)
					euler_angles[i] = -180 + euler_angles[i];
				else
					euler_angles[i] = -180 + euler_angles[i];
			}
			if (euler_angles[i]<-90 && euler_angles[i]>-180)
				euler_angles[i] = 180 + euler_angles[i];
			if (abs(euler_angles[i]) > allowAngleDegree) {
				if (euler_angles[i] > 0)
					euler_angles[i] = allowAngleDegree;
				else
					euler_angles[i] = -allowAngleDegree;
			}
		}
		// 只保留 z 轴旋转。
		euler_angles[1] = euler_angles[2] = 0;
		euler_angles = euler_angles * kPi / 180.0;
		Eigen::Matrix3d after_matrix;
		after_matrix = Eigen::AngleAxisd(euler_angles[0], Eigen::Vector3d::UnitZ())*
			Eigen::AngleAxisd(euler_angles[1], Eigen::Vector3d::UnitY())*
			Eigen::AngleAxisd(euler_angles[2], Eigen::Vector3d::UnitX());
		for (int i = 0;i < m_dim;++i)
		{
			for (int j = 0;j < m_dim;++j)
			{
				Rvalue[m_dim*i + j] = after_matrix(i, j);
			}
		}
		tempR = RMatrix::Matrix(m_dim, m_dim, Rvalue);
		R_ = tempR;
		delete[]Rvalue;
	}

	RMatrix::Matrix t_ = ~mu_m - R_ * ~mu_t;
	// compose: R|t = R_|t_ * R|t
	R = R_ * R;
	t = R_ * t + t_;

	// return max delta in parameters
	if (m_dim == 2) return max((R_ - RMatrix::Matrix::eye(2)).l2norm(), t_.l2norm());
	else        return max((R_ - RMatrix::Matrix::eye(3)).l2norm(), t_.l2norm());
}

// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
//double CIcpPointToPoint::fitStep(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const std::vector<int32_t> &active) {
//
//	int i;
//	int nact = (int)active.size();
//
//	//// init matrix for point correspondences
//	//RMatrix::Matrix p_m(nact, m_dim); // model
//	//RMatrix::Matrix p_t(nact, m_dim); // template
//
//	// init mean
//	RMatrix::Matrix mu_m(1, m_dim);
//	RMatrix::Matrix mu_t(1, m_dim);
//
//	vector<Point3Df> tempModelPoints;
//	vector<Point3Df> tempTemplatePoints;
//	vector<double> distanceGroup;
//	int sucessPointCount = 0;
//	// dimensionality 2
//	if (m_dim == 2) {
//
//		// extract matrix and translation vector
//		double r00 = R.val[0][0]; double r01 = R.val[0][1];
//		double r10 = R.val[1][0]; double r11 = R.val[1][1];
//		double t0 = t.val[0][0]; double t1 = t.val[1][0];
//		double mum0 = 0.0, mum1 = 0.0;
//		double mut0 = 0.0, mut1 = 0.0;
//
//		// establish correspondences
//#pragma omp parallel for private(i) default(none) shared(T,active,nact,p_m,p_t,r00,r01,r10,r11,t0,t1) reduction(+:mum0,mum1, mut0,mut1) // schedule (dynamic,2)
//		for (i = 0; i < nact; i++) {
//			// kd tree query + result
//			std::vector<float>         query(m_dim);
//			kdtree::KDTreeResultVector result;
//
//			// get index of active point
//			int32_t idx = active[i];
//
//			// transform point according to R|t
//			query[0] = (float)(r00*T[idx * 2 + 0] + r01 * T[idx * 2 + 1] + t0);
//			query[1] = (float)(r10*T[idx * 2 + 0] + r11 * T[idx * 2 + 1] + t1);
//
//			// search nearest neighbor
//			m_tree->n_nearest(query, 1, result);
//
//			// set model point
//			//p_m.val[i][0] = m_tree->the_data[result[0].idx][0]; mum0 += p_m.val[i][0];
//			//p_m.val[i][1] = m_tree->the_data[result[0].idx][1]; mum1 += p_m.val[i][1];
//
//			//// set template point
//			//p_t.val[i][0] = query[0]; mut0 += p_t.val[i][0];
//			//p_t.val[i][1] = query[1]; mut1 += p_t.val[i][1];
//		}
//		mu_m.val[0][0] = mum0;
//		mu_m.val[0][1] = mum1;
//
//		mu_t.val[0][0] = mut0;
//		mu_t.val[0][1] = mut1;
//
//		// dimensionality 3
//	}
//	else {
//
//		// extract matrix and translation vector
//		double r00 = R.val[0][0]; double r01 = R.val[0][1]; double r02 = R.val[0][2];
//		double r10 = R.val[1][0]; double r11 = R.val[1][1]; double r12 = R.val[1][2];
//		double r20 = R.val[2][0]; double r21 = R.val[2][1]; double r22 = R.val[2][2];
//		double t0 = t.val[0][0]; double t1 = t.val[1][0]; double t2 = t.val[2][0];
//		double mum0 = 0.0, mum1 = 0.0, mum2 = 0.0;
//		double mut0 = 0.0, mut1 = 0.0, mut2 = 0.0;
//
//		// establish correspondences
//#pragma omp parallel for private(i) default(none) shared(T,active,nact,p_m,p_t,r00,r01,r02,r10,r11,r12,r20,r21,r22,t0,t1,t2) reduction(+:mum0,mum1,mum2, mut0,mut1,mut2) // schedule (dynamic,2)
//		for (i = 0; i < nact; i++) {
//			// kd tree query + result
//			std::vector<float>         query(m_dim);
//			kdtree::KDTreeResultVector result;
//
//			// get index of active point
//			int32_t idx = active[i];
//
//			// transform point according to R|t
//			query[0] = (float)(r00*T[idx * 3 + 0] + r01 * T[idx * 3 + 1] + r02 * T[idx * 3 + 2] + t0);
//			query[1] = (float)(r10*T[idx * 3 + 0] + r11 * T[idx * 3 + 1] + r12 * T[idx * 3 + 2] + t1);
//			query[2] = (float)(r20*T[idx * 3 + 0] + r21 * T[idx * 3 + 1] + r22 * T[idx * 3 + 2] + t2);
//
//			// search nearest neighbor
//			m_tree->n_nearest(query, 1, result);
//
//			// set model point
//			//p_m.val[i][0] = m_tree->the_data[result[0].idx][0]; mum0 += p_m.val[i][0];
//			//p_m.val[i][1] = m_tree->the_data[result[0].idx][1]; mum1 += p_m.val[i][1];
//			//p_m.val[i][2] = m_tree->the_data[result[0].idx][2]; mum2 += p_m.val[i][2];
//			Point3Df tempM,tempT;
//			tempM.x = m_tree->the_data[result[0].idx][0];
//			tempM.y = m_tree->the_data[result[0].idx][1];
//			tempM.z = m_tree->the_data[result[0].idx][2];
//			tempT.x = query[0];
//			tempT.y = query[1];
//			tempT.z = query[2];
//			double tempDis = 0;
//			tempDis = sqrt(pow(tempM.x- tempT.x,2)+ pow(tempM.y - tempT.y, 2)+ pow(tempM.z - tempT.z, 2));
//			distanceGroup.push_back(tempDis);
//			tempModelPoints.push_back(tempM);
//			tempTemplatePoints.push_back(tempT);
//			// set template point
//			//p_t.val[i][0] = query[0]; mut0 += p_t.val[i][0];
//			//p_t.val[i][1] = query[1]; mut1 += p_t.val[i][1];
//			//p_t.val[i][2] = query[2]; mut2 += p_t.val[i][2];
//
//		}
//
// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
//		//auto tempDistanceGroup = distanceGroup;
//		//sort(distanceGroup.begin(), distanceGroup.end());
//		//double midDistanceValue = distanceGroup[(distanceGroup.size() - 1) / 2];
//		//double distanceAllowRange = 0.5*midDistanceValue;
//		//for (int i=0;i<distanceGroup.size();++i)
//		//{
//		//	if (tempDistanceGroup[i]<midDistanceValue+distanceAllowRange)
//		//	{
//		//		mum0 += tempModelPoints[i].x;
//		//		mum1 += tempModelPoints[i].y;
//		//		mum2 += tempModelPoints[i].z;
//
//		//		mut0 += tempTemplatePoints[i].x;
//		//		mut1 += tempTemplatePoints[i].y;
//		//		mut2 += tempTemplatePoints[i].z;
//		//		sucessPointCount++;
//		//	}
//		//	else {
//		//		tempModelPoints[i].x = 0;
//		//		tempModelPoints[i].y = 0;
//		//		tempModelPoints[i].z = 0;
//
//		//		tempTemplatePoints[i].x = 0;
//		//		tempTemplatePoints[i].y = 0;
//		//		tempTemplatePoints[i].z = 0;
//		//	}
//		//}
//		mu_m.val[0][0] = mum0;
//		mu_m.val[0][1] = mum1;
//		mu_m.val[0][2] = mum2;
//
//		mu_t.val[0][0] = mut0;
//		mu_t.val[0][1] = mut1;
//		mu_t.val[0][2] = mut2;
//	}
//
//	// init matrix for point correspondences
//	RMatrix::Matrix p_m(sucessPointCount, m_dim); // model
//	RMatrix::Matrix p_t(sucessPointCount, m_dim); // template
//
//	int tempCounter = 0;
//	for (int i=0;i<sucessPointCount;++i)
//	{
//		if (tempModelPoints[tempCounter].x != 0 && tempModelPoints[tempCounter].y != 0 && tempModelPoints[tempCounter].z != 0) {
//			p_m.val[i][0] = tempModelPoints[tempCounter].x;
//				p_m.val[i][1]=tempModelPoints[tempCounter].y;
//				p_m.val[i][2] = tempModelPoints[tempCounter].z;
//
//				p_t.val[i][0] = tempTemplatePoints[tempCounter].x;
//				p_t.val[i][1] = tempTemplatePoints[tempCounter].y;
//				p_t.val[i][2] = tempTemplatePoints[tempCounter].z;
//		}
//		else {
//			i--;
//		}
//		tempCounter++;
//	}
//	// subtract mean
//	mu_m = mu_m / sucessPointCount;
//	mu_t = mu_t / sucessPointCount;
//
// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
//	if (m_keepConnectFlag==1)
//	{
//		mu_m.val[0][0] = p_m.val[0][0];
//		mu_m.val[0][1] = p_m.val[0][1];
//		mu_m.val[0][2] = p_m.val[0][2];
//
//		mu_t.val[0][0] = p_t.val[0][0];
//		mu_t.val[0][1] = p_t.val[0][1];
//		mu_t.val[0][2] = p_t.val[0][2];
//	}
// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
//	RMatrix::Matrix q_m = p_m - RMatrix::Matrix::ones(sucessPointCount, 1)*mu_m;
//	RMatrix::Matrix q_t = p_t - RMatrix::Matrix::ones(sucessPointCount, 1)*mu_t;
//
//	// compute relative rotation matrix R and translation vector t
//	RMatrix::Matrix H = ~q_t*q_m;
//	RMatrix::Matrix U, W, V;
//	H.svd(U, W, V);
//	RMatrix::Matrix R_ = V * ~U;
//
//	// fix improper matrix problem
//	if (R_.det() < 0) {
//		RMatrix::Matrix B = RMatrix::Matrix::eye(m_dim);
//		B.val[m_dim - 1][m_dim - 1] = R_.det();
//		R_ = V * B*~U;
//	}
// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
//	if (m_keepRotation == 1)
//	{
//		//1.0
//		R_ = RMatrix::Matrix::eye(m_dim);
//
// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
//		//double* Rvalue = new double[m_dim*m_dim];
//		//RMatrix::Matrix tempR = R_ * R;
//		//tempR.getData(Rvalue);
//		//cv::Mat temp_matrix(m_dim, m_dim, CV_64FC1);
//		//for (int i = 0;i < m_dim;++i) {
//		//	for (int j = 0;j < m_dim;++j) {
//		//		temp_matrix.at<double>(i, j) = Rvalue[m_dim*i + j];
//		//	}
//		//}
//		//std::vector<double> rotationVector;
//		//cv::Rodrigues(temp_matrix, rotationVector);
//		//double rotationVectorAngle = 0;
//		//for (int i = 0;i < m_dim;++i)
//		//{
//		//	rotationVectorAngle += pow(rotationVector[i], 2);
//		//}
//		//rotationVectorAngle = sqrt(rotationVectorAngle);
//		//const double MAX_ROTATION_ANGLE = 0.15;
//		//if (rotationVectorAngle > MAX_ROTATION_ANGLE)
//		//{
//		//	for (int i = 0;i < m_dim;++i)
//		//	{
//		//		rotationVector[i] *= double(MAX_ROTATION_ANGLE / rotationVectorAngle);
//		//	}
//		//}
//		//cv::Mat afterMatrix(3, 3, CV_64FC1);
//		//cv::Rodrigues(rotationVector, afterMatrix);
//		//for (int i = 0;i < m_dim;++i)
//		//{
//		//	for (int j = 0;j < m_dim;++j)
//		//	{
//		//		Rvalue[m_dim*i + j] = afterMatrix.at<double>(i, j);
//		//	}
//		//}
//		//tempR = RMatrix::Matrix(m_dim, m_dim, Rvalue);
//		//R_ = tempR * RMatrix::Matrix::inv(R);
//
// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
//		//Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
//		//double* Rvalue = new double[m_dim*m_dim];
//		//RMatrix::Matrix tempR = R_ * R;
//		//tempR.getData(Rvalue);
//		//for (int i = 0;i < m_dim;++i) {
//		//	for (int j = 0;j < m_dim;++j) {
//		//		rotation_matrix(i, j) = Rvalue[m_dim*i + j];
//		//	}
//		//}
//		//Eigen::Vector3d euler_angles;
//		//euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
//		//euler_angles = euler_angles / kPi * 180.0;
//		//const int allowAngleDegree = 3;
//		//for (int i = 0;i < m_dim;++i) {
//		//	if (euler_angles[i] > 90 && euler_angles[i] < 180)
//		//	{
//		//		if (i == 0)
//		//			euler_angles[i] = -180 + euler_angles[i];
//		//		else if (i == 1)
//		//			euler_angles[i] = -180 + euler_angles[i];
//		//		else
//		//			euler_angles[i] = -180 + euler_angles[i];
//		//	}
//		//	if (euler_angles[i]<-90 && euler_angles[i]>-180)
//		//		euler_angles[i] = 180 + euler_angles[i];
//		//	if (abs(euler_angles[i]) > allowAngleDegree) {
//		//		if (euler_angles[i] > 0)
//		//			euler_angles[i] = allowAngleDegree;
//		//		else
//		//			euler_angles[i] = -allowAngleDegree;
//		//	}
//		//}
// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
//		//euler_angles = euler_angles * kPi / 180.0;
//		//Eigen::Matrix3d after_matrix;
//		//after_matrix = Eigen::AngleAxisd(euler_angles[0], Eigen::Vector3d::UnitZ())*
//		//	Eigen::AngleAxisd(euler_angles[1], Eigen::Vector3d::UnitY())*
//		//	Eigen::AngleAxisd(euler_angles[2], Eigen::Vector3d::UnitX());
//		//for (int i = 0;i < m_dim;++i)
//		//{
//		//	for (int j = 0;j < m_dim;++j)
//		//	{
//		//		Rvalue[m_dim*i + j] = after_matrix(i, j);
//		//	}
//		//}
//		//tempR = RMatrix::Matrix(m_dim, m_dim, Rvalue);
//		//R_ = tempR * RMatrix::Matrix::inv(R);
//		//delete[]Rvalue;
//	}
//
//	// compose: R|t = R_|t_ * R|t
//	R = R_ * R;
//	RMatrix::Matrix t_ = ~mu_m - R_ * ~mu_t;
//	t = R_ * t + t_;
//	//RMatrix::Matrix t_ = ~mu_m - ~mu_t;
//	//t =  t + t_;
//	// return max delta in parameters
//	if (m_dim == 2) return max((R_ - RMatrix::Matrix::eye(2)).l2norm(), t_.l2norm());
//	else        return max((R_ - RMatrix::Matrix::eye(3)).l2norm(), t_.l2norm());
//}

//double CIcpPointToPoint::fitStepRemoveR(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const std::vector<int32_t> &active,const int &m_keepRotation) {
//
//	int i;
//	int nact = (int)active.size();
//
//	//// init matrix for point correspondences
//	//RMatrix::Matrix p_m(nact, m_dim); // model
//	//RMatrix::Matrix p_t(nact, m_dim); // template
//
//	// init mean
//	RMatrix::Matrix mu_m(1, m_dim);
//	RMatrix::Matrix mu_t(1, m_dim);
//
//	vector<Point3Df> tempModelPoints;
//	vector<Point3Df> tempTemplatePoints;
//	vector<double> distanceGroup;
//	int sucessPointCount = 0;
//	// dimensionality 2
//	if (m_dim == 2) {
//
//		// extract matrix and translation vector
//		double r00 = R.val[0][0]; double r01 = R.val[0][1];
//		double r10 = R.val[1][0]; double r11 = R.val[1][1];
//		double t0 = t.val[0][0]; double t1 = t.val[1][0];
//		double mum0 = 0.0, mum1 = 0.0;
//		double mut0 = 0.0, mut1 = 0.0;
//
//		// establish correspondences
//#pragma omp parallel for private(i) default(none) shared(T,active,nact,p_m,p_t,r00,r01,r10,r11,t0,t1) reduction(+:mum0,mum1, mut0,mut1) // schedule (dynamic,2)
//		for (i = 0; i < nact; i++) {
//			// kd tree query + result
//			std::vector<float>         query(m_dim);
//			kdtree::KDTreeResultVector result;
//
//			// get index of active point
//			int32_t idx = active[i];
//
//			// transform point according to R|t
//			query[0] = (float)(r00*T[idx * 2 + 0] + r01 * T[idx * 2 + 1] + t0);
//			query[1] = (float)(r10*T[idx * 2 + 0] + r11 * T[idx * 2 + 1] + t1);
//
//			// search nearest neighbor
//			m_tree->n_nearest(query, 1, result);
//
//			// set model point
//			//p_m.val[i][0] = m_tree->the_data[result[0].idx][0]; mum0 += p_m.val[i][0];
//			//p_m.val[i][1] = m_tree->the_data[result[0].idx][1]; mum1 += p_m.val[i][1];
//
//			//// set template point
//			//p_t.val[i][0] = query[0]; mut0 += p_t.val[i][0];
//			//p_t.val[i][1] = query[1]; mut1 += p_t.val[i][1];
//		}
//		mu_m.val[0][0] = mum0;
//		mu_m.val[0][1] = mum1;
//
//		mu_t.val[0][0] = mut0;
//		mu_t.val[0][1] = mut1;
//
//		// dimensionality 3
//	}
//	else {
//
//		// extract matrix and translation vector
//		double r00 = R.val[0][0]; double r01 = R.val[0][1]; double r02 = R.val[0][2];
//		double r10 = R.val[1][0]; double r11 = R.val[1][1]; double r12 = R.val[1][2];
//		double r20 = R.val[2][0]; double r21 = R.val[2][1]; double r22 = R.val[2][2];
//		double t0 = t.val[0][0]; double t1 = t.val[1][0]; double t2 = t.val[2][0];
//		double mum0 = 0.0, mum1 = 0.0, mum2 = 0.0;
//		double mut0 = 0.0, mut1 = 0.0, mut2 = 0.0;
//
//		// establish correspondences
//#pragma omp parallel for private(i) default(none) shared(T,active,nact,p_m,p_t,r00,r01,r02,r10,r11,r12,r20,r21,r22,t0,t1,t2) reduction(+:mum0,mum1,mum2, mut0,mut1,mut2) // schedule (dynamic,2)
//		for (i = 0; i < nact; i++) {
//			// kd tree query + result
//			std::vector<float>         query(m_dim);
//			kdtree::KDTreeResultVector result;
//
//			// get index of active point
//			int32_t idx = active[i];
//
//			// transform point according to R|t
//			query[0] = (float)(r00*T[idx * 3 + 0] + r01 * T[idx * 3 + 1] + r02 * T[idx * 3 + 2] + t0);
//			query[1] = (float)(r10*T[idx * 3 + 0] + r11 * T[idx * 3 + 1] + r12 * T[idx * 3 + 2] + t1);
//			query[2] = (float)(r20*T[idx * 3 + 0] + r21 * T[idx * 3 + 1] + r22 * T[idx * 3 + 2] + t2);
//
//			// search nearest neighbor
//			m_tree->n_nearest(query, 1, result);
//
//			// set model point
//			//p_m.val[i][0] = m_tree->the_data[result[0].idx][0]; mum0 += p_m.val[i][0];
//			//p_m.val[i][1] = m_tree->the_data[result[0].idx][1]; mum1 += p_m.val[i][1];
//			//p_m.val[i][2] = m_tree->the_data[result[0].idx][2]; mum2 += p_m.val[i][2];
//			Point3Df tempM, tempT;
//			tempM.x = m_tree->the_data[result[0].idx][0];
//			tempM.y = m_tree->the_data[result[0].idx][1];
//			tempM.z = m_tree->the_data[result[0].idx][2];
//			tempT.x = query[0];
//			tempT.y = query[1];
//			tempT.z = query[2];
//			double tempDis = 0;
//			tempDis = sqrt(pow(tempM.x - tempT.x, 2) + pow(tempM.y - tempT.y, 2) + pow(tempM.z - tempT.z, 2));
//			distanceGroup.push_back(tempDis);
//			tempModelPoints.push_back(tempM);
//			tempTemplatePoints.push_back(tempT);
//			// set template point
//			//p_t.val[i][0] = query[0]; mut0 += p_t.val[i][0];
//			//p_t.val[i][1] = query[1]; mut1 += p_t.val[i][1];
//			//p_t.val[i][2] = query[2]; mut2 += p_t.val[i][2];
//
//		}
//
// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
//		auto tempDistanceGroup = distanceGroup;
//		sort(distanceGroup.begin(), distanceGroup.end());
//		double midDistanceValue = distanceGroup[(distanceGroup.size() - 1) / 2];
//		double distanceAllowRange = 0.5*midDistanceValue;
//		for (int i = 0;i < distanceGroup.size();++i)
//		{
//			if (tempDistanceGroup[i] < midDistanceValue + distanceAllowRange)
//			{
//				mum0 += tempModelPoints[i].x;
//				mum1 += tempModelPoints[i].y;
//				mum2 += tempModelPoints[i].z;
//
//				mut0 += tempTemplatePoints[i].x;
//				mut1 += tempTemplatePoints[i].y;
//				mut2 += tempTemplatePoints[i].z;
//				sucessPointCount++;
//			}
//			else {
//				tempModelPoints[i].x = 0;
//				tempModelPoints[i].y = 0;
//				tempModelPoints[i].z = 0;
//
//				tempTemplatePoints[i].x = 0;
//				tempTemplatePoints[i].y = 0;
//				tempTemplatePoints[i].z = 0;
//			}
//		}
//		mu_m.val[0][0] = mum0;
//		mu_m.val[0][1] = mum1;
//		mu_m.val[0][2] = mum2;
//
//		mu_t.val[0][0] = mut0;
//		mu_t.val[0][1] = mut1;
//		mu_t.val[0][2] = mut2;
//	}
//
//	// init matrix for point correspondences
//	RMatrix::Matrix p_m(sucessPointCount, m_dim); // model
//	RMatrix::Matrix p_t(sucessPointCount, m_dim); // template
//
//	int tempCounter = 0;
//	for (int i = 0;i < sucessPointCount;++i)
//	{
//		if (tempModelPoints[tempCounter].x != 0 && tempModelPoints[tempCounter].y != 0 && tempModelPoints[tempCounter].z != 0) {
//			p_m.val[i][0] = tempModelPoints[tempCounter].x;
//			p_m.val[i][1] = tempModelPoints[tempCounter].y;
//			p_m.val[i][2] = tempModelPoints[tempCounter].z;
//
//			p_t.val[i][0] = tempTemplatePoints[tempCounter].x;
//			p_t.val[i][1] = tempTemplatePoints[tempCounter].y;
//			p_t.val[i][2] = tempTemplatePoints[tempCounter].z;
//		}
//		else {
//			i--;
//		}
//		tempCounter++;
//	}
//	// subtract mean
//	mu_m = mu_m / sucessPointCount;
//	mu_t = mu_t / sucessPointCount;
//
// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
//	RMatrix::Matrix q_m = p_m - RMatrix::Matrix::ones(sucessPointCount, 1)*mu_m;
//	RMatrix::Matrix q_t = p_t - RMatrix::Matrix::ones(sucessPointCount, 1)*mu_t;
//
//	// compute relative rotation matrix R and translation vector t
//	RMatrix::Matrix H = ~q_t*q_m;
//	RMatrix::Matrix U, W, V;
//	H.svd(U, W, V);
//	RMatrix::Matrix R_ = V * ~U;
//
//	// fix improper matrix problem
//	if (R_.det() < 0) {
//		RMatrix::Matrix B = RMatrix::Matrix::eye(m_dim);
//		B.val[m_dim - 1][m_dim - 1] = R_.det();
//		R_ = V * B*~U;
//	}
//
//	if (m_keepRotation == 1)
//	{
//		R = RMatrix::Matrix::eye(m_dim);
//	}
//	RMatrix::Matrix t_ = ~mu_m - R_ * ~mu_t;
//
//	// compose: R|t = R_|t_ * R|t
//	R = R_ * R;
//	t = R_ * t + t_;
//
//	// return max delta in parameters
//	if (m_dim == 2) return max((R_ - RMatrix::Matrix::eye(2)).l2norm(), t_.l2norm());
//	else        return max((R_ - RMatrix::Matrix::eye(3)).l2norm(), t_.l2norm());
//}

void CIcpPointToPoint::keepRotation()
{
	m_keepRotation = 1;
}

void CIcpPointToPoint::keepConnect()
{
	m_keepConnectFlag = 1;
}

void CIcpPointToPoint::keepRemoveFailPoint()
{
	m_keepRemoveFailPointFlag = 1;
}

double CIcpPointToPoint::fitStep_scale(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, RMatrix::Matrix&S, const std::vector<int32_t> &active) {

	int i;
	int nact = (int)active.size();

	// init matrix for point correspondences
	RMatrix::Matrix p_m(nact, m_dim); // model
	RMatrix::Matrix p_t(nact, m_dim); // template

	// init mean
	RMatrix::Matrix mu_m(1, m_dim);
	RMatrix::Matrix mu_t(1, m_dim);

	// dimensionality 2
	if (m_dim == 2) {

		// extract matrix and translation vector
		double r00 = R.val[0][0]; double r01 = R.val[0][1];
		double r10 = R.val[1][0]; double r11 = R.val[1][1];
		double t0 = t.val[0][0]; double t1 = t.val[1][0];
		double mum0 = 0.0, mum1 = 0.0;
		double mut0 = 0.0, mut1 = 0.0;

		// establish correspondences
#pragma omp parallel for private(i) default(none) shared(T,active,nact,p_m,p_t,r00,r01,r10,r11,t0,t1) reduction(+:mum0,mum1, mut0,mut1) // schedule (dynamic,2)
		for (i = 0; i < nact; i++) {
			// kd tree query + result
			std::vector<float>         query(m_dim);
			kdtree::KDTreeResultVector result;

			// get index of active point
			int32_t idx = active[i];

			// transform point according to R|t
			query[0] = (float)(r00*T[idx * 2 + 0] + r01 * T[idx * 2 + 1] + t0);
			query[1] = (float)(r10*T[idx * 2 + 0] + r11 * T[idx * 2 + 1] + t1);

			// search nearest neighbor
			m_tree->n_nearest(query, 1, result);

			// set model point
			p_m.val[i][0] = m_tree->the_data[result[0].idx][0]; mum0 += p_m.val[i][0];
			p_m.val[i][1] = m_tree->the_data[result[0].idx][1]; mum1 += p_m.val[i][1];

			// set template point
			p_t.val[i][0] = query[0]; mut0 += p_t.val[i][0];
			p_t.val[i][1] = query[1]; mut1 += p_t.val[i][1];
		}
		mu_m.val[0][0] = mum0;
		mu_m.val[0][1] = mum1;

		mu_t.val[0][0] = mut0;
		mu_t.val[0][1] = mut1;

		// dimensionality 3
	}
	else {

		// extract matrix and translation vector
		double r00 = R.val[0][0]; double r01 = R.val[0][1]; double r02 = R.val[0][2];
		double r10 = R.val[1][0]; double r11 = R.val[1][1]; double r12 = R.val[1][2];
		double r20 = R.val[2][0]; double r21 = R.val[2][1]; double r22 = R.val[2][2];
		double s0 = S.val[0][0];double s1 = S.val[1][1];double s2 = S.val[2][2];
		double t0 = t.val[0][0]; double t1 = t.val[1][0]; double t2 = t.val[2][0];
		double mum0 = 0.0, mum1 = 0.0, mum2 = 0.0;
		double mut0 = 0.0, mut1 = 0.0, mut2 = 0.0;

		// establish correspondences
#pragma omp parallel for private(i) default(none) shared(T,active,nact,p_m,p_t,r00,r01,r02,r10,r11,r12,r20,r21,r22,t0,t1,t2) reduction(+:mum0,mum1,mum2, mut0,mut1,mut2) // schedule (dynamic,2)
		for (i = 0; i < nact; i++) {
			// kd tree query + result
			std::vector<float>         query(m_dim);
			kdtree::KDTreeResultVector result;

			// get index of active point
			int32_t idx = active[i];

			// transform point according to R|t
			query[0] = (float)(r00*s0*T[idx * 3 + 0] + r01 * s1* T[idx * 3 + 1] + r02 * s2*T[idx * 3 + 2] + t0);
			query[1] = (float)(r10*s0*T[idx * 3 + 0] + r11 * s1* T[idx * 3 + 1] + r12 * s2*T[idx * 3 + 2] + t1);
			query[2] = (float)(r20*s0*T[idx * 3 + 0] + r21 * s1*T[idx * 3 + 1] + r22 * s2*T[idx * 3 + 2] + t2);


			// search nearest neighbor
			m_tree->n_nearest(query, 1, result);

			// set model point
			p_m.val[i][0] = m_tree->the_data[result[0].idx][0]; mum0 += p_m.val[i][0];
			p_m.val[i][1] = m_tree->the_data[result[0].idx][1]; mum1 += p_m.val[i][1];
			p_m.val[i][2] = m_tree->the_data[result[0].idx][2]; mum2 += p_m.val[i][2];

			// set template point
			p_t.val[i][0] = query[0]; mut0 += p_t.val[i][0];
			p_t.val[i][1] = query[1]; mut1 += p_t.val[i][1];
			p_t.val[i][2] = query[2]; mut2 += p_t.val[i][2];
		}
		mu_m.val[0][0] = mum0;
		mu_m.val[0][1] = mum1;
		mu_m.val[0][2] = mum2;

		mu_t.val[0][0] = mut0;
		mu_t.val[0][1] = mut1;
		mu_t.val[0][2] = mut2;
	}

	// subtract mean
	mu_m = mu_m / (double)active.size();
	mu_t = mu_t / (double)active.size();
	RMatrix::Matrix q_m = p_m - RMatrix::Matrix::ones(active.size(), 1)*mu_m;
	RMatrix::Matrix q_t = p_t - RMatrix::Matrix::ones(active.size(), 1)*mu_t;

	/***************q_m target, q_t model R*q_t+t=q_m******************/
	if (m_currentIter == 0)
	{
		//Matrix Cov_qm = ~q_m*q_m;
		//Matrix Cov_qt = ~q_t*q_t;
	/*Calculate initial Matrix(S)*/
		RMatrix::Matrix U_m, W_m, V_m;
		q_m.svd(U_m, W_m, V_m);
		RMatrix::Matrix U_t, W_t, V_t;
		q_t.svd(U_t, W_t, V_t);

		double tempa = W_m.val[0][0] / W_t.val[0][0];
		double tempb = W_m.val[1][0] / W_t.val[1][0];
		double tempc = W_m.val[2][0] / W_t.val[2][0];
		double upbound = 1.1;
		double lowbound = 0.90;
		for (int i = 0;i < 3;++i)
		{
			double temp = W_m.val[i][0] / W_t.val[i][0];
			if (temp > upbound || temp < lowbound)
			{
				if (abs(temp - upbound) < abs(temp - lowbound))
				{
					temp = upbound;
				}
				else
					temp = lowbound;
			}
			S.val[i][i] = temp;
		}
		//cout << S << endl;
	}

	int scale_max_iter = 5;
	RMatrix::Matrix R_;
	RMatrix::Matrix S_ = RMatrix::Matrix::eye(3);

	for (int S_iter = 0;S_iter < scale_max_iter;++S_iter)
	{
		// compute relative rotation matrix R and translation vector t
		RMatrix::Matrix H = S_ * ~q_t*q_m;

		RMatrix::Matrix U, W, V;
		H.svd(U, W, V);
		R_ = V * ~U;
		// fix improper matrix problem
		if (R_.det() < 0) {
			RMatrix::Matrix B = RMatrix::Matrix::eye(m_dim);
			B.val[m_dim - 1][m_dim - 1] = R_.det();
			R_ = V * B*~U;
		}
		double tempsum = 0;
		double temp_s1_mo = 0;
		double temp_s2_mo = 0;
		double temp_s3_mo = 0;

		double temp_s1_do = 0;
		double temp_s2_do = 0;
		double temp_s3_do = 0;
		for (int j = 0; j < nact; ++j)
		{
			temp_s1_mo += q_t.val[j][0] * (R_.val[0][0] * q_m.val[j][0] + R_.val[1][0] * q_m.val[j][1] + R_.val[2][0] * q_m.val[j][2]);
			temp_s2_mo += q_t.val[j][1] * (R_.val[0][1] * q_m.val[j][0] + R_.val[1][1] * q_m.val[j][1] + R_.val[2][1] * q_m.val[j][2]);
			temp_s3_mo += q_t.val[j][2] * (R_.val[0][2] * q_m.val[j][0] + R_.val[1][2] * q_m.val[j][1] + R_.val[2][2] * q_m.val[j][2]);
			temp_s1_do += q_t.val[j][0] * q_t.val[j][0];
			temp_s2_do += q_t.val[j][1] * q_t.val[j][1];
			temp_s3_do += q_t.val[j][2] * q_t.val[j][2];
		}
		double temp_s1 = temp_s1_mo / temp_s1_do;
		double temp_s2 = temp_s2_mo / temp_s2_do;
		double temp_s3 = temp_s3_mo / temp_s3_do;
		tempsum = (temp_s1 + temp_s2 + temp_s3) / 3.0;
		double upbound = 1.10;
		double lowbound = 0.85;
		if (temp_s1 > upbound || temp_s1 < lowbound)
		{
			if (abs(temp_s1 - upbound) < abs(temp_s1 - lowbound))
			{
				temp_s1 = upbound;
			}
			else
				temp_s1 = lowbound;
		}
		if (temp_s2 > upbound || temp_s2 < lowbound)
		{
			if (abs(temp_s2 - upbound) < abs(temp_s2 - lowbound))
			{
				temp_s2 = upbound;
			}
			else
				temp_s2 = lowbound;
		}
		if (temp_s3 > upbound || temp_s3 < lowbound)
		{
			if (abs(temp_s3 - upbound) < abs(temp_s3 - lowbound))
			{
				temp_s3 = upbound;
			}
			else
				temp_s3 = lowbound;
		}
		double delta = (temp_s1 - S_.val[0][0])*(temp_s1 - S_.val[0][0]) + (temp_s2 - S_.val[1][1])*(temp_s2 - S_.val[1][1])
			+ (temp_s3 - S_.val[2][2])*(temp_s3 - S_.val[2][2]);
		S_.val[0][0] = temp_s1;
		S_.val[1][1] = temp_s2;
		S_.val[2][2] = temp_s3;
		if (delta < m_minDelta)
		{
			break;
		}
	}


	//Matrix t_ = ~mu_t - R_*~mu_m;
	RMatrix::Matrix t_ = ~mu_m - R_ * ~mu_t;
	double tempsum = (S.val[0][0] + S.val[1][1] + S.val[2][2]) / 3.0;
	double upbound = 1.10;
	double lowbound = 0.85;
	// compose: R|t = R_|t_ * R|t
	R = R_ * R;
	S = S_ * S;
	if (S.val[0][0] < lowbound)
	{
		S.val[0][0] = lowbound;
	}
	if (S.val[1][1] < lowbound)
	{
		S.val[1][1] = lowbound;
	}
	if (S.val[2][2] < lowbound)
	{
		S.val[2][2] = lowbound;
	}
	if (S.val[0][0] > upbound)
	{
		S.val[0][0] = upbound;
	}
	if (S.val[1][1] > upbound)
	{
		S.val[1][1] = upbound;
	}
	if (S.val[2][2] > upbound)
	{
		S.val[2][2] = upbound;
	}
	t = R_ * t + t_;

	// return max delta in parameters
	if (m_dim == 2) return max((R_ - RMatrix::Matrix::eye(2)).l2norm(), t_.l2norm());
	else        return max((R_ - RMatrix::Matrix::eye(3)).l2norm(), t_.l2norm());
}


std::vector<int32_t> CIcpPointToPoint::getInliers(double *T, const int32_t T_num, const RMatrix::Matrix &R, const RMatrix::Matrix &t, const double indist) {

	// init inlier vector + query point + query result
	vector<int32_t>            inliers;
	std::vector<float>         query(m_dim);
	kdtree::KDTreeResultVector neighbor;

	// dimensionality 2
	if (m_dim == 2) {

		// extract matrix and translation vector
		double r00 = R.val[0][0]; double r01 = R.val[0][1];
		double r10 = R.val[1][0]; double r11 = R.val[1][1];
		double t0 = t.val[0][0]; double t1 = t.val[1][0];

		// check for all points if they are inliers
		for (int32_t i = 0; i < T_num; i++) {

			// transform point according to R|t
			query[0] = (float)(r00*T[i * 2 + 0] + r01 * T[i * 2 + 1] + t0);
			query[1] = (float)(r10*T[i * 2 + 0] + r11 * T[i * 2 + 1] + t1);

			// search nearest neighbor
			m_tree->n_nearest(query, 1, neighbor);

			// check if it is an inlier
			if (neighbor[0].dis < indist)
				inliers.push_back(i);
		}

		// dimensionality 3
	}
	else {

		// extract matrix and translation vector
		double r00 = R.val[0][0]; double r01 = R.val[0][1]; double r02 = R.val[0][2];
		double r10 = R.val[1][0]; double r11 = R.val[1][1]; double r12 = R.val[1][2];
		double r20 = R.val[2][0]; double r21 = R.val[2][1]; double r22 = R.val[2][2];
		double t0 = t.val[0][0]; double t1 = t.val[1][0]; double t2 = t.val[2][0];

		// check for all points if they are inliers
		for (int32_t i = 0; i < T_num; i++) {

			// transform point according to R|t
			query[0] = (float)(r00*T[i * 3 + 0] + r01 * T[i * 3 + 1] + r02 * T[i * 3 + 2] + t0);
			query[1] = (float)(r10*T[i * 3 + 0] + r11 * T[i * 3 + 1] + r12 * T[i * 3 + 2] + t1);
			query[2] = (float)(r20*T[i * 3 + 0] + r21 * T[i * 3 + 1] + r22 * T[i * 3 + 2] + t2);

			// search nearest neighbor
			m_tree->n_nearest(query, 1, neighbor);

			// check if it is an inlier
			if (neighbor[0].dis < indist)
				inliers.push_back(i);
		}
	}

	// return vector with inliers
	return inliers;
}
// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
double CIcpPointToPoint::fitStepAffine(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const std::vector<int32_t> &active) {

	int i;
	int nact = (int)active.size();

	// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
	RMatrix::Matrix K(2 * nact, 2 * (m_dim + 1));  // 旧工程迁移注释。
	K.zero();
	RMatrix::Matrix cv_err(m_dim*nact, 1);		// 旧工程迁移注释。
	cv_err.zero();

	// dimensionality 2
	if (m_dim == 2) {

		// extract matrix and translation vector
		double r00 = R.val[0][0]; double r01 = R.val[0][1];
		double r10 = R.val[1][0]; double r11 = R.val[1][1];
		double t0 = t.val[0][0]; double t1 = t.val[1][0];

		// establish correspondences
#pragma omp parallel for private(i) default(none) shared(T,active,nact,r00,r01,r10,r11,t0,t1,K,cv_err) // schedule (dynamic,2)
		for (i = 0; i < nact; i++) {
			// kd tree query + result
			std::vector<float>         query(m_dim);
			kdtree::KDTreeResultVector result;

			// get index of active point
			int32_t idx = active[i];

			// transform point according to R|t
			query[0] = (float)(r00*T[idx * 2 + 0] + r01 * T[idx * 2 + 1] + t0);
			query[1] = (float)(r10*T[idx * 2 + 0] + r11 * T[idx * 2 + 1] + t1);

			// search nearest neighbor
			m_tree->n_nearest(query, 1, result);

			// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
			for (int a = 0; a < m_dim; a++)
			{
				K.val[i][a] = query[a];
				K.val[i + nact][a + m_dim + 1] = query[a];
			}
			K.val[i][m_dim] = 1.0;
			K.val[i + nact][2 * m_dim + 1] = 1.0;
			// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
			for (int b = 0; b < m_dim; b++)
			{
				cv_err.val[i + b * nact][0] = m_tree->the_data[result[0].idx][b];
			}
		}
	}
	// dimensionality 3
	else {

		// extract matrix and translation vector
		double r00 = R.val[0][0]; double r01 = R.val[0][1]; double r02 = R.val[0][2];
		double r10 = R.val[1][0]; double r11 = R.val[1][1]; double r12 = R.val[1][2];
		double r20 = R.val[2][0]; double r21 = R.val[2][1]; double r22 = R.val[2][2];
		double t0 = t.val[0][0]; double t1 = t.val[1][0]; double t2 = t.val[2][0];

		// establish correspondences
#pragma omp parallel for private(i) default(none) shared(T,active,nact,r00,r01,r02,r10,r11,r12,r20,r21,r22,t0,t1,t2,K,cv_err)// schedule (dynamic,2)
		for (i = 0; i < nact; i++) {
			// kd tree query + result
			std::vector<float>         query(m_dim);
			kdtree::KDTreeResultVector result;

			// get index of active point
			int32_t idx = active[i];

			// transform point according to R|t
			query[0] = (float)(r00*T[idx * 3 + 0] + r01 * T[idx * 3 + 1] + r02 * T[idx * 3 + 2] + t0);
			query[1] = (float)(r10*T[idx * 3 + 0] + r11 * T[idx * 3 + 1] + r12 * T[idx * 3 + 2] + t1);
			query[2] = (float)(r20*T[idx * 3 + 0] + r21 * T[idx * 3 + 1] + r22 * T[idx * 3 + 2] + t2);

			// search nearest neighbor
			m_tree->n_nearest(query, 1, result);

			for (int a = 0; a < m_dim; a++)
			{
				K.val[i][a] = query[a];
				K.val[i + nact][a + m_dim + 1] = query[a];
				K.val[i][m_dim] = 1.0;
				K.val[i + nact][2 * m_dim + 1] = 1.0;
			}
			// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
			for (int b = 0; b < m_dim; b++)
			{
				cv_err.val[i + b * nact][0] = m_tree->the_data[result[0].idx][b];
			}
		}
	}

	RMatrix::Matrix ParaVec = (RMatrix::Matrix::inv(~K*K))*(~K)*cv_err;
	RMatrix::Matrix TransMat = RMatrix::Matrix::eye(m_dim + 1);
	for (int i = 0; i < m_dim; i++)
	{
		for (int j = 0; j < m_dim + 1; j++)
		{
			TransMat.val[i][j] = ParaVec.val[i*(m_dim + 1) + j][0];
		}
	}

	RMatrix::Matrix R_ = TransMat.getMat(0, 0, m_dim - 1, m_dim - 1);
	RMatrix::Matrix t_ = TransMat.getMat(0, m_dim, m_dim - 1, m_dim);

	// compose: R|t = R_|t_ * R|t
	R = R_ * R;
	t = R_ * t + t_;

	double delta = 0;
	if (m_dim == 2)
	{
		delta = max((R_ - RMatrix::Matrix::eye(2)).l2norm(), t_.l2norm());
	}
	else
	{
		delta = max((R_ - RMatrix::Matrix::eye(3)).l2norm(), t_.l2norm());
	}

	// return max delta in parameters
	if (m_dim == 2) return max((R_ - RMatrix::Matrix::eye(2)).l2norm(), t_.l2norm());
	else        return max((R_ - RMatrix::Matrix::eye(3)).l2norm(), t_.l2norm());
}

// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
double CIcpPointToPoint::fitStepAffineBDD(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const std::vector<int32_t> &active) {

	int i;
	int nact = (int)active.size();
	int LastClosestID_T2M = 0;	// 旧工程迁移注释。
	double MinDist_M2T = 0;	// 旧工程迁移注释。
	std::vector<int> AllPointPairsID;	// 旧工程迁移注释。
	std::vector<double> AllPointPairsDist;	// 旧工程迁移注释。
	AllPointPairsID.resize(2 * nact);
	AllPointPairsDist.resize(nact);

	std::vector<int> PointPairsID;	// 旧工程迁移注释。

	// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
	RMatrix::Matrix K;  // 旧工程迁移注释。
	RMatrix::Matrix cv_err;		// 旧工程迁移注释。

	// dimensionality 2
	if (m_dim == 2) {

		// extract matrix and translation vector
		double r00 = R.val[0][0]; double r01 = R.val[0][1];
		double r10 = R.val[1][0]; double r11 = R.val[1][1];
		double t0 = t.val[0][0]; double t1 = t.val[1][0];

		// establish correspondences
		/*int MaxThreadNum = omp_get_max_threads();
		omp_set_num_threads(MaxThreadNum);*/
#pragma omp parallel for private(i) default(none) shared(T,active,nact,r00,r01,r10,r11,t0,t1,AllPointPairsID,AllPointPairsDist) // schedule (dynamic,2)
		for (i = 0; i < nact; i++) {
			// kd tree query + result
			std::vector<float>         query(m_dim);
			kdtree::KDTreeResultVector result;

			// get index of active point
			int32_t idx = active[i];

			// transform point according to R|t
			query[0] = (float)(r00*T[idx * 2 + 0] + r01 * T[idx * 2 + 1] + t0);
			query[1] = (float)(r10*T[idx * 2 + 0] + r11 * T[idx * 2 + 1] + t1);

			// search nearest neighbor
			m_tree->n_nearest(query, 1, result);

			AllPointPairsID[2 * i + 0] = result[0].idx;
			AllPointPairsID[2 * i + 1] = i;
			AllPointPairsDist[i] = result[0].dis;
		}

		for (int i = 0; i < nact; i++)
		{
			if ((i > 0) && (AllPointPairsID[2 * i + 0] == LastClosestID_T2M))
			{
				if (AllPointPairsDist[i] < MinDist_M2T)
				{
					MinDist_M2T = AllPointPairsDist[i];
					PointPairsID.pop_back();
					PointPairsID.pop_back();
					PointPairsID.push_back(AllPointPairsID[2 * i + 0]);
					PointPairsID.push_back(AllPointPairsID[2 * i + 1]);
				}
			}
			else
			{
				MinDist_M2T = AllPointPairsDist[i];
				PointPairsID.push_back(AllPointPairsID[2 * i + 0]);
				PointPairsID.push_back(AllPointPairsID[2 * i + 1]);
			}

			LastClosestID_T2M = AllPointPairsID[2 * i + 0];
		}

		// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
		std::vector<double> DropPointsOnModel;
		for (int i = 0; i < PointPairsID.size() / 2; i++)
		{
			int tmp_id1 = 0, tmp_id2 = 0;
			std::vector<float> query(m_dim);
			double r00 = R.val[0][0]; double r01 = R.val[0][1];
			double r10 = R.val[1][0]; double r11 = R.val[1][1];
			double t0 = t.val[0][0]; double t1 = t.val[1][0];
			query[0] = (float)(r00*T[PointPairsID[2 * i + 1] * 2 + 0] + r01 * T[PointPairsID[2 * i + 1] * 2 + 1] + t0);
			query[1] = (float)(r10*T[PointPairsID[2 * i + 1] * 2 + 0] + r11 * T[PointPairsID[2 * i + 1] * 2 + 1] + t1);
			if (PointPairsID[2 * i + 0] == 0)
			{
				tmp_id1 = PointPairsID[2 * i + 0];
				tmp_id2 = tmp_id1 + 1;
			}
			else if (PointPairsID[2 * i + 0] == m_tree->N - 1)
			{
				tmp_id1 = PointPairsID[2 * i + 0] - 1;
				tmp_id2 = tmp_id1 + 1;
			}
			else
			{
				double dist_tmp1 = 0, dist_tmp2 = 0;
				dist_tmp1 = sqrt(pow(m_tree->the_data[PointPairsID[2 * i + 0] - 1][0] - query[0], 2)
					+ pow(m_tree->the_data[PointPairsID[2 * i + 0] - 1][1] - query[1], 2));
				dist_tmp2 = sqrt(pow(m_tree->the_data[PointPairsID[2 * i + 0] + 1][0] - query[0], 2)
					+ pow(m_tree->the_data[PointPairsID[2 * i + 0] + 1][1] - query[1], 2));
				if (dist_tmp1 < dist_tmp2)
				{
					tmp_id1 = PointPairsID[2 * i + 0] - 1;
					tmp_id2 = tmp_id1 + 1;
				}
				else
				{
					tmp_id1 = PointPairsID[2 * i + 0];
					tmp_id2 = tmp_id1 + 1;
				}
			}

			RMatrix::Matrix Vec_M1_D(2, 1), Vec_M1_M2(1, 2), Vec_M1_DP(1, 2);
			Vec_M1_D.val[0][0] = query[0] - m_tree->the_data[tmp_id1][0];
			Vec_M1_D.val[1][0] = query[1] - m_tree->the_data[tmp_id1][1];

			Vec_M1_M2.val[0][0] = m_tree->the_data[tmp_id2][0] - m_tree->the_data[tmp_id1][0];
			Vec_M1_M2.val[0][1] = m_tree->the_data[tmp_id2][1] - m_tree->the_data[tmp_id1][1];
			Vec_M1_M2 = Vec_M1_M2 / Vec_M1_M2.l2norm();

			Vec_M1_DP = (Vec_M1_M2 * Vec_M1_D) * Vec_M1_M2;

			DropPointsOnModel.push_back(m_tree->the_data[tmp_id1][0] + Vec_M1_DP.val[0][0]);
			DropPointsOnModel.push_back(m_tree->the_data[tmp_id1][1] + Vec_M1_DP.val[0][1]);
		}

		RMatrix::Matrix K_tmp(PointPairsID.size(), 2 * (m_dim + 1));
		K_tmp.zero();
		RMatrix::Matrix cv_err_tmp(m_dim*(PointPairsID.size() / 2), 1);
		cv_err_tmp.zero();
		K = K_tmp;
		cv_err = cv_err_tmp;
		for (int i = 0; i < PointPairsID.size() / 2; i++)
		{
			// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
			int idx = active[PointPairsID[2 * i + 1]];
			K.val[i][0] = (float)(r00*T[idx * 2 + 0] + r01 * T[idx * 2 + 1] + t0);
			K.val[i][1] = (float)(r10*T[idx * 2 + 0] + r11 * T[idx * 2 + 1] + t1);
			K.val[i + PointPairsID.size() / 2][0 + m_dim + 1] = (float)(r00*T[idx * 2 + 0] + r01 * T[idx * 2 + 1] + t0);
			K.val[i + PointPairsID.size() / 2][1 + m_dim + 1] = (float)(r10*T[idx * 2 + 0] + r11 * T[idx * 2 + 1] + t1);
			K.val[i][m_dim] = 1.0;
			K.val[i + PointPairsID.size() / 2][2 * m_dim + 1] = 1.0;
			// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
			for (int b = 0; b < m_dim; b++)
			{
				//cv_err.val[i + b * PointPairsID.size() / 2][0] = m_tree->the_data[PointPairsID[2*i+0]][b];
				cv_err.val[i + b * PointPairsID.size() / 2][0] = DropPointsOnModel[2 * i + b];
			}
		}
	}
	// dimensionality 3
	else {

		// extract matrix and translation vector
		double r00 = R.val[0][0]; double r01 = R.val[0][1]; double r02 = R.val[0][2];
		double r10 = R.val[1][0]; double r11 = R.val[1][1]; double r12 = R.val[1][2];
		double r20 = R.val[2][0]; double r21 = R.val[2][1]; double r22 = R.val[2][2];
		double t0 = t.val[0][0]; double t1 = t.val[1][0]; double t2 = t.val[2][0];
		/*double mum[3] = { 0.0 };
		double mut[3] = { 0.0 };*/

		// establish correspondences
		/*int MaxThreadNum = omp_get_max_threads();
		omp_set_num_threads(MaxThreadNum);*/
#pragma omp parallel for private(i) default(none) shared(T,active,nact,r00,r01,r02,r10,r11,r12,r20,r21,r22,t0,t1,t2,AllPointPairsID,AllPointPairsDist) // schedule (dynamic,2)
		for (i = 0; i < nact; i++) {
			// kd tree query + result
			std::vector<float>         query(m_dim);
			kdtree::KDTreeResultVector result;

			// get index of active point
			int32_t idx = active[i];

			// transform point according to R|t
			query[0] = (float)(r00*T[idx * 3 + 0] + r01 * T[idx * 3 + 1] + r02 * T[idx * 3 + 2] + t0);
			query[1] = (float)(r10*T[idx * 3 + 0] + r11 * T[idx * 3 + 1] + r12 * T[idx * 3 + 2] + t1);
			query[2] = (float)(r20*T[idx * 3 + 0] + r21 * T[idx * 3 + 1] + r22 * T[idx * 3 + 2] + t2);

			// search nearest neighbor
			m_tree->n_nearest(query, 1, result);

			AllPointPairsID[2 * i + 0] = result[0].idx;
			AllPointPairsID[2 * i + 1] = i;
			AllPointPairsDist[i] = result[0].dis;
		}

		for (int i = 0; i < nact; i++)
		{
			if ((i > 0) && (AllPointPairsID[2 * i + 0] == LastClosestID_T2M))
			{
				if (AllPointPairsDist[i] < MinDist_M2T)
				{
					MinDist_M2T = AllPointPairsDist[i];
					PointPairsID.pop_back();
					PointPairsID.pop_back();
					PointPairsID.push_back(AllPointPairsID[2 * i + 0]);
					PointPairsID.push_back(AllPointPairsID[2 * i + 1]);
				}
			}
			else
			{
				MinDist_M2T = AllPointPairsDist[i];
				PointPairsID.push_back(AllPointPairsID[2 * i + 0]);
				PointPairsID.push_back(AllPointPairsID[2 * i + 1]);
			}

			LastClosestID_T2M = AllPointPairsDist[i];
		}

		// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
		std::vector<double> DropPointsOnModel;
		for (int i = 0; i < PointPairsID.size() / 2; i++)
		{
			int tmp_id1 = 0, tmp_id2 = 0;
			std::vector<float> query(m_dim);
			double r00 = R.val[0][0]; double r01 = R.val[0][1]; double r02 = R.val[0][2];
			double r10 = R.val[1][0]; double r11 = R.val[1][1]; double r12 = R.val[1][2];
			double r20 = R.val[2][0]; double r21 = R.val[2][1]; double r22 = R.val[2][2];
			double t0 = t.val[0][0]; double t1 = t.val[1][0]; double t2 = t.val[2][0];
			query[0] = (float)(r00*T[PointPairsID[2 * i + 1] * 3 + 0] + r01 * T[PointPairsID[2 * i + 1] * 3 + 1] + r02 * T[PointPairsID[2 * i + 1] * 3 + 2] + t0);
			query[1] = (float)(r10*T[PointPairsID[2 * i + 1] * 3 + 0] + r11 * T[PointPairsID[2 * i + 1] * 3 + 1] + r12 * T[PointPairsID[2 * i + 1] * 3 + 2] + t1);
			query[2] = (float)(r20*T[PointPairsID[2 * i + 1] * 3 + 0] + r21 * T[PointPairsID[2 * i + 1] * 3 + 1] + r22 * T[PointPairsID[2 * i + 1] * 3 + 2] + t2);
			if (PointPairsID[2 * i + 0] == 0)
			{
				tmp_id1 = PointPairsID[2 * i + 0];
				tmp_id2 = tmp_id1 + 1;
			}
			else if (PointPairsID[2 * i + 0] == m_tree->N - 1)
			{
				tmp_id1 = PointPairsID[2 * i + 0] - 1;
				tmp_id2 = tmp_id1 + 1;
			}
			else
			{
				double dist_tmp1 = 0, dist_tmp2 = 0;
				dist_tmp1 = sqrt(pow(m_tree->the_data[PointPairsID[2 * i + 0] - 1][0] - query[0], 2)
					+ pow(m_tree->the_data[PointPairsID[2 * i + 0] - 1][1] - query[1], 2)
					+ pow(m_tree->the_data[PointPairsID[2 * i + 0] - 1][2] - query[2], 2));
				dist_tmp2 = sqrt(pow(m_tree->the_data[PointPairsID[2 * i + 0] + 1][0] - query[0], 2)
					+ pow(m_tree->the_data[PointPairsID[2 * i + 0] + 1][1] - query[1], 2)
					+ pow(m_tree->the_data[PointPairsID[2 * i + 0] + 1][2] - query[2], 2));
				if (dist_tmp1 < dist_tmp2)
				{
					tmp_id1 = PointPairsID[2 * i + 0] - 1;
					tmp_id2 = tmp_id1 + 1;
				}
				else
				{
					tmp_id1 = PointPairsID[2 * i + 0];
					tmp_id2 = tmp_id1 + 1;
				}
			}

			RMatrix::Matrix Vec_M1_D(3, 1), Vec_M1_M2(1, 3), Vec_M1_DP(1, 3);
			Vec_M1_D.val[0][0] = query[0] - m_tree->the_data[tmp_id1][0];
			Vec_M1_D.val[1][0] = query[1] - m_tree->the_data[tmp_id1][1];
			Vec_M1_D.val[2][0] = query[2] - m_tree->the_data[tmp_id1][2];

			Vec_M1_M2.val[0][0] = m_tree->the_data[tmp_id2][0] - m_tree->the_data[tmp_id1][0];
			Vec_M1_M2.val[0][1] = m_tree->the_data[tmp_id2][1] - m_tree->the_data[tmp_id1][1];
			Vec_M1_M2.val[0][2] = m_tree->the_data[tmp_id2][2] - m_tree->the_data[tmp_id1][2];
			Vec_M1_M2 = Vec_M1_M2 / Vec_M1_M2.l2norm();

			Vec_M1_DP = (Vec_M1_M2 * Vec_M1_D) * Vec_M1_M2;

			DropPointsOnModel.push_back(m_tree->the_data[tmp_id1][0] + Vec_M1_DP.val[0][0]);
			DropPointsOnModel.push_back(m_tree->the_data[tmp_id1][1] + Vec_M1_DP.val[0][1]);
			DropPointsOnModel.push_back(m_tree->the_data[tmp_id1][2] + Vec_M1_DP.val[0][2]);
		}

		RMatrix::Matrix K_tmp(PointPairsID.size(), 2 * (m_dim + 1));
		K_tmp.zero();
		RMatrix::Matrix cv_err_tmp(m_dim*(PointPairsID.size() / 2), 1);
		cv_err_tmp.zero();
		K = K_tmp;
		cv_err = cv_err_tmp;
		for (int i = 0; i < PointPairsID.size() / 2; i++)
		{
			// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
			int idx = active[PointPairsID[2 * i + 1]];
			K.val[i][0] = (float)(r00*T[idx * 3 + 0] + r01 * T[idx * 3 + 1] + r02 * T[idx * 3 + 2] + t0);
			K.val[i][1] = (float)(r10*T[idx * 3 + 0] + r11 * T[idx * 3 + 1] + r12 * T[idx * 3 + 2] + t1);
			K.val[i][2] = (float)(r20*T[idx * 3 + 0] + r21 * T[idx * 3 + 1] + r22 * T[idx * 3 + 2] + t2);
			K.val[i + PointPairsID.size() / 2][0 + m_dim + 1] = (float)(r00*T[idx * 3 + 0] + r01 * T[idx * 3 + 1] + r02 * T[idx * 3 + 2] + t0);
			K.val[i + PointPairsID.size() / 2][1 + m_dim + 1] = (float)(r10*T[idx * 3 + 0] + r11 * T[idx * 3 + 1] + r12 * T[idx * 3 + 2] + t1);
			K.val[i + PointPairsID.size() / 2][2 + m_dim + 1] = (float)(r20*T[idx * 3 + 0] + r21 * T[idx * 3 + 1] + r22 * T[idx * 3 + 2] + t2);
			K.val[i][m_dim] = 1.0;
			K.val[i + PointPairsID.size() / 2][2 * m_dim + 1] = 1.0;
			// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
			for (int b = 0; b < m_dim; b++)
			{
				//cv_err.val[i + b * PointPairsID.size() / 2][0] = m_tree->the_data[PointPairsID[2 * i + 0]][b];
				cv_err.val[i + b * PointPairsID.size() / 2][0] = DropPointsOnModel[2 * i + b];
			}
		}
	}

	RMatrix::Matrix ParaVec = (RMatrix::Matrix::inv(~K*K))*(~K)*cv_err;
	RMatrix::Matrix TransMat = RMatrix::Matrix::eye(m_dim + 1);
	for (int i = 0; i < m_dim; i++)
	{
		for (int j = 0; j < m_dim + 1; j++)
		{
			TransMat.val[i][j] = ParaVec.val[i*(m_dim + 1) + j][0];
		}
	}

	RMatrix::Matrix R_ = TransMat.getMat(0, 0, m_dim - 1, m_dim - 1);
	RMatrix::Matrix t_ = TransMat.getMat(0, m_dim, m_dim - 1, m_dim);

	// compose: R|t = R_|t_ * R|t
	R = R_ * R;
	t = R_ * t + t_;

	double delta = 0;
	if (m_dim == 2)
	{
		delta = max((R_ - RMatrix::Matrix::eye(2)).l2norm(), t_.l2norm());
	}
	else
	{
		delta = max((R_ - RMatrix::Matrix::eye(3)).l2norm(), t_.l2norm());
	}

	// return max delta in parameters
	if (m_dim == 2) return max((R_ - RMatrix::Matrix::eye(2)).l2norm(), t_.l2norm());
	else        return max((R_ - RMatrix::Matrix::eye(3)).l2norm(), t_.l2norm());
}

// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
void CIcpPointToPoint::EvaluateMatchResultBDD(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const std::vector<int32_t> &active, double &MaxDist, double &MinDist, double &MeanDist, double &MsrDist)
{
	int i;
	int nact = (int)active.size();
	int LastClosestID_T2M = 0;	// 旧工程迁移注释。
	double MinDist_M2T = 0;	// 旧工程迁移注释。
	std::vector<double> Dist_P2P;	// 旧工程迁移注释。
	std::vector<int> AllPointPairsID;	// 旧工程迁移注释。
	std::vector<double> AllPointPairsDist;	// 旧工程迁移注释。
	AllPointPairsID.resize(2 * nact);
	AllPointPairsDist.resize(nact);

	std::vector<int> PointPairsID;	// 旧工程迁移注释。

	// dimensionality 2
	if (m_dim == 2) {

		// extract matrix and translation vector
		double r00 = R.val[0][0]; double r01 = R.val[0][1];
		double r10 = R.val[1][0]; double r11 = R.val[1][1];
		double t0 = t.val[0][0]; double t1 = t.val[1][0];

		// establish correspondences
		/*int MaxThreadNum = omp_get_max_threads();
		omp_set_num_threads(MaxThreadNum);*/
#pragma omp parallel for private(i) default(none) shared(T,active,nact,r00,r01,r10,r11,t0,t1,AllPointPairsID,AllPointPairsDist) // schedule (dynamic,2)
		for (i = 0; i < nact; i++) {
			// kd tree query + result
			std::vector<float>         query(m_dim);
			kdtree::KDTreeResultVector result;

			// get index of active point
			int32_t idx = active[i];

			// transform point according to R|t
			query[0] = (float)(r00*T[idx * 2 + 0] + r01 * T[idx * 2 + 1] + t0);
			query[1] = (float)(r10*T[idx * 2 + 0] + r11 * T[idx * 2 + 1] + t1);

			// search nearest neighbor
			m_tree->n_nearest(query, 1, result);

			AllPointPairsID[2 * i + 0] = result[0].idx;
			AllPointPairsID[2 * i + 1] = i;
			AllPointPairsDist[i] = result[0].dis;
		}

		for (int i = 0; i < nact; i++)
		{
			if ((i > 0) && (AllPointPairsID[2 * i + 0] == LastClosestID_T2M))
			{
				if (AllPointPairsDist[i] < MinDist_M2T)
				{
					MinDist_M2T = AllPointPairsDist[i];
					PointPairsID.pop_back();
					PointPairsID.pop_back();
					PointPairsID.push_back(AllPointPairsID[2 * i + 0]);
					PointPairsID.push_back(AllPointPairsID[2 * i + 1]);
				}
			}
			else
			{
				MinDist_M2T = AllPointPairsDist[i];
				PointPairsID.push_back(AllPointPairsID[2 * i + 0]);
				PointPairsID.push_back(AllPointPairsID[2 * i + 1]);
			}

			LastClosestID_T2M = AllPointPairsDist[i];
		}
		// dimensionality 3
	}
	else {

		// extract matrix and translation vector
		double r00 = R.val[0][0]; double r01 = R.val[0][1]; double r02 = R.val[0][2];
		double r10 = R.val[1][0]; double r11 = R.val[1][1]; double r12 = R.val[1][2];
		double r20 = R.val[2][0]; double r21 = R.val[2][1]; double r22 = R.val[2][2];
		double t0 = t.val[0][0]; double t1 = t.val[1][0]; double t2 = t.val[2][0];

		// establish correspondences
		/*int MaxThreadNum = omp_get_max_threads();
		omp_set_num_threads(MaxThreadNum);*/
#pragma omp parallel for private(i) default(none) shared(T,active,nact,r00,r01,r02,r10,r11,r12,r20,r21,r22,t0,t1,t2,AllPointPairsID,AllPointPairsDist) // schedule (dynamic,2)
		for (i = 0; i < nact; i++) {
			// kd tree query + result
			std::vector<float>         query(m_dim);
			kdtree::KDTreeResultVector result;

			// get index of active point
			int32_t idx = active[i];

			// transform point according to R|t
			query[0] = (float)(r00*T[idx * 3 + 0] + r01 * T[idx * 3 + 1] + r02 * T[idx * 3 + 2] + t0);
			query[1] = (float)(r10*T[idx * 3 + 0] + r11 * T[idx * 3 + 1] + r12 * T[idx * 3 + 2] + t1);
			query[2] = (float)(r20*T[idx * 3 + 0] + r21 * T[idx * 3 + 1] + r22 * T[idx * 3 + 2] + t2);

			// search nearest neighbor
			m_tree->n_nearest(query, 1, result);

			AllPointPairsID[2 * i + 0] = result[0].idx;
			AllPointPairsID[2 * i + 1] = i;
			AllPointPairsDist[i] = result[0].dis;
		}

		for (int i = 0; i < nact; i++)
		{
			if ((i > 0) && (AllPointPairsID[2 * i + 0] == LastClosestID_T2M))
			{
				if (AllPointPairsDist[i] < MinDist_M2T)
				{
					MinDist_M2T = AllPointPairsDist[i];
					PointPairsID.pop_back();
					PointPairsID.pop_back();
					PointPairsID.push_back(AllPointPairsID[2 * i + 0]);
					PointPairsID.push_back(AllPointPairsID[2 * i + 1]);
				}
			}
			else
			{
				MinDist_M2T = AllPointPairsDist[i];
				PointPairsID.push_back(AllPointPairsID[2 * i + 0]);
				PointPairsID.push_back(AllPointPairsID[2 * i + 1]);
			}

			LastClosestID_T2M = AllPointPairsDist[i];
		}
	}

	MaxDist = 0;
	MinDist = 10000;
	MeanDist = 0;
	MsrDist = 0;
	for (int i = 0; i < PointPairsID.size() / 2; i++)
	{
		int tmp_id1 = 0, tmp_id2 = 0;
		std::vector<float> query(m_dim);
		if (m_dim == 2)
		{
			double r00 = R.val[0][0]; double r01 = R.val[0][1];
			double r10 = R.val[1][0]; double r11 = R.val[1][1];
			double t0 = t.val[0][0]; double t1 = t.val[1][0];
			query[0] = (float)(r00*T[PointPairsID[2 * i + 1] * 2 + 0] + r01 * T[PointPairsID[2 * i + 1] * 2 + 1] + t0);
			query[1] = (float)(r10*T[PointPairsID[2 * i + 1] * 2 + 0] + r11 * T[PointPairsID[2 * i + 1] * 2 + 1] + t1);
		}
		else
		{
			double r00 = R.val[0][0]; double r01 = R.val[0][1]; double r02 = R.val[0][2];
			double r10 = R.val[1][0]; double r11 = R.val[1][1]; double r12 = R.val[1][2];
			double r20 = R.val[2][0]; double r21 = R.val[2][1]; double r22 = R.val[2][2];
			double t0 = t.val[0][0]; double t1 = t.val[1][0]; double t2 = t.val[2][0];
			query[0] = (float)(r00*T[PointPairsID[2 * i + 1] * 3 + 0] + r01 * T[PointPairsID[2 * i + 1] * 3 + 1] + r02 * T[PointPairsID[2 * i + 1] * 3 + 2] + t0);
			query[1] = (float)(r10*T[PointPairsID[2 * i + 1] * 3 + 0] + r11 * T[PointPairsID[2 * i + 1] * 3 + 1] + r12 * T[PointPairsID[2 * i + 1] * 3 + 2] + t1);
			query[2] = (float)(r20*T[PointPairsID[2 * i + 1] * 3 + 0] + r21 * T[PointPairsID[2 * i + 1] * 3 + 1] + r22 * T[PointPairsID[2 * i + 1] * 3 + 2] + t2);
		}
		if (PointPairsID[2 * i + 0] == 0)
		{
			tmp_id1 = PointPairsID[2 * i + 0];
			tmp_id2 = tmp_id1 + 1;
		}
		else if (PointPairsID[2 * i + 0] == m_tree->N - 1)
		{
			tmp_id1 = PointPairsID[2 * i + 0] - 1;
			tmp_id2 = tmp_id1 + 1;
		}
		else
		{
			double dist_tmp1 = 0, dist_tmp2 = 0;
			if (m_dim == 2)
			{
				dist_tmp1 = sqrt(pow(m_tree->the_data[PointPairsID[2 * i + 0] - 1][0] - query[0], 2)
					+ pow(m_tree->the_data[PointPairsID[2 * i + 0] - 1][1] - query[1], 2));
				dist_tmp2 = sqrt(pow(m_tree->the_data[PointPairsID[2 * i + 0] + 1][0] - query[0], 2)
					+ pow(m_tree->the_data[PointPairsID[2 * i + 0] + 1][1] - query[1], 2));
				if (dist_tmp1 < dist_tmp2)
				{
					tmp_id1 = PointPairsID[2 * i + 0] - 1;
					tmp_id2 = tmp_id1 + 1;
				}
				else
				{
					tmp_id1 = PointPairsID[2 * i + 0];
					tmp_id2 = tmp_id1 + 1;
				}
			}
			else
			{
				dist_tmp1 = sqrt(pow(m_tree->the_data[PointPairsID[2 * i + 0] - 1][0] - query[0], 2)
					+ pow(m_tree->the_data[PointPairsID[2 * i + 0] - 1][1] - query[1], 2)
					+ pow(m_tree->the_data[PointPairsID[2 * i + 0] - 1][2] - query[2], 2));
				dist_tmp2 = sqrt(pow(m_tree->the_data[PointPairsID[2 * i + 0] + 1][0] - query[0], 2)
					+ pow(m_tree->the_data[PointPairsID[2 * i + 0] + 1][1] - query[1], 2)
					+ pow(m_tree->the_data[PointPairsID[2 * i + 0] + 1][2] - query[2], 2));
				if (dist_tmp1 < dist_tmp2)
				{
					tmp_id1 = PointPairsID[2 * i + 0] - 1;
					tmp_id2 = tmp_id1 + 1;
				}
				else
				{
					tmp_id1 = PointPairsID[2 * i + 0];
					tmp_id2 = tmp_id1 + 1;
				}
			}
		}

		double dist = 0;
		if (m_dim == 2)
		{
			RMatrix::Matrix Vec_M1_D(3, 1), Vec_M2_D(3, 1), Vec_M1_M2(3, 1);
			RMatrix::Matrix crossVec12(3, 1);
			Vec_M1_D.val[0][0] = m_tree->the_data[tmp_id1][0] - query[0];
			Vec_M1_D.val[1][0] = m_tree->the_data[tmp_id1][1] - query[1];
			Vec_M1_D.val[2][0] = 0;
			Vec_M2_D.val[0][0] = m_tree->the_data[tmp_id2][0] - query[0];
			Vec_M2_D.val[1][0] = m_tree->the_data[tmp_id2][1] - query[1];
			Vec_M2_D.val[2][0] = 0;
			Vec_M1_M2.val[0][0] = m_tree->the_data[tmp_id1][0] - m_tree->the_data[tmp_id2][0];
			Vec_M1_M2.val[1][0] = m_tree->the_data[tmp_id1][1] - m_tree->the_data[tmp_id2][1];
			Vec_M1_M2.val[2][0] = 0;
			crossVec12 = crossVec12.cross(Vec_M2_D, Vec_M1_D);
			dist = abs((crossVec12.l2norm() / Vec_M1_M2.l2norm()));
		}
		else
		{
			RMatrix::Matrix Vec_M1_D(3, 1), Vec_M2_D(3, 1), Vec_M1_M2(3, 1);
			Vec_M1_D.val[0][0] = m_tree->the_data[tmp_id1][0] - query[0];
			Vec_M1_D.val[1][0] = m_tree->the_data[tmp_id1][1] - query[1];
			Vec_M1_D.val[2][0] = m_tree->the_data[tmp_id1][2] - query[2];
			Vec_M2_D.val[0][0] = m_tree->the_data[tmp_id2][0] - query[0];
			Vec_M2_D.val[1][0] = m_tree->the_data[tmp_id2][1] - query[1];
			Vec_M2_D.val[2][0] = m_tree->the_data[tmp_id2][2] - query[2];
			Vec_M1_M2.val[0][0] = m_tree->the_data[tmp_id1][0] - m_tree->the_data[tmp_id2][0];
			Vec_M1_M2.val[1][0] = m_tree->the_data[tmp_id1][1] - m_tree->the_data[tmp_id2][1];
			Vec_M1_M2.val[2][0] = m_tree->the_data[tmp_id1][2] - m_tree->the_data[tmp_id2][2];
			dist = abs((Vec_M1_D.cross(Vec_M2_D, Vec_M1_D).l2norm() / Vec_M1_M2.l2norm()));
		}
		if (dist > MaxDist)
			MaxDist = dist;
		if (dist < MinDist)
			MinDist = dist;
		MeanDist += dist;
		MsrDist += dist * dist;
	}
	MeanDist = MeanDist / (PointPairsID.size() / 2);
	MsrDist = sqrt(MsrDist / (PointPairsID.size() / 2));
}

double CIcpPointToPoint::fitStepBDD(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const std::vector<int32_t> &active) {

	int i;
	int nact = (int)active.size();

	int LastClosestID_T2M = 0;	// 旧工程迁移注释。
	double MinDist_M2T = 0;	// 旧工程迁移注释。
	std::vector<int> AllPointPairsID;	// 旧工程迁移注释。
	std::vector<double> AllPointPairsDist;	// 旧工程迁移注释。
	AllPointPairsID.resize(2 * nact);
	AllPointPairsDist.resize(nact);

	std::vector<int> PointPairsID;	// 旧工程迁移注释。

	// init matrix for point correspondences
	RMatrix::Matrix p_m; // model
	RMatrix::Matrix p_t; // template

	// init mean
	RMatrix::Matrix mu_m(1, m_dim);
	RMatrix::Matrix mu_t(1, m_dim);

	// dimensionality 2
	if (m_dim == 2) {

		// extract matrix and translation vector
		double r00 = R.val[0][0]; double r01 = R.val[0][1];
		double r10 = R.val[1][0]; double r11 = R.val[1][1];
		double t0 = t.val[0][0]; double t1 = t.val[1][0];
		double mum0 = 0.0, mum1 = 0.0;
		double mut0 = 0.0, mut1 = 0.0;

		// establish correspondences
		/*int MaxThreadNum = omp_get_max_threads();
		omp_set_num_threads(MaxThreadNum);*/
#pragma omp parallel for private(i) default(none) shared(T,active,nact,r00,r01,r10,r11,t0,t1,AllPointPairsID,AllPointPairsDist) reduction(+:mum0,mum1, mut0,mut1) // schedule (dynamic,2)
		for (i = 0; i < nact; i++) {
			// kd tree query + result
			std::vector<float>         query(m_dim);
			kdtree::KDTreeResultVector result;

			// get index of active point
			int32_t idx = active[i];

			// transform point according to R|t
			query[0] = (float)(r00*T[idx * 2 + 0] + r01 * T[idx * 2 + 1] + t0);
			query[1] = (float)(r10*T[idx * 2 + 0] + r11 * T[idx * 2 + 1] + t1);

			// search nearest neighbor
			m_tree->n_nearest(query, 1, result);

			AllPointPairsID[2 * i + 0] = result[0].idx;
			AllPointPairsID[2 * i + 1] = i;
			AllPointPairsDist[i] = result[0].dis;
		}

		for (int i = 0; i < nact; i++)
		{
			if ((i > 0) && (AllPointPairsID[2 * i + 0] == LastClosestID_T2M))
			{
				if (AllPointPairsDist[i] < MinDist_M2T)
				{
					MinDist_M2T = AllPointPairsDist[i];
					PointPairsID.pop_back();
					PointPairsID.pop_back();
					PointPairsID.push_back(AllPointPairsID[2 * i + 0]);
					PointPairsID.push_back(AllPointPairsID[2 * i + 1]);
				}
			}
			else
			{
				MinDist_M2T = AllPointPairsDist[i];
				PointPairsID.push_back(AllPointPairsID[2 * i + 0]);
				PointPairsID.push_back(AllPointPairsID[2 * i + 1]);
			}

			LastClosestID_T2M = AllPointPairsDist[i];
		}

		RMatrix::Matrix p_m_tmp(PointPairsID.size() / 2, m_dim);
		p_m = p_m_tmp;
		RMatrix::Matrix p_t_tmp(PointPairsID.size() / 2, m_dim);
		p_t = p_t_tmp;
		for (int i = 0; i < PointPairsID.size() / 2; i++)
		{
			// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
			p_m.val[i][0] = m_tree->the_data[PointPairsID[2 * i + 0]][0]; mum0 += p_m.val[i][0];
			p_m.val[i][1] = m_tree->the_data[PointPairsID[2 * i + 0]][1]; mum1 += p_m.val[i][1];

			// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
			int idx = active[PointPairsID[2 * i + 1]];
			p_t.val[i][0] = (float)(r00*T[idx * 2 + 0] + r01 * T[idx * 2 + 1] + t0); mut0 += p_t.val[i][0];
			p_t.val[i][1] = (float)(r10*T[idx * 2 + 0] + r11 * T[idx * 2 + 1] + t1); mut1 += p_t.val[i][1];
		}

		mu_m.val[0][0] = mum0;
		mu_m.val[0][1] = mum1;

		mu_t.val[0][0] = mut0;
		mu_t.val[0][1] = mut1;

		// dimensionality 3
	}
	else {

		// extract matrix and translation vector
		double r00 = R.val[0][0]; double r01 = R.val[0][1]; double r02 = R.val[0][2];
		double r10 = R.val[1][0]; double r11 = R.val[1][1]; double r12 = R.val[1][2];
		double r20 = R.val[2][0]; double r21 = R.val[2][1]; double r22 = R.val[2][2];
		double t0 = t.val[0][0]; double t1 = t.val[1][0]; double t2 = t.val[2][0];
		double mum0 = 0.0, mum1 = 0.0, mum2 = 0.0;
		double mut0 = 0.0, mut1 = 0.0, mut2 = 0.0;

		// establish correspondences
		/*int MaxThreadNum = omp_get_max_threads();
		omp_set_num_threads(MaxThreadNum);*/
#pragma omp parallel for private(i) default(none) shared(T,active,nact,r00,r01,r02,r10,r11,r12,r20,r21,r22,t0,t1,t2,AllPointPairsID,AllPointPairsDist) reduction(+:mum0,mum1,mum2, mut0,mut1,mut2) // schedule (dynamic,2)
		for (i = 0; i < nact; i++) {
			// kd tree query + result
			std::vector<float>         query(m_dim);
			kdtree::KDTreeResultVector result;

			// get index of active point
			int32_t idx = active[i];

			// transform point according to R|t
			query[0] = (float)(r00*T[idx * 3 + 0] + r01 * T[idx * 3 + 1] + r02 * T[idx * 3 + 2] + t0);
			query[1] = (float)(r10*T[idx * 3 + 0] + r11 * T[idx * 3 + 1] + r12 * T[idx * 3 + 2] + t1);
			query[2] = (float)(r20*T[idx * 3 + 0] + r21 * T[idx * 3 + 1] + r22 * T[idx * 3 + 2] + t2);

			// search nearest neighbor
			m_tree->n_nearest(query, 1, result);

			AllPointPairsID[2 * i + 0] = result[0].idx;
			AllPointPairsID[2 * i + 1] = i;
			AllPointPairsDist[i] = result[0].dis;
		}

		for (int i = 0; i < nact; i++)
		{
			if ((i > 0) && (AllPointPairsID[2 * i + 0] == LastClosestID_T2M))
			{
				if (AllPointPairsDist[i] < MinDist_M2T)
				{
					MinDist_M2T = AllPointPairsDist[i];
					PointPairsID.pop_back();
					PointPairsID.pop_back();
					PointPairsID.push_back(AllPointPairsID[2 * i + 0]);
					PointPairsID.push_back(AllPointPairsID[2 * i + 1]);
				}
			}
			else
			{
				MinDist_M2T = AllPointPairsDist[i];
				PointPairsID.push_back(AllPointPairsID[2 * i + 0]);
				PointPairsID.push_back(AllPointPairsID[2 * i + 1]);
			}

			LastClosestID_T2M = AllPointPairsDist[i];
		}

		RMatrix::Matrix p_m_tmp(PointPairsID.size() / 2, m_dim);
		p_m = p_m_tmp;
		RMatrix::Matrix p_t_tmp(PointPairsID.size() / 2, m_dim);
		p_t = p_t_tmp;
		for (int i = 0; i < PointPairsID.size() / 2; i++)
		{
			// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
			p_m.val[i][0] = m_tree->the_data[PointPairsID[2 * i + 0]][0]; mum0 += p_m.val[i][0];
			p_m.val[i][1] = m_tree->the_data[PointPairsID[2 * i + 0]][1]; mum1 += p_m.val[i][1];
			p_m.val[i][2] = m_tree->the_data[PointPairsID[2 * i + 0]][2]; mum1 += p_m.val[i][2];
			// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
			int idx = active[PointPairsID[2 * i + 1]];
			p_t.val[i][0] = (float)(r00*T[idx * 3 + 0] + r01 * T[idx * 3 + 1] + r02 * T[idx * 3 + 2] + t0); mut0 += p_t.val[i][0];
			p_t.val[i][1] = (float)(r10*T[idx * 3 + 0] + r11 * T[idx * 3 + 1] + r12 * T[idx * 3 + 2] + t1); mut1 += p_t.val[i][1];
			p_t.val[i][2] = (float)(r20*T[idx * 3 + 0] + r21 * T[idx * 3 + 1] + r22 * T[idx * 3 + 2] + t2); mut2 += p_t.val[i][2];
		}

		mu_m.val[0][0] = mum0;
		mu_m.val[0][1] = mum1;
		mu_m.val[0][2] = mum2;

		mu_t.val[0][0] = mut0;
		mu_t.val[0][1] = mut1;
		mu_t.val[0][2] = mut2;
	}

	// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
	mu_m = mu_m / (double)(PointPairsID.size() / 2);
	mu_t = mu_t / (double)(PointPairsID.size() / 2);
	RMatrix::Matrix q_m = p_m - RMatrix::Matrix::ones(PointPairsID.size() / 2, 1)*mu_m;
	RMatrix::Matrix q_t = p_t - RMatrix::Matrix::ones(PointPairsID.size() / 2, 1)*mu_t;

	// compute relative rotation matrix R and translation vector t
	RMatrix::Matrix H = ~q_t*q_m;
	RMatrix::Matrix U, W, V;
	H.svd(U, W, V);
	RMatrix::Matrix R_ = V * ~U;

	// fix improper matrix problem
	if (R_.det() < 0) {
		RMatrix::Matrix B = RMatrix::Matrix::eye(m_dim);
		B.val[m_dim - 1][m_dim - 1] = R_.det();
		R_ = V * B*~U;
	}

	RMatrix::Matrix t_ = ~mu_m - R_ * ~mu_t;

	// compose: R|t = R_|t_ * R|t
	R = R_ * R;
	t = R_ * t + t_;

	// return max delta in parameters
	if (m_dim == 2) return max((R_ - RMatrix::Matrix::eye(2)).l2norm(), t_.l2norm());
	else        return max((R_ - RMatrix::Matrix::eye(3)).l2norm(), t_.l2norm());
}



