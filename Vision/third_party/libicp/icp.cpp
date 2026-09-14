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
 * @file icp.cpp
 * @brief libicp 基类迭代流程实现。
 * @details 旧工程迁入文件；当前仅补充文档和 include 顺序，不改变 ICP 迭代逻辑。
 */

#include "icp.h"

using namespace std;

CIcp::CIcp(double *M, const int32_t M_num, const int32_t m_dim, int32_t m_maxIter, double m_minDelta) :
	m_dim(m_dim), m_maxIter(m_maxIter), m_minDelta(m_minDelta) {
	m_currentIter = 0;
	// check for correct dimensionality
	if (m_dim != 2 && m_dim != 3) {
		cout << "ERROR: LIBICP works only for data of dimensionality 2 or 3" << endl;
		m_tree = 0;
		return;
	}

	// check for minimum number of points
	if (M_num < 5) {
		cout << "ERROR: LIBICP works only with at least 5 model points" << endl;
		m_tree = 0;
		return;
	}

	// copy model points to m_data
	m_data.resize(boost::extents[M_num][m_dim]);
	for (int32_t m = 0; m < M_num; m++)
		for (int32_t n = 0; n < m_dim; n++)
			m_data[m][n] = (float)M[m*m_dim + n];

	// build a kd tree from the model point cloud
	m_tree = new kdtree::KDTree(m_data);
}

CIcp::~CIcp() {
	if (m_tree)
		delete m_tree;
}

void CIcp::fit(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const double indist) {

	// make sure we have a model tree
	if (!m_tree) {
		cout << "ERROR: No model available." << endl;
		return;
	}

	// check for minimum number of points
	if (T_num < 5) {
		cout << "ERROR: CIcp works only with at least 5 template points" << endl;
		return;
	}

	// set active points
	vector<int32_t> active;
	if (indist <= 0) {
		active.clear();
		for (int32_t i = 0; i < T_num; i++)
			active.push_back(i);
	}
	else {
		active = getInliers(T, T_num, R, t, indist);
	}

	// run CIcp
	fitIterate(T, T_num, R, t, active);
}

//void CIcp::fitRemoveR(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const double indist,const int m_keepRotation) {
//
//	// make sure we have a model tree
//	if (!m_tree) {
//		cout << "ERROR: No model available." << endl;
//		return;
//	}
//
//	// check for minimum number of points
//	if (T_num < 5) {
//		cout << "ERROR: CIcp works only with at least 5 template points" << endl;
//		return;
//	}
//
//	// set active points
//	vector<int32_t> active;
//	if (indist <= 0) {
//		active.clear();
//		for (int32_t i = 0; i < T_num; i++)
//			active.push_back(i);
//	}
//	else {
//		active = getInliers(T, T_num, R, t, indist);
//	}
//
//	// run CIcp
//	fitIterateRemoveR(T, T_num, R, t, active,m_keepRotation);
// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
//	double* Rvalue = new double[m_dim*m_dim];
//	R.getData(Rvalue);
//	FILE*fp = fopen((SRobotleading_data_path + "ICP_R_VALUE.txt").data(), "a+");
//	for (int i = 0;i < m_dim;++i)
//	{
//		for (int j = 0;j < m_dim;++j)
//		{
//			fprintf(fp, "%f  ", Rvalue[3 * i + j]);
//		}
//		fprintf(fp, "\n ");
//	}
//	fclose(fp);
//}


void CIcp::fit_scale(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, RMatrix::Matrix& S, const double indist) {

	// make sure we have a model tree
	if (!m_tree) {
		cout << "ERROR: No model available." << endl;
		return;
	}

	// check for minimum number of points
	if (T_num < 5) {
		cout << "ERROR: CIcp works only with at least 5 template points" << endl;
		return;
	}

	// set active points
	vector<int32_t> active;
	if (indist <= 0) {
		active.clear();
		for (int32_t i = 0; i < T_num; i++)
			active.push_back(i);
	}
	else {
		active = getInliers(T, T_num, R, t, indist);
	}

	// run CIcp
	fitIterate_scale(T, T_num, R, t, S, active);
}

void CIcp::fitBDD(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const double indist) {

	// make sure we have a model tree
	if (!m_tree) {
		cout << "ERROR: No model available." << endl;
		return;
	}

	// check for minimum number of points
	if (T_num < 5) {
		cout << "ERROR: CIcp works only with at least 5 template points" << endl;
		return;
	}

	// set active points
	vector<int32_t> active;
	if (indist <= 0) {
		active.clear();
		for (int32_t i = 0; i < T_num; i++)
			active.push_back(i);
	}
	else {
		active = getInliers(T, T_num, R, t, indist);
	}

	// run CIcp
	fitIterateBDD(T, T_num, R, t, active);
}

// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
void CIcp::fitAffine(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const double indist, double &MaxDist, double &MinDist, double &MeanDist, double &MsrDist)
{
	// make sure we have a model tree
	if (!m_tree) {
		cout << "ERROR: No model available." << endl;
		return;
	}

	// check for minimum number of points
	if (T_num < 5) {
		cout << "ERROR: CIcp works only with at least 5 template points" << endl;
		return;
	}

	// set active points
	vector<int32_t> active;
	if (indist <= 0) {
		active.clear();
		for (int32_t i = 0; i < T_num; i++)
			active.push_back(i);
	}
	else {
		active = getInliers(T, T_num, R, t, indist);
	}

	// run CIcp
	fitIterateAffine(T, T_num, R, t, active, MaxDist, MinDist, MeanDist, MsrDist);
}

void CIcp::fitAffineBDD(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const double indist, double &MaxDist, double &MinDist, double &MeanDist, double &MsrDist)
{
	// make sure we have a model tree
	if (!m_tree) {
		cout << "ERROR: No model available." << endl;
		return;
	}

	// check for minimum number of points
	if (T_num < 5) {
		cout << "ERROR: CIcp works only with at least 5 template points" << endl;
		return;
	}

	// set active points
	vector<int32_t> active;
	if (indist <= 0) {
		active.clear();
		for (int32_t i = 0; i < T_num; i++)
			active.push_back(i);
	}
	else {
		active = getInliers(T, T_num, R, t, indist);
	}

	// run CIcp
	fitIterateAffineBDD(T, T_num, R, t, active, MaxDist, MinDist, MeanDist, MsrDist);
}

void CIcp::fitIterate(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const std::vector<int32_t> &active) {

	// check if we have at least 5 active points
	if (active.size() < 5)
		return;

	// iterate until convergence
	for (int32_t iter = 0; iter < m_maxIter; iter++)
		if (fitStep(T, T_num, R, t, active) < m_minDelta)
			break;
}

//void CIcp::fitIterateRemoveR(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const std::vector<int32_t> &active,const int m_keepRotation) {
//
//	// check if we have at least 5 active points
//	if (active.size() < 5)
//		return;
//
//	// iterate until convergence
//	for (int32_t iter = 0; iter < m_maxIter; iter++)
//		if (fitStepRemoveR(T, T_num, R, t, active,m_keepRotation) < m_minDelta)
//			break;
//}

void CIcp::fitIterate_scale(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, RMatrix::Matrix&S, const std::vector<int32_t> &active) {

	// check if we have at least 5 active points
	if (active.size() < 5)
		return;

	// iterate until convergence
	for (int32_t iter = 0; iter < m_maxIter; iter++)
	{
		m_currentIter = iter;
		if (fitStep_scale(T, T_num, R, t, S, active) < m_minDelta)
			break;
	}

}

// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
void CIcp::fitIterateAffine(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, std::vector<int32_t> &active, double &MaxDist, double &MinDist, double &MeanDist, double &MsrDist) {

	// check if we have at least 5 active points
	if (active.size() < 5)
		return;

	// iterate until convergence
	for (int32_t iter = 0; iter < m_maxIter; iter++)
		if (fitStepAffine(T, T_num, R, t, active) < m_minDelta)
			break;
	//EvaluateMatchResultBDD(T, T_num, R, t, active, MaxDist, MinDist, MeanDist, MsrDist);
}

// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
void CIcp::fitIterateAffineBDD(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, std::vector<int32_t> &active, double &MaxDist, double &MinDist, double &MeanDist, double &MsrDist) {

	// check if we have at least 5 active points
	if (active.size() < 5)
		return;

	// iterate until convergence
	for (int32_t iter = 0; iter < m_maxIter; iter++)
		if (fitStepAffineBDD(T, T_num, R, t, active) < m_minDelta)
			break;
	//EvaluateMatchResultBDD(T, T_num, R, t, active, MaxDist, MinDist, MeanDist, MsrDist);
}

// 旧工程中文注释在迁入时已清理，算法逻辑保持不变。
void CIcp::fitIterateBDD(double *T, const int32_t T_num, RMatrix::Matrix &R, RMatrix::Matrix &t, const std::vector<int32_t> &active) {

	// check if we have at least 5 active points
	if (active.size() < 5)
		return;

	// iterate until convergence
	for (int32_t iter = 0; iter < m_maxIter; iter++)
		if (fitStepBDD(T, T_num, R, t, active) < m_minDelta)
			break;
}


