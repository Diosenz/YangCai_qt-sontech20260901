/*******************************************************************************
 *       ______   ______   _______         __                  __
 *      /      | /      \ /       \       /  |                /  |
 *      $$$$$$/ /$$$$$$  |$$$$$$$  |      $$ |        ______  $$ |____
 *        $$ |  $$____$$ |$$ |__$$ |      $$ |       /      \ $$      \
 *        $$ |   /    $$/ $$    $$<       $$ |       $$$$$$  |$$$$$$$  |
 *        $$ |  /$$$$$$/  $$$$$$$  |      $$ |       /    $$ |$$ |  $$ |
 *       _$$ |_ $$ |_____ $$ |  $$ |      $$ |_____ /$$$$$$$ |$$ |__$$ |
 *      / $$   |$$       |$$ |  $$ |      $$       |$$    $$ |$$    $$/
 *      $$$$$$/ $$$$$$$$/ $$/   $$/       $$$$$$$$/  $$$$$$$/ $$$$$$$/
 *
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        KcfFhog.h
 * @brief       KCF FHOG 特征接口
 * @details     LatentSVM FHOG 数据结构和函数接口，供 CKcfTracker
 *              按 KCF 特征流程提取梯度方向特征。
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
/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2010-2013, University of Nizhny Novgorod, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/
#ifndef I2R_RLC_VISION_TRACK_KCF_FHOG_H
#define I2R_RLC_VISION_TRACK_KCF_FHOG_H

#include <cfloat>
#include <opencv2/imgproc/imgproc_c.h>

/**
 * @brief LatentSVM FHOG 特征图。
 *
 * `sizeX/sizeY` 为 cell 网格尺寸，
 * `numFeatures` 为每个 cell 的特征维度，`map` 使用连续 `float` 内存。
 */
typedef struct {
    /// @brief cell 网格宽度，单位为 cell。
    int sizeX;
    /// @brief cell 网格高度，单位为 cell。
    int sizeY;
    /// @brief 每个 cell 的特征维度。
    int numFeatures;
    /// @brief 连续特征缓冲区，长度为 `sizeX * sizeY * numFeatures`。
    float *map;
} CvLSVMFeatureMapCaskade;

/// @brief 梯度方向计算使用的圆周率常量。
#define PI CV_PI
/// @brief FHOG 归一化和除法保护使用的小量。
#define EPS 0.000001
/// @brief 单精度浮点最大值，沿用 LatentSVM FHOG 旧实现。
#define F_MAX FLT_MAX
/// @brief 单精度浮点最小值，沿用 LatentSVM FHOG 旧实现。
#define F_MIN -FLT_MAX
/// @brief 每个半圆方向直方图的扇区数量。
#define NUM_SECTOR 9
/// @brief FHOG block 归一化中的正则化系数。
#define LAMBDA 10
/// @brief 默认 cell 边长，单位 pixel。
#define SIDE_LENGTH 8
/// @brief FHOG 归一化后的截断阈值。
#define VAL_OF_TRUNCATE 0.2f

/// @brief LatentSVM/FHOG 操作成功。
#define LATENT_SVM_OK 0
/// @brief 输出特征图指针为空。
#define LATENT_SVM_MEM_NULL 2
/// @brief 距离变换操作成功。
#define DISTANCE_TRANSFORM_OK 1
/// @brief 距离变换求交失败。
#define DISTANCE_TRANSFORM_GET_INTERSECTION_ERROR -1
/// @brief 距离变换通用错误。
#define DISTANCE_TRANSFORM_ERROR -2
/// @brief 距离变换输入点重合。
#define DISTANCE_TRANSFORM_EQUAL_POINTS -3
/// @brief 特征金字塔生成失败。
#define LATENT_SVM_GET_FEATURE_PYRAMID_FAILED -4
/// @brief 目标搜索失败。
#define LATENT_SVM_SEARCH_OBJECT_FAILED -5
/// @brief 候选区域叠加失败。
#define LATENT_SVM_FAILED_SUPERPOSITION -6
/// @brief 滤波窗口越界。
#define FILTER_OUT_OF_BOUNDARIES -7
/// @brief TBB 调度创建失败，保留旧状态码兼容。
#define LATENT_SVM_TBB_SCHEDULE_CREATION_FAILED -8
/// @brief TBB 线程数非法，保留旧状态码兼容。
#define LATENT_SVM_TBB_NUMTHREADS_NOT_CORRECT -9
/// @brief FFT 操作成功。
#define FFT_OK 2
/// @brief FFT 操作失败。
#define FFT_ERROR -10
/// @brief 模型解析文件不存在。
#define LSVM_PARSER_FILE_NOT_FOUND -11

/**
 * @brief 计算 KCF 使用的 FHOG 初始特征图。
 * @param image 输入 IplImage，支持灰度或多通道图像。
 * @param k cell 尺寸，单位 pixel。
 * @param map 输出特征图，由调用方通过 freeFeatureMapObject 释放。
 * @return LATENT_SVM_OK 表示成功。
 */
int getFeatureMaps(const IplImage *image, int k, CvLSVMFeatureMapCaskade **map);

/**
 * @brief 按 4-cell block 归一化并截断 FHOG 特征。
 * @param map 待处理特征图。
 * @param alfa 截断阈值。
 * @return LATENT_SVM_OK 表示成功。
 */
int normalizeAndTruncate(CvLSVMFeatureMapCaskade *map, float alfa);

/**
 * @brief 执行 LatentSVM FHOG 的 PCA 降维步骤。
 * @param map 待处理特征图。
 * @return LATENT_SVM_OK 表示成功。
 */
int PCAFeatureMaps(CvLSVMFeatureMapCaskade *map);

/**
 * @brief 分配 FHOG 特征图对象。
 * @param obj 输出对象指针；成功后由调用方通过 freeFeatureMapObject 释放。
 * @param sizeX cell 网格宽度，单位为 cell。
 * @param sizeY cell 网格高度，单位为 cell。
 * @param numFeatures 每个 cell 的特征维度。
 * @return LATENT_SVM_OK 表示成功，LATENT_SVM_MEM_NULL 表示输入输出指针无效。
 */
int allocFeatureMapObject(
    CvLSVMFeatureMapCaskade **obj,
    int sizeX,
    int sizeY,
    int numFeatures);

/**
 * @brief 释放 FHOG 特征图对象。
 * @param obj 待释放对象的二级指针；函数会释放内部 `map` 与对象本体。
 * @return LATENT_SVM_OK 表示成功，LATENT_SVM_MEM_NULL 表示输入指针无效。
 */
int freeFeatureMapObject(CvLSVMFeatureMapCaskade **obj);

#endif // I2R_RLC_VISION_TRACK_KCF_FHOG_H
