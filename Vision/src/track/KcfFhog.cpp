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
 * @file        KcfFhog.cpp
 * @brief       KCF FHOG 特征实现
 * @details     保留旧工程 LatentSVM FHOG 的梯度直方图、block 归一化、
 *              截断和 PCA 降维流程，用于 CKcfTracker 的 HOG 特征分支。
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
#include <cmath>
#include <cstdlib>
#include <vector>

#include "track/KcfFhog.h"

/**
 * @brief 按旧 LatentSVM FHOG 流程提取 cell 梯度方向直方图。
 *
 * 函数会先在 pixel 网格上计算梯度，再聚合到 cell 网格，最后由调用侧继续执行
 * block 归一化和 PCA 压缩。输出对象由调用方释放。
 */
int getFeatureMaps(const IplImage *image, const int k, CvLSVMFeatureMapCaskade **map)
{
    int sizeX, sizeY;
    int p, px, stringSize;
    int height, width, numChannels;
    int i, j, kk, c, ii, jj, d;
    float *datadx, *datady;

    int ch;
    float magnitude, x, y, tx, ty;

    IplImage *dx;
    IplImage *dy;
    float a_x, b_x;

    float kernel[3] = {-1.F, 0.F, 1.F};
    CvMat kernel_dx = cvMat(1, 3, CV_32F, kernel);
    CvMat kernel_dy = cvMat(3, 1, CV_32F, kernel);

    std::vector<float> r;
    std::vector<int> alfa;

    float boundary_x[NUM_SECTOR + 1];
    float boundary_y[NUM_SECTOR + 1];
    float maxValue, dotProd;
    int maxi;

    if (image == nullptr || map == nullptr || k <= 0)
    {
        return LATENT_SVM_MEM_NULL;
    }

    height = image->height;
    width = image->width;
    numChannels = image->nChannels;

    dx = cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_32F, numChannels);
    dy = cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_32F, numChannels);

    sizeX = width / k;
    sizeY = height / k;
    px = 3 * NUM_SECTOR;
    p = px;
    stringSize = sizeX * p;
    if (allocFeatureMapObject(map, sizeX, sizeY, p) != LATENT_SVM_OK || *map == nullptr)
    {
        cvReleaseImage(&dx);
        cvReleaseImage(&dy);
        return LATENT_SVM_MEM_NULL;
    }

    cvFilter2D(image, dx, &kernel_dx, cvPoint(-1, 0));
    cvFilter2D(image, dy, &kernel_dy, cvPoint(0, -1));

    for (i = 0; i <= NUM_SECTOR; i++)
    {
        const float argVector = static_cast<float>(i) * (static_cast<float>(PI) / NUM_SECTOR);
        boundary_x[i] = cosf(argVector);
        boundary_y[i] = sinf(argVector);
    }

    r.assign(static_cast<size_t>(width) * static_cast<size_t>(height), 0.0F);
    alfa.assign(static_cast<size_t>(width) * static_cast<size_t>(height) * 2U, 0);

    for (j = 1; j < height - 1; j++)
    {
        datadx = reinterpret_cast<float *>(dx->imageData + dx->widthStep * j);
        datady = reinterpret_cast<float *>(dy->imageData + dy->widthStep * j);
        for (i = 1; i < width - 1; i++)
        {
            c = 0;
            x = datadx[i * numChannels + c];
            y = datady[i * numChannels + c];

            r[static_cast<size_t>(j) * width + i] = sqrtf(x * x + y * y);
            for (ch = 1; ch < numChannels; ch++)
            {
                tx = datadx[i * numChannels + ch];
                ty = datady[i * numChannels + ch];
                magnitude = sqrtf(tx * tx + ty * ty);
                if (magnitude > r[static_cast<size_t>(j) * width + i])
                {
                    r[static_cast<size_t>(j) * width + i] = magnitude;
                    c = ch;
                    x = tx;
                    y = ty;
                }
            }

            maxValue = boundary_x[0] * x + boundary_y[0] * y;
            maxi = 0;
            for (kk = 0; kk < NUM_SECTOR; kk++)
            {
                dotProd = boundary_x[kk] * x + boundary_y[kk] * y;
                if (dotProd > maxValue)
                {
                    maxValue = dotProd;
                    maxi = kk;
                }
                else if (-dotProd > maxValue)
                {
                    maxValue = -dotProd;
                    maxi = kk + NUM_SECTOR;
                }
            }

            alfa[static_cast<size_t>(j) * width * 2U + static_cast<size_t>(i) * 2U]
                = maxi % NUM_SECTOR;
            alfa[static_cast<size_t>(j) * width * 2U + static_cast<size_t>(i) * 2U + 1U]
                = maxi;
        }
    }

    std::vector<int> nearest(static_cast<size_t>(k), 0);
    std::vector<float> w(static_cast<size_t>(k) * 2U, 0.0F);

    for (i = 0; i < k / 2; i++)
    {
        nearest[i] = -1;
    }
    for (i = k / 2; i < k; i++)
    {
        nearest[i] = 1;
    }

    for (j = 0; j < k / 2; j++)
    {
        b_x = static_cast<float>(k / 2 + j) + 0.5F;
        a_x = static_cast<float>(k / 2 - j) - 0.5F;
        w[j * 2] = 1.0F / a_x * ((a_x * b_x) / (a_x + b_x));
        w[j * 2 + 1] = 1.0F / b_x * ((a_x * b_x) / (a_x + b_x));
    }
    for (j = k / 2; j < k; j++)
    {
        a_x = static_cast<float>(j - k / 2) + 0.5F;
        b_x = static_cast<float>(-j + k / 2) - 0.5F + static_cast<float>(k);
        w[j * 2] = 1.0F / a_x * ((a_x * b_x) / (a_x + b_x));
        w[j * 2 + 1] = 1.0F / b_x * ((a_x * b_x) / (a_x + b_x));
    }

    for (i = 0; i < sizeY; i++)
    {
        for (j = 0; j < sizeX; j++)
        {
            for (ii = 0; ii < k; ii++)
            {
                for (jj = 0; jj < k; jj++)
                {
                    if ((i * k + ii > 0)
                        && (i * k + ii < height - 1)
                        && (j * k + jj > 0)
                        && (j * k + jj < width - 1))
                    {
                        d = (k * i + ii) * width + (j * k + jj);
                        const size_t dIndex = static_cast<size_t>(d);
                        (*map)->map[i * stringSize + j * (*map)->numFeatures + alfa[dIndex * 2U]]
                            += r[dIndex] * w[static_cast<size_t>(ii) * 2U] * w[static_cast<size_t>(jj) * 2U];
                        (*map)->map[i * stringSize + j * (*map)->numFeatures
                            + alfa[dIndex * 2U + 1U] + NUM_SECTOR]
                            += r[dIndex] * w[static_cast<size_t>(ii) * 2U] * w[static_cast<size_t>(jj) * 2U];

                        if ((i + nearest[ii] >= 0) && (i + nearest[ii] <= sizeY - 1))
                        {
                            (*map)->map[(i + nearest[ii]) * stringSize
                                + j * (*map)->numFeatures + alfa[dIndex * 2U]]
                                += r[dIndex] * w[static_cast<size_t>(ii) * 2U + 1U] * w[static_cast<size_t>(jj) * 2U];
                            (*map)->map[(i + nearest[ii]) * stringSize
                                + j * (*map)->numFeatures + alfa[dIndex * 2U + 1U] + NUM_SECTOR]
                                += r[dIndex] * w[static_cast<size_t>(ii) * 2U + 1U] * w[static_cast<size_t>(jj) * 2U];
                        }
                        if ((j + nearest[jj] >= 0) && (j + nearest[jj] <= sizeX - 1))
                        {
                            (*map)->map[i * stringSize
                                + (j + nearest[jj]) * (*map)->numFeatures + alfa[dIndex * 2U]]
                                += r[dIndex] * w[static_cast<size_t>(ii) * 2U] * w[static_cast<size_t>(jj) * 2U + 1U];
                            (*map)->map[i * stringSize
                                + (j + nearest[jj]) * (*map)->numFeatures
                                + alfa[dIndex * 2U + 1U] + NUM_SECTOR]
                                += r[dIndex] * w[static_cast<size_t>(ii) * 2U] * w[static_cast<size_t>(jj) * 2U + 1U];
                        }
                        if ((i + nearest[ii] >= 0)
                            && (i + nearest[ii] <= sizeY - 1)
                            && (j + nearest[jj] >= 0)
                            && (j + nearest[jj] <= sizeX - 1))
                        {
                            (*map)->map[(i + nearest[ii]) * stringSize
                                + (j + nearest[jj]) * (*map)->numFeatures + alfa[dIndex * 2U]]
                                += r[dIndex] * w[static_cast<size_t>(ii) * 2U + 1U] * w[static_cast<size_t>(jj) * 2U + 1U];
                            (*map)->map[(i + nearest[ii]) * stringSize
                                + (j + nearest[jj]) * (*map)->numFeatures
                                + alfa[dIndex * 2U + 1U] + NUM_SECTOR]
                                += r[dIndex] * w[static_cast<size_t>(ii) * 2U + 1U] * w[static_cast<size_t>(jj) * 2U + 1U];
                        }
                    }
                }
            }
        }
    }

    cvReleaseImage(&dx);
    cvReleaseImage(&dy);

    return LATENT_SVM_OK;
}

/**
 * @brief 对 FHOG block 特征执行归一化与截断。
 *
 * `alfa` 控制每个归一化分量的最大值，避免单个梯度方向支配 KCF 模板。
 */
int normalizeAndTruncate(CvLSVMFeatureMapCaskade *map, const float alfa)
{
    int i, j, ii;
    int sizeX, sizeY, p, pos, pp, xp, pos1, pos2;
    float *partOfNorm;
    float *newData;
    float valOfNorm;

    if (map == nullptr || map->map == nullptr)
    {
        return LATENT_SVM_MEM_NULL;
    }

    sizeX = map->sizeX;
    sizeY = map->sizeY;
    partOfNorm = static_cast<float *>(malloc(sizeof(float) * sizeX * sizeY));
    if (partOfNorm == nullptr)
    {
        return LATENT_SVM_MEM_NULL;
    }

    p = NUM_SECTOR;
    xp = NUM_SECTOR * 3;
    pp = NUM_SECTOR * 12;

    for (i = 0; i < sizeX * sizeY; i++)
    {
        valOfNorm = 0.0F;
        pos = i * map->numFeatures;
        for (j = 0; j < p; j++)
        {
            valOfNorm += map->map[pos + j] * map->map[pos + j];
        }
        partOfNorm[i] = valOfNorm;
    }

    sizeX -= 2;
    sizeY -= 2;
    newData = static_cast<float *>(malloc(sizeof(float) * sizeX * sizeY * pp));
    if (newData == nullptr)
    {
        free(partOfNorm);
        return LATENT_SVM_MEM_NULL;
    }

    for (i = 1; i <= sizeY; i++)
    {
        for (j = 1; j <= sizeX; j++)
        {
            valOfNorm = sqrtf(
                partOfNorm[i * (sizeX + 2) + j]
                + partOfNorm[i * (sizeX + 2) + (j + 1)]
                + partOfNorm[(i + 1) * (sizeX + 2) + j]
                + partOfNorm[(i + 1) * (sizeX + 2) + (j + 1)]) + FLT_EPSILON;
            pos1 = i * (sizeX + 2) * xp + j * xp;
            pos2 = (i - 1) * sizeX * pp + (j - 1) * pp;
            for (ii = 0; ii < p; ii++)
            {
                newData[pos2 + ii] = map->map[pos1 + ii] / valOfNorm;
            }
            for (ii = 0; ii < 2 * p; ii++)
            {
                newData[pos2 + ii + p * 4] = map->map[pos1 + ii + p] / valOfNorm;
            }

            valOfNorm = sqrtf(
                partOfNorm[i * (sizeX + 2) + j]
                + partOfNorm[i * (sizeX + 2) + (j + 1)]
                + partOfNorm[(i - 1) * (sizeX + 2) + j]
                + partOfNorm[(i - 1) * (sizeX + 2) + (j + 1)]) + FLT_EPSILON;
            for (ii = 0; ii < p; ii++)
            {
                newData[pos2 + ii + p] = map->map[pos1 + ii] / valOfNorm;
            }
            for (ii = 0; ii < 2 * p; ii++)
            {
                newData[pos2 + ii + p * 6] = map->map[pos1 + ii + p] / valOfNorm;
            }

            valOfNorm = sqrtf(
                partOfNorm[i * (sizeX + 2) + j]
                + partOfNorm[i * (sizeX + 2) + (j - 1)]
                + partOfNorm[(i + 1) * (sizeX + 2) + j]
                + partOfNorm[(i + 1) * (sizeX + 2) + (j - 1)]) + FLT_EPSILON;
            for (ii = 0; ii < p; ii++)
            {
                newData[pos2 + ii + p * 2] = map->map[pos1 + ii] / valOfNorm;
            }
            for (ii = 0; ii < 2 * p; ii++)
            {
                newData[pos2 + ii + p * 8] = map->map[pos1 + ii + p] / valOfNorm;
            }

            valOfNorm = sqrtf(
                partOfNorm[i * (sizeX + 2) + j]
                + partOfNorm[i * (sizeX + 2) + (j - 1)]
                + partOfNorm[(i - 1) * (sizeX + 2) + j]
                + partOfNorm[(i - 1) * (sizeX + 2) + (j - 1)]) + FLT_EPSILON;
            for (ii = 0; ii < p; ii++)
            {
                newData[pos2 + ii + p * 3] = map->map[pos1 + ii] / valOfNorm;
            }
            for (ii = 0; ii < 2 * p; ii++)
            {
                newData[pos2 + ii + p * 10] = map->map[pos1 + ii + p] / valOfNorm;
            }
        }
    }

    for (i = 0; i < sizeX * sizeY * pp; i++)
    {
        if (newData[i] > alfa)
        {
            newData[i] = alfa;
        }
    }

    map->numFeatures = pp;
    map->sizeX = sizeX;
    map->sizeY = sizeY;
    free(map->map);
    free(partOfNorm);
    map->map = newData;

    return LATENT_SVM_OK;
}

/**
 * @brief 将完整 FHOG 特征压缩为 KCF 使用的 PCA 后通道。
 *
 * 此实现保留 OpenCV LatentSVM 的通道布局，输出会覆盖输入 map。
 */
int PCAFeatureMaps(CvLSVMFeatureMapCaskade *map)
{
    int i, j, ii, jj, k;
    int sizeX, sizeY, p, pp, xp, yp, pos1, pos2;
    float *newData;
    float val;
    float nx, ny;

    if (map == nullptr || map->map == nullptr)
    {
        return LATENT_SVM_MEM_NULL;
    }

    sizeX = map->sizeX;
    sizeY = map->sizeY;
    p = map->numFeatures;
    pp = NUM_SECTOR * 3 + 4;
    yp = 4;
    xp = NUM_SECTOR;

    nx = 1.0F / sqrtf(static_cast<float>(xp * 2));
    ny = 1.0F / sqrtf(static_cast<float>(yp));

    newData = static_cast<float *>(malloc(sizeof(float) * sizeX * sizeY * pp));
    if (newData == nullptr)
    {
        return LATENT_SVM_MEM_NULL;
    }

    for (i = 0; i < sizeY; i++)
    {
        for (j = 0; j < sizeX; j++)
        {
            pos1 = (i * sizeX + j) * p;
            pos2 = (i * sizeX + j) * pp;
            k = 0;
            for (jj = 0; jj < xp * 2; jj++)
            {
                val = 0.0F;
                for (ii = 0; ii < yp; ii++)
                {
                    val += map->map[pos1 + yp * xp + ii * xp * 2 + jj];
                }
                newData[pos2 + k] = val * ny;
                k++;
            }
            for (jj = 0; jj < xp; jj++)
            {
                val = 0.0F;
                for (ii = 0; ii < yp; ii++)
                {
                    val += map->map[pos1 + ii * xp + jj];
                }
                newData[pos2 + k] = val * ny;
                k++;
            }
            for (ii = 0; ii < yp; ii++)
            {
                val = 0.0F;
                for (jj = 0; jj < 2 * xp; jj++)
                {
                    val += map->map[pos1 + yp * xp + ii * xp * 2 + jj];
                }
                newData[pos2 + k] = val * nx;
                k++;
            }
        }
    }

    map->numFeatures = pp;
    free(map->map);
    map->map = newData;

    return LATENT_SVM_OK;
}

/**
 * @brief 分配连续 FHOG 特征缓冲区。
 *
 * 成功后 `obj` 拥有 `map` 指向的连续 float 内存，释放必须走
 * freeFeatureMapObject 以匹配旧接口。
 */
int allocFeatureMapObject(
    CvLSVMFeatureMapCaskade **obj,
    const int sizeX,
    const int sizeY,
    const int numFeatures)
{
    if (obj == nullptr)
    {
        return LATENT_SVM_MEM_NULL;
    }

    *obj = static_cast<CvLSVMFeatureMapCaskade *>(malloc(sizeof(CvLSVMFeatureMapCaskade)));
    if (*obj == nullptr)
    {
        return LATENT_SVM_MEM_NULL;
    }
    (*obj)->sizeX = sizeX;
    (*obj)->sizeY = sizeY;
    (*obj)->numFeatures = numFeatures;
    (*obj)->map = static_cast<float *>(malloc(sizeof(float) * sizeX * sizeY * numFeatures));
    if ((*obj)->map == nullptr)
    {
        free(*obj);
        *obj = nullptr;
        return LATENT_SVM_MEM_NULL;
    }
    for (int i = 0; i < sizeX * sizeY * numFeatures; i++)
    {
        (*obj)->map[i] = 0.0F;
    }
    return LATENT_SVM_OK;
}

/**
 * @brief 释放 FHOG 特征图及其内部缓冲区。
 *
 * 函数接受二级指针是为了兼容旧 LatentSVM C API 的所有权传递方式。
 */
int freeFeatureMapObject(CvLSVMFeatureMapCaskade **obj)
{
    if (obj == nullptr || *obj == nullptr)
    {
        return LATENT_SVM_MEM_NULL;
    }
    free((*obj)->map);
    free(*obj);
    *obj = nullptr;
    return LATENT_SVM_OK;
}
