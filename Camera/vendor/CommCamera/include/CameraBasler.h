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
 * @file        CameraCRTPBasler.h
 * @brief       Basler 相机实现类
 * @details     基于 Pylon SDK 实现，支持单张采集和连续采集。
 *
 * @version     1.2
 * @author      RyuYM
 * @date        2026 年 6 月 3 日
 *
 * @par         修改日志
 * <table>
 * <tr><th>Version  <th>Author      <th>Date        <th>Description         </tr>
 * <tr><td>1.0      <td>RyuYM       <td>260522      <td>创建文件            </tr>
 * <tr><td>1.1      <td>RyuYM       <td>260525      <td>初步完成            </tr>
 * <tr><td>1.1.1    <td>RyuYM       <td>260601      <td>修正相机连接失败的问题</tr>
 * <tr><td>1.2      <td>RyuYM       <td>260603      <td>使用 CRTP           </tr>
 * </table>
 *
 ******************************************************************************/

#pragma once
#ifndef I2R_COMM_CAMERA_BASLER_H
#define I2R_COMM_CAMERA_BASLER_H

#include <string>

#include <pylon/ImageFormatConverter.h>
#include <pylon/InstantCamera.h>
#include <pylon/PylonImage.h>

#include "CameraCRTP.h"

namespace i2r_comm_camera
{
    /**
     * @class       CameraBasler
     * @brief       Basler 相机实现类
     * @details     基于 Pylon SDK 实现，支持单张采集和连续采集。
     */
    class CameraBasler final : public CameraCRTP<CameraBasler>
    {
        friend class CameraCRTP<CameraBasler>;

    public:
        /**
         * @enum        PixelFormat
         * @brief       Basler 相机像素格式
         * @details     枚举名需要与 Pylon SDK 保持一致，实际是通过 magic_enum 将枚举量反射成枚举名字符串进行设置。
         */
        enum class PixelFormat
        {
            Mono8,
            BayerGR8, BayerRG8, BayerGB8, BayerBG8,
            BayerGR12, BayerGR12p,
            BayerRG12, BayerRG12p,
            BayerGB12, BayerGB12p,
            BayerBG12, BayerBG12p,
            RGB8, BGR8, YCbCr422_8
        };

    private:
        Pylon::CInstantCamera m_camera;     ///< Pylon 相机对象

    public:
        explicit CameraBasler();
        ~CameraBasler() { disconnect(); }

        /**
         * @brief       设置相机像素格式
         * @details
         */
        std::expected<void, Error> setPixelFormat(PixelFormat pixelFormat);

        /**
         * @brief       获取相机像素格式
         * @details
         */
        std::expected<std::string, Error> getPixelFormat();

    private:
        std::expected<void, Error> connectImpl(unsigned int cameraIndex);
        void disconnectImpl();

        std::expected<cv::Mat, Error> grabSingleImpl(int timeout);
        std::expected<void, Error> grabContinuousStartImpl();
        void grabContinuousStopImpl();

        std::expected<void, Error> setExposureTimeImpl(double exposureTimeUs);
        std::expected<double, Error> getExposureTimeImpl();

        std::expected<void, Error> setGainImpl(double gain);
        std::expected<double, Error> getGainImpl();

        std::expected<void, Error> setBlackLevelImpl(double blackLevel);
        std::expected<double, Error> getBlackLevelImpl();

        std::expected<void, Error> setGammaImpl(double gamma);
        std::expected<double, Error> getGammaImpl();
    };
}

#endif // !I2R_COMM_CAMERA_BASLER_H
