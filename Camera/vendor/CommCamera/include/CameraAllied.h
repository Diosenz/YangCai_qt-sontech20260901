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
 * @file        CameraAllied.h
 * @brief       Allied 相机实现类
 * @details     基于 Vimba X SDK 实现，支持单张采集和连续采集。
 *
 * @version     1.2
 * @author      RyuYM
 * @date        2026 年 6 月 3 日
 *
 * @par         修改日志
 * <table>
 * <tr><th>Version  <th>Author      <th>Date        <th>Description         </tr>
 * <tr><td>1.0      <td>RyuYM       <td>260602      <td>创建文件            </tr>
 * <tr><td>1.1      <td>RyuYM       <td>260603      <td>初步完成            </tr>
 * <tr><td>1.2      <td>RyuYM       <td>260603      <td>使用 CRTP           </tr>
 * </table>
 *
 ******************************************************************************/

#pragma once
#ifndef I2R_COMM_CAMERA_CAMERA_ALLIED_H
#define I2R_COMM_CAMERA_CAMERA_ALLIED_H

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>

#include <VmbCPP/IFrameObserver.h>
#include <VmbCPP/SharedPointerDefines.h>

#include "CameraCRTP.h"

namespace i2r_comm_camera
{
    /**
     * @class       CameraAllied
     * @brief       Allied 相机实现类
     * @details     基于 Vimba X SDK 实现，支持单张采集和异步连续采集。
     */
    class CameraAllied final : public CameraCRTP<CameraAllied>
    {
        friend class CameraCRTP<CameraAllied>;

    private:
        /**
         * @class       FrameObserver
         * @brief       Vimba SDK 帧回调观察者
         * @details     Vimba 疑似没有提供同步连续采集功能，因此使用回调观察者，将帧存入私有帧缓冲 m_frame。
         */
        class FrameObserver : public VmbCPP::IFrameObserver
        {
        private:
            CameraAllied& m_parent;

        public:
            explicit FrameObserver(VmbCPP::CameraPtr camera, CameraAllied& parent) :
                IFrameObserver(camera), m_parent(parent)
            {

            }
            virtual ~FrameObserver() = default;
            virtual void FrameReceived(const VmbCPP::FramePtr frame) override;
        };

        /**
         * @enum        PixelFormat
         * @brief       Allied 相机像素格式
         * @details     枚举名需要与 Vimba SDK 保持一致，实际是通过 magic_enum 将枚举量反射成枚举名字符串进行设置。
         */
        enum class PixelFormat { Mono8, Mono10, Mono12 };

    private:
        VmbCPP::CameraPtr m_camera;                         ///< Allied 相机对象
        std::shared_ptr<FrameObserver> m_frameObserver;     ///< 帧回调观察者

        mutable std::mutex m_frameMutex;                    ///< 帧缓冲互斥锁
        std::condition_variable m_frameCondition;           ///< 新帧到达条件变量
        VmbCPP::FramePtr m_frame;                           ///< 帧缓冲
        bool m_frameAvailable = false;                      ///< 新帧可用标志

    public:
        explicit CameraAllied();
        ~CameraAllied() { disconnect(); }                   ///< 注意 CRTP 不需要虚析构

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

#endif // !I2R_COMM_CAMERA_CAMERA_ALLIED_H
