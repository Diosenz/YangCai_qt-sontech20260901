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
 * @file        CameraCRTP.h
 * @brief       基于 CRTP 的相机基类
 * @details     基于 C++23 的推导 this 实现的 CRTP 实现的静态多态的相机基类，统一所有相机厂商的接口规范，所有通用逻辑集中在基类，派生类只需实现硬件相关的 Impl 接口。
 * @todo        查询相机列表
 * @todo        连续采集图像使用回调函数会不会有更好的性能?
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
 * <tr><td>1.2      <td>RyuYM       <td>260603      <td>使用 CRTP           </tr>
 * </table>
 *
 ******************************************************************************/

#pragma once
#ifndef I2R_COMM_CAMERA_CAMERA_CRTP_H
#define I2R_COMM_CAMERA_CAMERA_CRTP_H

#include <atomic>
#include <expected>
#include <mutex>
#include <thread>

#include <opencv4/opencv2/core/mat.hpp>

#include "Config.h"

namespace i2r_comm_camera
{
    enum class TypeCamera{ UNKNOWN, ALLIED, BASLER, DAHENG, HIK };

    enum class Error
    {
        UNKNOWN_ERROR,
        CAMERA_DUPLICATE_CONNECTED,
        CAMERA_NOT_FOUND,
        CAMERA_INIT_FAILED,
        CAMERA_NOT_CONNECTED,
        SHOOT_FAILED,
        SHOOT_TIMEOUT,
        SHOOT_CONTINUOUS_ALREADY_RUNNING,
        SHOOT_CONTINUOUS_NOT_RUNNING,
        IMAGE_BUFFER_EMPTY
    };

    /**
     * @class       CameraCRTP
     * @brief       基于 CRTP 的相机基类
     * @details     基于 C++23 的推导 this 实现的 CRTP 实现的静态多态的相机基类，统一所有相机厂商的接口规范，所有通用逻辑集中在基类，派生类只需实现硬件相关的 Impl 接口。
     * @tparam      Derived: 相机派生类
     */
    template<typename Derived>
    class CameraCRTP
    {
    private:
        const TypeCamera m_typeCamera;                      ///< 相机类型
        std::atomic<bool> m_isConnected = false;            ///< 相机连接状态
        std::atomic<bool> m_isShootingContinuous = false;   ///< 相机连续拍摄状态
        std::jthread m_jthreadContinuous;                   ///< 连续采图线程
        mutable std::mutex m_mutexImage;                    ///< 多线程访问图像互斥锁
        cv::Mat m_image;                                    ///< 当前图像

    protected:
        explicit CameraCRTP(TypeCamera type) : m_typeCamera(type) {}
        ~CameraCRTP() = default;                            ///< 注意 CRTP 不需要虚析构

    public:
        CameraCRTP(const CameraCRTP&) = delete;
        CameraCRTP(CameraCRTP&&) = delete;
        CameraCRTP& operator=(const CameraCRTP&) = delete;
        CameraCRTP& operator=(CameraCRTP&&) = delete;

        /**
         * @brief       获取相机类型
         * @details
         */
        TypeCamera getTypeCamera() const { return m_typeCamera; }

        /**
         * @brief       是否处于连接状态
         * @details
         */
        bool isConnected() const { return m_isConnected; }

        /**
         * @brief       是否正在连续拍摄
         * @details
         */
        bool isShootingContinuous() const { return m_isShootingContinuous; }

        /**
         * @brief       连接相机设备
         * @details
         * @warning     必须由派生类实现 connectImpl
         */
        template<typename Self>
        std::expected<void, Error> connect(this Self&& self, unsigned int cameraIndex = 0)
        {
            if (self.m_isConnected) return std::unexpected(Error::CAMERA_DUPLICATE_CONNECTED);
            auto result = self.connectImpl(cameraIndex);
            if (result) self.m_isConnected = true;
            return result;
        }

        /**
         * @brief       断开相机设备
         * @details
         * @warning     必须由派生类实现 disconnectImpl
         */
        template<typename Self>
        void disconnect(this Self&& self)
        {
            if (self.m_isShootingContinuous) self.shootContinuousStop();
            if (self.m_isConnected)
            {
                self.disconnectImpl();
                self.m_isConnected = false;
            }
        }

        /**
         * @brief       拍摄单张照片
         * @details
         * @warning     必须由派生类实现 grabSingleImpl
         */
        template<typename Self>
        std::expected<cv::Mat, Error> shootSingle(this Self&& self,
            int timeout = config::SHOOT_SINGLE_TIMEOUT)
        {
            if (!self.m_isConnected)
                return std::unexpected(Error::CAMERA_NOT_CONNECTED);
            if (self.m_isShootingContinuous)
                return std::unexpected(Error::SHOOT_CONTINUOUS_ALREADY_RUNNING);

            return self.grabSingleImpl(timeout);
        }

        /**
         * @brief       开始连续拍摄
         * @details
         * @warning     必须由派生类实现 grabContinuousStartImpl、grabSingleImpl 和 grabContinuousStopImpl
         */
        template<typename Self>
        std::expected<void, Error> shootContinuousStart(this Self&& self,
            int timeout = config::SHOOT_CONTINUOUS_TIMEOUT)
        {
            if (!self.m_isConnected)
                return std::unexpected(Error::CAMERA_NOT_CONNECTED);
            if (self.m_isShootingContinuous)
                return std::unexpected(Error::SHOOT_CONTINUOUS_ALREADY_RUNNING);
            if (auto result = self.grabContinuousStartImpl())
            {
                self.m_isShootingContinuous = true;
                self.m_jthreadContinuous = std::jthread([&self, timeout](std::stop_token st)
                {
                    while (!st.stop_requested() && self.m_isShootingContinuous)
                    {
                        try
                        {
                            if (const auto& result = self.grabSingleImpl(timeout))
                            {
                                std::lock_guard lock(self.m_mutexImage);
                                self.m_image = result.value().clone();
                            }
                        }
                        catch (...) {} // 忽略连续采集异常
                    }

                    self.grabContinuousStopImpl();
                    self.m_isShootingContinuous = false;
                });

                return {};
            }
            else return result;
        }

        /**
         * @brief       停止连续拍摄
         * @details
         */
        template<typename Self>
        void shootContinuousStop(this Self&& self)
        {
            if (self.m_isShootingContinuous)
            {
                self.m_jthreadContinuous.request_stop();
                if (self.m_jthreadContinuous.joinable())
                {
                    self.m_jthreadContinuous.join();
                }
            }
        }

        /**
         * @brief       获取连续拍摄最新图像
         * @details
         */
        template<typename Self>
        std::expected<cv::Mat, Error> getImage(this const Self& self)
        {
            if (!self.m_isShootingContinuous)
                return std::unexpected(Error::SHOOT_CONTINUOUS_NOT_RUNNING);

            std::lock_guard lock(self.m_mutexImage);
            if (self.m_image.empty())   return std::unexpected(Error::IMAGE_BUFFER_EMPTY);
            else                        return self.m_image.clone();
        }

        /**
         * @brief       设置曝光时间
         * @details
         * @warning     必须由派生类实现 setExposureTimeImpl
         */
        template<typename Self>
        std::expected<void, Error> setExposureTime(this Self&& self, double exposureTimeUs)
        {
            if (!self.m_isConnected)    return std::unexpected(Error::CAMERA_NOT_CONNECTED);
            else                        return self.setExposureTimeImpl(exposureTimeUs);
        }

        /**
         * @brief       获取曝光时间
         * @details
         * @warning     必须由派生类实现 getExposureTimeImpl
         */
        template<typename Self>
        std::expected<double, Error> getExposureTime(this Self&& self)
        {
            if (!self.m_isConnected)    return std::unexpected(Error::CAMERA_NOT_CONNECTED);
            else                        return self.getExposureTimeImpl();
        }

        /**
         * @brief       设置增益
         * @details
         * @warning     必须由派生类实现 setGainImpl
         */
        template<typename Self>
        std::expected<void, Error> setGain(this Self&& self, double gain)
        {
            if (!self.m_isConnected)    return std::unexpected(Error::CAMERA_NOT_CONNECTED);
            else                        return self.setGainImpl(gain);
        }

        /**
         * @brief       获取增益
         * @details
         * @warning     必须由派生类实现 getGainImpl
         */
        template<typename Self>
        std::expected<double, Error> getGain(this Self&& self)
        {
            if (!self.m_isConnected)    return std::unexpected(Error::CAMERA_NOT_CONNECTED);
            else                        return self.getGainImpl();
        }

        /**
         * @brief       设置黑电平
         * @details
         * @warning     必须由派生类实现 setBlackLevelImpl
         */
        template<typename Self>
        std::expected<void, Error> setBlackLevel(this Self&& self, double blackLevel)
        {
            if (!self.m_isConnected)    return std::unexpected(Error::CAMERA_NOT_CONNECTED);
            else                        return self.setBlackLevelImpl(blackLevel);
        }

        /**
         * @brief       获取黑电平
         * @details
         * @warning     必须由派生类实现 getBlackLevelImpl
         */
        template<typename Self>
        std::expected<double, Error> getBlackLevel(this Self&& self)
        {
            if (!self.m_isConnected)    return std::unexpected(Error::CAMERA_NOT_CONNECTED);
            else                        return self.getBlackLevelImpl();
        }

        /**
         * @brief       设置伽马值
         * @details
         * @warning     必须由派生类实现 setGammaImpl
         */
        template<typename Self>
        std::expected<void, Error> setGamma(this Self&& self, double gamma)
        {
            if (!self.m_isConnected)    return std::unexpected(Error::CAMERA_NOT_CONNECTED);
            else                        return self.setGammaImpl(gamma);
        }

        /**
         * @brief       获取伽马值
         * @details
         * @warning     必须由派生类实现 getGammaImpl
         */
        template<typename Self>
        std::expected<double, Error> getGamma(this Self&& self)
        {
            if (!self.m_isConnected)    return std::unexpected(Error::CAMERA_NOT_CONNECTED);
            else                        return self.getGammaImpl();
        }
    };
}

#endif // !I2R_COMM_CAMERA_CAMERA_CRTP_H
