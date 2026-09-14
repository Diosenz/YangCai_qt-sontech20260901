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
 * @file        CameraAllied.cpp
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

#include <chrono>

#include <VmbCPP/VmbCPP.h>
#include <VmbCPP/thirdparty/OpenCV.hpp>
#include <magic_enum/magic_enum.hpp>

#include "CameraAllied.h"

using namespace VmbCPP;
using namespace i2r_comm_camera;

namespace
{
    class AlliedSDKInitializer
    {
    private:
        AlliedSDKInitializer() { VmbSystem::GetInstance().Startup(); }
        ~AlliedSDKInitializer() { VmbSystem::GetInstance().Shutdown(); }

    public:
        static AlliedSDKInitializer& instance()
        {
            static AlliedSDKInitializer s_instance;
            return s_instance;
        }
    };
}

void CameraAllied::FrameObserver::FrameReceived(const FramePtr frame)
{
    if (VmbFrameStatusType frameStatus;
        frame->GetReceiveStatus(frameStatus) == VmbErrorSuccess &&
        frameStatus == VmbFrameStatusComplete)
    {
        std::lock_guard lock(m_parent.m_frameMutex);
        m_parent.m_frame = frame;
        m_parent.m_frameAvailable = true;
        m_parent.m_frameCondition.notify_one();
    }

    try
    {
        if (m_pCamera)
        {
            m_pCamera->QueueFrame(frame);
        }
    }
    catch (...) {}
}

CameraAllied::CameraAllied() : CameraCRTP(TypeCamera::ALLIED)
{
    AlliedSDKInitializer::instance();
}

/**
 * @brief       设置相机像素格式
 * @details
 */
std::expected<void, Error> CameraAllied::setPixelFormat(PixelFormat pixelFormat)
{
    try
    {
        FeaturePtr feature;
        if (m_camera->GetFeatureByName("PixelFormat", feature) == VmbErrorSuccess &&
            feature->SetValue(magic_enum::enum_name(pixelFormat).data()) == VmbErrorSuccess)
        {
            return {};
        }
        return std::unexpected(Error::UNKNOWN_ERROR);
    }
    catch (...) { return std::unexpected(Error::UNKNOWN_ERROR); }
}

/**
 * @brief       获取相机像素格式
 * @details
 */
std::expected<std::string, Error> CameraAllied::getPixelFormat()
{
    try
    {
        FeaturePtr feature;
        std::string value;
        if (m_camera->GetFeatureByName("PixelFormat", feature) == VmbErrorSuccess &&
            feature->GetValue(value) == VmbErrorSuccess)
        {
            return value;
        }
        return std::unexpected(Error::UNKNOWN_ERROR);
    }
    catch (...) { return std::unexpected(Error::UNKNOWN_ERROR); }
}

std::expected<void, Error> CameraAllied::connectImpl(unsigned int cameraIndex)
{
    try
    {
        // 枚举所有相机
        CameraPtrVector cameraList;
        if (VmbSystem::GetInstance().GetCameras(cameraList) != VmbErrorSuccess ||
            cameraIndex >= cameraList.size())
        {
            return std::unexpected(Error::CAMERA_NOT_FOUND);
        }

        // 完全访问权限打开相机
        m_camera = cameraList[cameraIndex];
        if (m_camera->Open(VmbAccessModeFull) != VmbErrorSuccess)
        {
            m_camera.reset();
            return std::unexpected(Error::CAMERA_INIT_FAILED);
        }

        // GigE 相机设置包大小
        if (StreamPtrVector streams;
            m_camera->GetStreams(streams) == VmbErrorSuccess && !streams.empty())
        {
            if (FeaturePtr feature;
                streams[0]->GetFeatureByName("GVSPAdjustPacketSize", feature) == VmbErrorSuccess)
            {
                if (feature->RunCommand() == VmbErrorSuccess)
                {
                    bool commandDone = false;
                    do
                    {
                        if (feature->IsCommandDone(commandDone) != VmbErrorSuccess) break;
                    } while (commandDone == false);
                }
            }
        }

        return {};
    }
    catch (...) { return std::unexpected(Error::CAMERA_INIT_FAILED); }
}

void CameraAllied::disconnectImpl()
{
    try
    {
        if (m_camera)
        {
            m_camera->Close();
            m_camera.reset();
        }
    }
    catch (...) {}  // 忽略断开时异常
}

std::expected<cv::Mat, Error> CameraAllied::grabSingleImpl(int timeout)
{
    try
    {
        FramePtr frame;

        if (isShootingContinuous())
        {
            std::unique_lock lock(m_frameMutex);
            if (!m_frameCondition.wait_for(
                lock, std::chrono::milliseconds(timeout), [this] {return m_frameAvailable; }))
            {
                return std::unexpected(Error::SHOOT_TIMEOUT);
            }

            frame = m_frame;
            m_frameAvailable = false;
        }
        else
        {
            if (const auto& error = m_camera->AcquireSingleImage(frame, timeout);
                error != VmbErrorSuccess)
            {
                if (error == VmbErrorTimeout) return std::unexpected(Error::SHOOT_TIMEOUT);
                else return std::unexpected(Error::SHOOT_FAILED);
            }
        }

        if (VmbFrameStatusType frameStatus;
            frame->GetReceiveStatus(frameStatus) != VmbErrorSuccess ||
            frameStatus != VmbFrameStatusComplete)
        {
            return std::unexpected(Error::SHOOT_FAILED);
        }

        cv::Mat image;
        if (VmbFrameToMat(frame, image) == VmbErrorSuccess) return image.clone();
        else return std::unexpected(Error::SHOOT_FAILED);
    }
    catch (...) { return std::unexpected(Error::UNKNOWN_ERROR); }
}

std::expected<void, Error> CameraAllied::grabContinuousStartImpl()
{
    try
    {
        {
            std::lock_guard lock(m_frameMutex);
            m_frame.reset();
            m_frameAvailable = false;
        }

        m_frameObserver = std::make_shared<FrameObserver>(m_camera, *this);
        if (m_camera->StartContinuousImageAcquisition(5, m_frameObserver) != VmbErrorSuccess)
        {
            m_frameObserver.reset();
            return std::unexpected(Error::SHOOT_FAILED);
        }

        return {};
    }
    catch (...) { return std::unexpected(Error::UNKNOWN_ERROR); }
}

void CameraAllied::grabContinuousStopImpl()
{
    try
    {
        if (m_camera)
        {
            m_camera->StopContinuousImageAcquisition();
        }
        m_frameObserver.reset();

        {
            std::lock_guard lock(m_frameMutex);
            m_frame.reset();
            m_frameAvailable = false;
        }
        m_frameCondition.notify_all();
    }
    catch (...) {}
}

std::expected<void, Error> CameraAllied::setExposureTimeImpl(double exposureTimeUs)
{
    try
    {
        FeaturePtr feature;
        if (m_camera->GetFeatureByName("ExposureAuto", feature) == VmbErrorSuccess &&
            feature->SetValue("Off") == VmbErrorSuccess &&
            m_camera->GetFeatureByName("ExposureTimeAbs", feature) == VmbErrorSuccess &&
            feature->SetValue(exposureTimeUs) == VmbErrorSuccess)
        {
            return {};
        }
        return std::unexpected(Error::UNKNOWN_ERROR);
    }
    catch (...) { return std::unexpected(Error::UNKNOWN_ERROR); }
}

std::expected<double, Error> CameraAllied::getExposureTimeImpl()
{
    try
    {
        FeaturePtr feature;
        double value;
        if (m_camera->GetFeatureByName("ExposureTimeAbs", feature) == VmbErrorSuccess &&
            feature->GetValue(value) == VmbErrorSuccess)
        {
            return value;
        }
        return std::unexpected(Error::UNKNOWN_ERROR);
    }
    catch (...) { return std::unexpected(Error::UNKNOWN_ERROR); }
}

std::expected<void, Error> CameraAllied::setGainImpl(double gain)
{
    try
    {
        FeaturePtr feature;
        if (m_camera->GetFeatureByName("GainAuto", feature) == VmbErrorSuccess &&
            feature->SetValue("Off") == VmbErrorSuccess &&
            m_camera->GetFeatureByName("Gain", feature) == VmbErrorSuccess &&
            feature->SetValue(gain) == VmbErrorSuccess)
        {
            return {};
        }
        return std::unexpected(Error::UNKNOWN_ERROR);
    }
    catch (...) { return std::unexpected(Error::UNKNOWN_ERROR); }
}

std::expected<double, Error> CameraAllied::getGainImpl()
{
    try
    {
        FeaturePtr feature;
        double value;
        if (m_camera->GetFeatureByName("Gain", feature) == VmbErrorSuccess &&
            feature->GetValue(value) == VmbErrorSuccess)
        {
            return value;
        }
        return std::unexpected(Error::UNKNOWN_ERROR);
    }
    catch (...) { return std::unexpected(Error::UNKNOWN_ERROR); }
}

std::expected<void, Error> CameraAllied::setBlackLevelImpl(double blackLevel)
{
    try
    {
        FeaturePtr feature;
        if (m_camera->GetFeatureByName("BlackLevelSelector", feature) == VmbErrorSuccess &&
            feature->SetValue("All") == VmbErrorSuccess &&
            m_camera->GetFeatureByName("BlackLevel", feature) == VmbErrorSuccess &&
            feature->SetValue(blackLevel) == VmbErrorSuccess)
        {
            return {};
        }
        return std::unexpected(Error::UNKNOWN_ERROR);
    }
    catch (...) { return std::unexpected(Error::UNKNOWN_ERROR); }
}

std::expected<double, Error> CameraAllied::getBlackLevelImpl()
{
    try
    {
        FeaturePtr feature;
        double value;
        if (m_camera->GetFeatureByName("BlackLevel", feature) == VmbErrorSuccess &&
            feature->GetValue(value) == VmbErrorSuccess)
        {
            return value;
        }
        return std::unexpected(Error::UNKNOWN_ERROR);
    }
    catch (...) { return std::unexpected(Error::UNKNOWN_ERROR); }
}

std::expected<void, Error> CameraAllied::setGammaImpl(double gamma)
{
    try
    {
        FeaturePtr feature;
        if (m_camera->GetFeatureByName("Gamma", feature) == VmbErrorSuccess &&
            feature->SetValue(gamma) == VmbErrorSuccess)
        {
            return {};
        }
        return std::unexpected(Error::UNKNOWN_ERROR);
    }
    catch (...) { return std::unexpected(Error::UNKNOWN_ERROR); }
}

std::expected<double, Error> CameraAllied::getGammaImpl()
{
    try
    {
        FeaturePtr feature;
        double value;
        if (m_camera->GetFeatureByName("Gamma", feature) == VmbErrorSuccess &&
            feature->GetValue(value) == VmbErrorSuccess)
        {
            return value;
        }
        return std::unexpected(Error::UNKNOWN_ERROR);
    }
    catch (...) { return std::unexpected(Error::UNKNOWN_ERROR); }
}
