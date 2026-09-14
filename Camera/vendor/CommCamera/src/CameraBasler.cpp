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
 * @file        CameraCRTPBasler.cpp
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

#include <magic_enum/magic_enum.hpp>
#include <pylon/PylonIncludes.h>

#include "CameraBasler.h"

using namespace Pylon;
using namespace i2r_comm_camera;

namespace
{
    class BaslerSDKInitializer
    {
    private:
        BaslerSDKInitializer() { PylonInitialize(); }   // 巴斯勒 SDK 自动初始化
        ~BaslerSDKInitializer() { PylonTerminate(); }   // 巴斯勒 SDK 自动终止

    public:
        static BaslerSDKInitializer& instance()
        {
            static BaslerSDKInitializer s_instance;
            return s_instance;
        }
    };

    cv::Mat convertImage2CVMat(const CGrabResultPtr& ptrGrabResult)
    {
        const int width = ptrGrabResult->GetWidth();
        const int height = ptrGrabResult->GetHeight();
        void* buffer = ptrGrabResult->GetBuffer();

        cv::Mat mat;

        switch (ptrGrabResult->GetPixelType())
        {
        case PixelType_Mono8: mat = cv::Mat(height, width, CV_8UC1, buffer); break;
        }

        return mat;
    }
}

CameraBasler::CameraBasler() : CameraCRTP(TypeCamera::BASLER)
{
    BaslerSDKInitializer::instance();
}

std::expected<void, Error> CameraBasler::setPixelFormat(PixelFormat pixelFormat)
{
    if (!isConnected()) return std::unexpected(Error::CAMERA_NOT_CONNECTED);

    try
    {
        CEnumParameter(m_camera.GetNodeMap(), "PixelFormat").
            SetValue(magic_enum::enum_name(pixelFormat).data());
        return {};
    }
    catch (...) { return std::unexpected(Error::UNKNOWN_ERROR); }
}

/**
 * @brief       获取相机像素格式
 * @details
 */
std::expected<std::string, Error> CameraBasler::getPixelFormat()
{
    if (!isConnected()) return std::unexpected(Error::CAMERA_NOT_CONNECTED);
    try { return CEnumParameter(m_camera.GetNodeMap(), "PixelFormat").GetValue().c_str(); }
    catch (...) { return std::unexpected(Error::UNKNOWN_ERROR); }
}

std::expected<void, Error> CameraBasler::connectImpl(unsigned int cameraIndex)
{
    // 枚举所有相机
    auto& tlFactory = CTlFactory::GetInstance();
    DeviceInfoList_t deviceList;
    if (tlFactory.EnumerateDevices(deviceList) == 0 ||
        cameraIndex >= deviceList.size())
    {
        return std::unexpected(Error::CAMERA_NOT_FOUND);
    }

    try
    {
        m_camera.Attach(tlFactory.CreateDevice(deviceList[cameraIndex]));   // 连接相机
        m_camera.Open();                                                    // 打开相机

        CEnumParameter(m_camera.GetNodeMap(), "TriggerMode").SetValue("Off");
        CEnumParameter(m_camera.GetNodeMap(), "PixelFormat").SetValue("Mono8");
    }
    catch (...) { return std::unexpected(Error::CAMERA_INIT_FAILED); }

    return {};
}

void CameraBasler::disconnectImpl()
{
    try
    {
        if (m_camera.IsGrabbing()) m_camera.StopGrabbing(); // 断开连接
        m_camera.Close();
        m_camera.DetachDevice();
    }
    catch (...) {}  // 忽略断开时异常
}

std::expected<cv::Mat, Error> CameraBasler::grabSingleImpl(int timeout)
{
    try
    {
        CGrabResultPtr ptrGrabResult;

        if (isShootingContinuous())
        {
            if (!m_camera.RetrieveResult(timeout, ptrGrabResult, TimeoutHandling_Return))
            {
                return std::unexpected(Error::SHOOT_TIMEOUT);
            }
        }
        else
        {
            m_camera.GrabOne(timeout, ptrGrabResult, TimeoutHandling_ThrowException);
        }

        if (!ptrGrabResult->GrabSucceeded()) { return std::unexpected(Error::SHOOT_FAILED); }

        return convertImage2CVMat(ptrGrabResult);
    }
    catch (const TimeoutException&) { return std::unexpected(Error::SHOOT_TIMEOUT); }
    catch (...) { return std::unexpected(Error::SHOOT_FAILED); }
}

std::expected<void, Error> CameraBasler::grabContinuousStartImpl()
{
    try
    {
        m_camera.StartGrabbing(GrabStrategy_LatestImageOnly, GrabLoop_ProvidedByUser);
        return {};
    }
    catch (...) { return std::unexpected(Error::SHOOT_FAILED); }
}

void CameraBasler::grabContinuousStopImpl()
{
    try
    {
        if (m_camera.IsGrabbing())
        {
            m_camera.StopGrabbing();
        }
    }
    catch (...) {}  // 忽略停止时异常
}

std::expected<void, Error> CameraBasler::setExposureTimeImpl(double time)
{
    try
    {
        CEnumParameter(m_camera.GetNodeMap(), "ExposureAuto").SetValue("Off");
        CFloatParameter(m_camera.GetNodeMap(), "ExposureTime").SetValue(time);
        return {};
    }
    catch (...) { return std::unexpected(Error::UNKNOWN_ERROR); }
}

std::expected<double, Error> CameraBasler::getExposureTimeImpl()
{
    try { return CFloatParameter(m_camera.GetNodeMap(), "ExposureTime").GetValue(); }
    catch (...) { return std::unexpected(Error::UNKNOWN_ERROR); }
}

std::expected<void, Error> CameraBasler::setGainImpl(double gain)
{
    try
    {
        CEnumParameter(m_camera.GetNodeMap(), "GainAuto").SetValue("Off");
        CEnumParameter(m_camera.GetNodeMap(), "GainSelector").SetValue("All");
        CFloatParameter(m_camera.GetNodeMap(), "Gain").SetValue(gain);
        return {};
    }
    catch (...) { return std::unexpected(Error::UNKNOWN_ERROR); }
}

std::expected<double, Error> CameraBasler::getGainImpl()
{
    try { return CFloatParameter(m_camera.GetNodeMap(), "Gain").GetValue(); }
    catch (...) { return std::unexpected(Error::UNKNOWN_ERROR); }
}

std::expected<void, Error> CameraBasler::setBlackLevelImpl(double blackLevel)
{
    try
    {
        CEnumParameter(m_camera.GetNodeMap(), "BlackLevelSelector").SetValue("All");
        CFloatParameter(m_camera.GetNodeMap(), "BlackLevel").SetValue(blackLevel);
        return {};
    }
    catch (...) { return std::unexpected(Error::UNKNOWN_ERROR); }
}

std::expected<double, Error> CameraBasler::getBlackLevelImpl()
{
    try { return CFloatParameter(m_camera.GetNodeMap(), "BlackLevel").GetValue(); }
    catch (...) { return std::unexpected(Error::UNKNOWN_ERROR); }
}

std::expected<void, Error> CameraBasler::setGammaImpl(double gamma)
{
    try
    {
        CFloatParameter(m_camera.GetNodeMap(), "Gamma").SetValue(gamma);
        return {};
    }
    catch (...) { return std::unexpected(Error::UNKNOWN_ERROR); }
}

std::expected<double, Error> CameraBasler::getGammaImpl()
{
    try { return CFloatParameter(m_camera.GetNodeMap(), "Gamma").GetValue(); }
    catch (...) { return std::unexpected(Error::UNKNOWN_ERROR); }
}
