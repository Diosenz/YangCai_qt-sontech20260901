/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_camera_vendor_smoke.cpp
 * @brief       camera vendor smoke 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <iostream>

#if defined(I2R_RLC_ENABLE_CAMERA_ALLIED)
#include "AlliedCameraDevice.h"
#endif

#if defined(I2R_RLC_ENABLE_CAMERA_BASLER)
#include "BaslerCameraDevice.h"
#endif

namespace {

bool expectTrue(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }

    return true;
}

} // namespace

/**
 * @brief 真实相机适配层轻量级测试入口。
 *
 * 该测试只验证已启用 vendor 适配类的构造和未启动状态，不连接真实相机，
 * 不触发厂商 SDK 采集流程。
 */
int main()
{
    bool ok = true;

#if defined(I2R_RLC_ENABLE_CAMERA_ALLIED)
    i2r_rlc::camera::AlliedCameraDevice alliedDevice;
    ok &= expectTrue(!alliedDevice.stateText().isEmpty(),
        "Allied adapter should report initial state");
    ok &= expectTrue(alliedDevice.latestFrame().image.isNull(),
        "Allied adapter should return an empty frame before start");
#endif

#if defined(I2R_RLC_ENABLE_CAMERA_BASLER)
    i2r_rlc::camera::BaslerCameraDevice baslerDevice;
    ok &= expectTrue(!baslerDevice.stateText().isEmpty(),
        "Basler adapter should report initial state");
    ok &= expectTrue(baslerDevice.latestFrame().image.isNull(),
        "Basler adapter should return an empty frame before start");
#endif

    if (!ok) {
        return 1;
    }

    std::cout << "PASS: camera vendor smoke test\n";
    return 0;
}
