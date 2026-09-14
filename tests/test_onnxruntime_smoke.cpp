/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_onnxruntime_smoke.cpp
 * @brief       onnxruntime smoke 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <iostream>

#include <onnxruntime_cxx_api.h>

/**
 * @brief ONNX Runtime CPU 版 CMake 接入冒烟测试入口。
 *
 * 该测试只验证 C++ API 头文件、导入库和运行时 DLL 链路可用，
 * 不加载真实模型，也不启用 DirectML、CUDA 或其他加速后端。
 */
int main()
{
    try
    {
        Ort::Env env(ORT_LOGGING_LEVEL_ERROR, "i2r_rlc_onnxruntime_smoke");
        const OrtApiBase *apiBase = OrtGetApiBase();
        if (apiBase == nullptr || apiBase->GetApi(ORT_API_VERSION) == nullptr)
        {
            std::cerr << "FAIL: ONNX Runtime API base is not available\n";
            return 1;
        }

        std::cout << "PASS: ONNX Runtime smoke test\n";
        std::cout << "ONNX Runtime version: "
                  << apiBase->GetVersionString() << '\n';
    }
    catch (const Ort::Exception &exception)
    {
        std::cerr << "FAIL: ONNX Runtime exception: "
                  << exception.what() << '\n';
        return 1;
    }

    return 0;
}
