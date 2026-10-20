/**
 * @file OnlineVisionDiagnosticOptions.h
 * @brief 在线二维检测诊断的会话级配置；不改变算法和控制参数。
 */
#ifndef I2R_RLC_UI_ONLINE_VISION_DIAGNOSTIC_OPTIONS_H
#define I2R_RLC_UI_ONLINE_VISION_DIAGNOSTIC_OPTIONS_H

#include <QtGlobal>

namespace i2r_rlc::ui {

/**
 * @brief 新会话固定的诊断策略；数值逐帧记录，图像仅在限量采样窗口保存。
 *
 * imageSampleEveryN 从第一帧开始按处理序号采样；maxImagePairs 为会话上限。
 * 图像还受压缩后字节预算、有界队列和 512 MiB 剩余空间保护。
 */
struct COnlineVisionDiagnosticOptions {
    bool enabled = true;
    bool imagesEnabled = true;
    int imageSampleEveryN = 15;
    int maxImagePairs = 30;
    qint64 maxImageBytes = 32LL * 1024 * 1024;
};

} // namespace i2r_rlc::ui

#endif // I2R_RLC_UI_ONLINE_VISION_DIAGNOSTIC_OPTIONS_H
