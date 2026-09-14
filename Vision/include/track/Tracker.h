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
 * @file        Tracker.h
 * @brief       图像目标跟踪接口
 * @details     定义 KCF 迁移使用的最小跟踪器基类，保持原工程 init/update 调用语义。
 *
 * @version     1.1
 * @author      LT
 * @date        2026 年 6 月 8 日
 *
 * @par         修改日志
 * <table>
 * <tr><th>Version  <th>Author      <th>Date        <th>Description         </tr>
 * <tr><td>1.0      <td>LT          <td>260608      <td>创建文件            </tr>
 * <tr><td>1.1      <td>LT          <td>260608      <td>初步完成            </tr>
 * </table>
 *
 ******************************************************************************/
#ifndef I2R_RLC_VISION_TRACK_CTRACKER_H
#define I2R_RLC_VISION_TRACK_CTRACKER_H

#include <opencv2/core.hpp>

namespace i2r_rlc::vision {

/**
 * @brief 图像目标跟踪器基类。
 *
 * `Tracker` 基类， KCF 所需的初始化和逐帧更新语义。
 * 输入坐标单位为 pixel，调用方负责保证图像生命周期。
 */
class CTracker {
public:
    CTracker() = default;
    virtual ~CTracker() = default;

    /**
     * @brief 使用初始 ROI 和图像初始化跟踪器。
     * @param roi 初始目标矩形，单位 pixel。
     * @param image 初始图像，支持 `CV_8UC1` 或 `CV_8UC3`。
     */
    virtual void init(const cv::Rect &roi, cv::Mat image) = 0;

    /**
     * @brief 在当前帧中更新目标位置。
     * @param image 当前帧图像，支持 `CV_8UC1` 或 `CV_8UC3`。
     * @return 目标矩形，单位 pixel。
     */
    virtual cv::Rect update(cv::Mat image) = 0;

    /// @brief 当前跟踪 ROI，单位 pixel；保留原工程直接读写 `_roi` 的调试便利性。
    cv::Rect_<float> m_roi;
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_TRACK_CTRACKER_H
