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
 * @file        WeldStartPointDetector.h
 * @brief       焊缝起点检测接口
 * @details     提供 T 型、对接、搭接、gap、端点和仿真激光定位等焊缝语义检测能力。
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
#ifndef I2R_RLC_VISION_FEATURE_WELD_START_POINT_DETECTOR_H
#define I2R_RLC_VISION_FEATURE_WELD_START_POINT_DETECTOR_H

#include <opencv2/core.hpp>

#include "feature/WeldStartPointTypes.h"

namespace i2r_rlc::vision {

/**
 * @brief 焊缝起点/终点检测器。
 *
 * 本类承接旧 `StartPtDet`、`StartPtDet_Area`、`ButtStartPtDet` 和
 * `ButtEndPtsFinding` 的入口边界。内部复用已迁移主动轮廓、gap 扫描、
 * split/RANSAC 和历史备用，不恢复 HALCON 显示对象和全局变量。
 */
class CWeldStartPointDetector final {
public:
    /**
     * @brief 从灰度图检测起点/终点。
     * @param gray8 输入 `CV_8UC1` 灰度图。
     * @param options 起点/终点检测参数，距离单位为 pixel。
     * @param state 可选跨帧状态；函数会更新历史点列，不持有该指针。
     * @return 起点/终点检测结果。
     */
    static WeldStartPointResult detect(
        const cv::Mat &gray8,
        const WeldStartPointOptions &options,
        WeldStartPointState *state);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_FEATURE_WELD_START_POINT_DETECTOR_H

