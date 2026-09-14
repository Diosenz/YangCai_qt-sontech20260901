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
 * @file        GaborFilterTypes.h
 * @brief       Gabor 滤波数据类型
 * @details     提供灰度图像校验、增强、形态学和质量评价能力，供条纹、特征和管线模块复用。
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
#ifndef I2R_RLC_VISION_PREPROCESS_GABOR_FILTER_TYPES_H
#define I2R_RLC_VISION_PREPROCESS_GABOR_FILTER_TYPES_H

namespace i2r_rlc::vision {

/**
 * @brief GaborFilter 方向估计使用的仿真激光图像点。
 *
 * 当前新工程未提供无额外依赖的 `MyLaserImagePoint` 定义，因此在
 * `vision/preprocess` 内保留最小字段。下游若 localization 或 digital_twin
 * 提供统一且无 Qt/HALCON/PCL 依赖的公共类型，再替换本兼容结构。
 */
struct MyLaserImagePoint {
    /// @brief X 像素坐标，单位 pixel。
    double px = 0.0;

    /// @brief Y 像素坐标，单位 pixel。
    double py = 0.0;
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_PREPROCESS_GABOR_FILTER_TYPES_H

