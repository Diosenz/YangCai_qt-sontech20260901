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
 * @file        Config.h
 * @brief       
 * @details     
 * 
 * @version     1.0
 * @author      RyuYM
 * @date        2026 年 5 月 22 日
 * 
 * @par         修改日志
 * <table>
 * <tr><th>Version  <th>Author      <th>Date        <th>Description         </tr>
 * <tr><td>1.0      <td>RyuYM       <td>260522      <td>创建文件            </tr>
 * </table>
 * 
 ******************************************************************************/

#pragma once
#ifndef I2R_COMM_CAMERA_CONFIG_H
#define I2R_COMM_CAMERA_CONFIG_H

namespace i2r_comm_camera::config
{
    inline constexpr int SHOOT_SINGLE_TIMEOUT = 5000;       ///< 采图超时时间 (ms)
    inline constexpr int SHOOT_CONTINUOUS_TIMEOUT = 100;    ///< 连续采图超时时间 (ms)
}

#endif // !I2R_COMM_CAMERA_CONFIG_H
