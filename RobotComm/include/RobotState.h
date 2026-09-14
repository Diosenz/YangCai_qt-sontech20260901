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
 * @file        RobotState.h
 * @brief       
 * @details     
 * 
 * @version     
 * @author      Windows 10 LTSC 2021
 * @date        2026 年 5 月 18 日
 * 
 * @par         修改日志
 * <table>
 * <tr><th>Version  <th>Author      <th>Date        <th>Description         </tr>
 * <tr><td>         <td>            <td>            <td>                    </tr>
 * </table>
 * 
 ******************************************************************************/

#pragma once
#ifndef i2r_rlc_ROBOT_COMM_ROBOT_STATE_H
#define i2r_rlc_ROBOT_COMM_ROBOT_STATE_H

#include <stdint.h>

namespace i2r_rlc::robot_comm
{
    struct RobotState
    {
        int comnum = 0;
        int sMode = 0;
        int sRemote = 0;
        int sServoPower = 0;
        int sStart = 0;
        int sHold = 0;
        int sIsAlarm = 0;
        int usJobLine = 0;
        int usStep = 0;
        int traflag = 0;
        int corflag = 0;
        int offline = 0;
        int detflag = 0;
        int s_BuffL = 0;
        long s_Bcalib = 0;
        long s_Bsavetraj = 0;
        int OfflineRun = 0;
        long curx_r = 0;
        long cury_r = 0;
        long curz_r = 0;
        long curRx_r = 0;
        long curRy_r = 0;
        long curRz_r = 0;
        long curS1 = 0;
        long curS2 = 0;
        long cur_interpolation = 0;
        long command_speed_x = 0;
        long command_speed_y = 0;
        long command_speed_z = 0;
        long CreateJOB_status = 0;
        int posflag = 0;
        int onlineReverseCompleteFlag = 0;
    };
}

#endif // !i2r_rlc_ROBOT_COMM_ROBOT_STATE_H
