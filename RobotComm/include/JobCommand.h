/*******************************************************************************
 * @file        JobCommand.h
 * @brief       机器人作业启动命令数据结构
 ******************************************************************************/
#ifndef I2R_RLC_ROBOT_COMM_JOB_COMMAND_H
#define I2R_RLC_ROBOT_COMM_JOB_COMMAND_H

#include <string>

namespace i2r_rlc::robot_comm {

/**
 * @brief 启动控制柜已有 JBI 作业的命令。
 *
 * 对应旧工程 `jobstart = 1` 与 `jobname_dt`。作业名直接作为旧 J 协议最后
 * 一个字段发送，因此不得包含空白字符或逗号。
 */
struct RobotJobStartCommand {
    /// @brief 控制柜中已有的 JBI 作业名称。
    std::string jobName;
    /// @brief true 表示请求启动该作业。
    bool requestStart = false;
    /// @brief true 表示命令内容已经由调用方确认。
    bool valid = false;
    /// @brief 诊断说明。
    std::string message;
};

} // namespace i2r_rlc::robot_comm

#endif // I2R_RLC_ROBOT_COMM_JOB_COMMAND_H
