#ifndef __COMMUNICA_H
#define __COMMUNICA_H

#include "ros/ros.h"
#include "boost/asio.hpp"
#include "stdint.h"
#include "r2_msgs/stm32.h"
#include "r2_msgs/controller_cmd.h"

typedef enum
{
    FW_INVERTED=2,  //摩擦轮反转
    FW_TAKE_BALL,   //摩擦轮取球
    FW_CONTROLLER_OFF,
    BP_SHOOT_BALL,  //滚筒出球
    BP_ABANDON_BALL,//滚筒筛球
	BP_INVERTED,    //滚筒反转
    BP_CONTROLLER_OFF,  //机构关闭

    LINEAR_ACTUATOR_GO,
    LINEAR_ACTUATOR_BACK,
    LINEAR_ACTUATOR_OFF
}CONTROLLER_STATE;

namespace control_base
{
class CONTROL_BASE
{
public:
    boost::asio::serial_port* boost_serial_point;       //上层串口
    boost::asio::serial_port* chassis_boost_serial_point;     //底盘串口
    bool ROS_READ_FROM_STM32(unsigned char &current_bp_state,unsigned char &current_fw_state,unsigned char &is_ball_in_car,
                             unsigned char &color_flag1,unsigned char &color_flag2,unsigned char &color_flag3);

    void ROS_WRITE_TO_STM32(float chassis_x, float chassis_y, float chassis_w, unsigned char chassis_control_flag, 
                            unsigned char BP_ctrl_state, unsigned char FW_ctrl_state,unsigned char is_fw_open_flag);
    //底盘发送
    void CHASSIS_TO_STM32(float chassis_x, float chassis_y, float chassis_w, unsigned char chassis_control_flag);                            
    CONTROL_BASE();
    void SerialInit(const char* serial);    //上层串口初始化
    void ChassisSerialInit(const char* serial); //底盘串口初始化
    unsigned char serial_get_crc8_value(unsigned char *data, unsigned char len);


private:
    boost::asio::io_service boost_io_service;   //上层io_service
    boost::asio::io_service chassis_boost_io_service;    //底盘io_service 
    // std::string stm32_serial_port_;
    ros::Publisher pub;
    unsigned char recieve_buf[20] = {0}; 
    unsigned char Buf[22] = {0};
    unsigned char Chassis_Buf[19] = {0}; //底盘发送缓存区   
    union Recieve_Union0
    {
        short data;
        unsigned char tmp_array[2];
    }Stm32ToRos_RPM1,Stm32ToRos_RPM2,Stm32ToRos_RPM3,Stm32ToRos_RPM4;

    union Recieve_Union1
    {
        float data;
        unsigned char tmp_array[4];
    }ROBOT_POS_X,ROBOT_POS_Y,ROBOT_POS_YAW;

    union Send_Union0
    {
        float data;
        unsigned char tmp_array[4];
    }RosToStm32_Chassis_X,RosToStm32_Chassis_Y,RosToStm32_Chassis_W;
    union Send_Union1
    {
        float data;
        unsigned char tmp_array[4];
    }Swerve_Chassis_X,Swerve_Chassis_Y,Swerve_Chassis_W;
    const unsigned char serial_header[2] = {0x55,0xaa};
    const unsigned char serial_ender[2] = {0x0d,0x0a};
};
} // namespace control_base



#endif
