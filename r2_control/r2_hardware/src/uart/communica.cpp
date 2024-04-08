#include "r2_hardware/communica.h"

using namespace std;
using namespace boost::asio;


namespace control_base
{
    CONTROL_BASE::CONTROL_BASE()
    {
    }

    

    void CONTROL_BASE::SerialInit(const char* serial)
    {
        boost_serial_point = new boost::asio::serial_port(boost_io_service, serial);
        boost_serial_point->set_option(serial_port::baud_rate(115200));
        boost_serial_point->set_option(serial_port::flow_control(serial_port::flow_control::none));
        boost_serial_point->set_option(serial_port::parity(serial_port::parity::none));
        boost_serial_point->set_option(serial_port::stop_bits(serial_port::stop_bits::one));
        boost_serial_point->set_option(serial_port::character_size(8));
    }


    //开关门状态，摩擦轮状态，滚筒状态，上位机开关，自动路径开关
    bool CONTROL_BASE::ROS_READ_FROM_STM32(unsigned char &current_bp_state,unsigned char &current_fw_state,unsigned char &is_ball_in_car,
                             unsigned char &color_flag1,unsigned char &color_flag2,unsigned char &color_flag3)
    {
        unsigned char length;   //数据长度
        
        unsigned char check_value;  //校验位

        try //串口接受数据
        {
            boost::asio::streambuf response;
            boost::system::error_code err;
            boost::asio::read_until(*boost_serial_point,response,"\r\n",err);
            copy(istream_iterator<unsigned char>(istream(&response) >> noskipws), istream_iterator<unsigned char>(),recieve_buf);
            
        }
        catch(const std::exception& err)
        {
            std::cerr << err.what() << '\n';
        }

        //监察信息头
        for (int i = 0; i < 2; i++)
        {
            if(recieve_buf[i] != serial_header[i])      //buf[0]    buf[1]
            {
                std::cerr << "read_header_error" << std::endl;
                return false;
            }
        }
        
        //数据长度
        length = recieve_buf[2];        //buf[2]
        
        //检查信息校验值
        check_value = serial_get_crc8_value(recieve_buf,length + 3);    //buf[6+3]=buf[9]
        if(check_value != recieve_buf[3 + length])
        {
            std::cerr << "check_value_error" << std::endl;
            return false;
        }
        //标志位赋值
        current_bp_state = recieve_buf[3];
        current_fw_state = recieve_buf[4];
        is_ball_in_car   = recieve_buf[5];
        color_flag1      = recieve_buf[6];
        color_flag2      = recieve_buf[7];
        color_flag3      = recieve_buf[8];

        return true;
    }

    void CONTROL_BASE::ROS_WRITE_TO_STM32(float chassis_x, float chassis_y, float chassis_w, unsigned char chassis_control_flag, 
                        unsigned char BP_ctrl_state, unsigned char FW_ctrl_state,unsigned char FW_open_flag)
    {
        // 协议数据缓存数组
        
        int i, Length = 0;

        //底盘命令赋值
        RosToStm32_Chassis_X.data = chassis_x;
        RosToStm32_Chassis_Y.data = chassis_y;
        RosToStm32_Chassis_W.data = chassis_w;

        //设置消息头
        for (i = 0; i < 2; i++)
        {
            Buf[i] = serial_header[i];
        }

        Length = 16; //4*3 + 1 +1 + 1 +1= 16
        Buf[2] = Length;    
        for (i = 0; i < 4; i++) //数据填充
        {
            Buf[i+3] = RosToStm32_Chassis_X.tmp_array[i];
            Buf[i+7] = RosToStm32_Chassis_Y.tmp_array[i];
            Buf[i+11] = RosToStm32_Chassis_W.tmp_array[i];
        }

        // 预留控制指令
        Buf[3 + Length - 4] = chassis_control_flag;             //buf[15]   
        Buf[3 + Length - 3] = BP_ctrl_state;                    //buf[16]
        Buf[3 + Length - 2] = FW_ctrl_state;                    //buf[17]
        Buf[3 + Length - 1] = FW_open_flag;                  //Buf[18]
        
        // 设置校验值、消息尾
        Buf[3 + Length] = serial_get_crc8_value(Buf, 3 + Length);	//buf[19]
        Buf[3 + Length + 1] = serial_ender[0];                      //buf[20]
        Buf[3 + Length + 2] = serial_ender[1];                      //buf[21]

        //串口发送数据
        boost::asio::write(*boost_serial_point,boost::asio::buffer(Buf));
    }

    //数据冗余检查
    unsigned char CONTROL_BASE::serial_get_crc8_value(unsigned char *data, unsigned char len)
    {
        unsigned char crc = 0;
        unsigned char i;
        while(len--)
        {
            crc ^= *data++;
            for(i = 0; i < 8; i++)
            {
                if(crc&0x01)
                    crc=(crc>>1)^0x8C;
                else
                    crc >>= 1;
            }
        }
        return crc;
    }

} // namespace control_base
