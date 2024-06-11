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
    void CONTROL_BASE::ChassisSerialInit(const char* serial)
    {
        chassis_boost_serial_point = new boost::asio::serial_port(chassis_boost_io_service, serial);
        chassis_boost_serial_point->set_option(serial_port::baud_rate(115200));
        chassis_boost_serial_point->set_option(serial_port::flow_control(serial_port::flow_control::none));
        chassis_boost_serial_point->set_option(serial_port::parity(serial_port::parity::none));
        chassis_boost_serial_point->set_option(serial_port::stop_bits(serial_port::stop_bits::one));
        chassis_boost_serial_point->set_option(serial_port::character_size(8));
    }

    //控制器当前状态，球的状态，颜色识别模块1，颜色识别模块2，颜色识别模块3
    bool CONTROL_BASE::ROS_READ_FROM_UPPER(unsigned char &now_controller_state,unsigned char &ball_state,
                             unsigned char &color_flag1,unsigned char &color_flag2,unsigned char &color_flag3)
    {
        unsigned char length;   //数据长度
        
        unsigned char check_value;  //校验位

        try //串口接受数据
        {
            boost::asio::streambuf response;
            boost::system::error_code err;
            boost::asio::read_until(*boost_serial_point,response,"\r\n",err);
            copy(istream_iterator<unsigned char>(istream(&response) >> noskipws), istream_iterator<unsigned char>(),Recieve_buf);
            
        }
        catch(const std::exception& err)
        {
            std::cerr << err.what() << '\n';
        }

        //监察信息头
        for (int i = 0; i < 2; i++)
        {
            if(Recieve_buf[i] != serial_header[i])      //buf[0]    buf[1]
            {
                std::cerr << "read_header_error" << std::endl;
                return false;
            }
        }
        
        //数据长度
        length = Recieve_buf[2];        //buf[2]
        
        //检查信息校验值
        check_value = serial_get_crc8_value(Recieve_buf,length + 3);    //buf[5+3]=buf[8]
        if(check_value != Recieve_buf[3 + length])
        {
            std::cerr << "check_value_error" << std::endl;
            return false;
        }
        //标志位赋值
        now_controller_state = Recieve_buf[3];
        ball_state = Recieve_buf[4];
        color_flag1   = Recieve_buf[5];
        color_flag2   = Recieve_buf[6];
        color_flag3   = Recieve_buf[7];

        return true;
    }

    void CONTROL_BASE::UPPER_TO_STM32(unsigned char next_ctrl_state)
    {
        // 协议数据缓存数组
        
        int i, Length = 0;
        //设置消息头
        for (i = 0; i < 2; i++)
        {
            Upper_Buf[i] = serial_header[i];          //Buf[0] Buf[1]
        }

        Length = 1; //上层控制命令
        Upper_Buf[2] = Length;            //Buf[2]
        // 控制指令
        Upper_Buf[3 + Length - 1] = next_ctrl_state;                  //Buf[3]
        
        // 设置校验值、消息尾
        Upper_Buf[3 + Length] = serial_get_crc8_value(Upper_Buf, 3 + Length);	//buf[4]
        Upper_Buf[3 + Length + 1] = serial_ender[0];                      //buf[5]
        Upper_Buf[3 + Length + 2] = serial_ender[1];                      //buf[6]

        //串口发送数据
        boost::asio::write(*boost_serial_point,boost::asio::buffer(Upper_Buf));
    }


    
    void CONTROL_BASE::CHASSIS_TO_STM32(float chassis_x, float chassis_y, float chassis_w, unsigned char chassis_control_flag,int8_t control_flag)
    {
        // 协议数据缓存数组
        
        int i, Length = 0;

        //底盘命令赋值
        Swerve_Chassis_X.data = chassis_x;
        Swerve_Chassis_Y.data = chassis_y;
        Swerve_Chassis_W.data = chassis_w;

        //设置消息头
        for (i = 0; i < 2; i++)
        {
            Chassis_Buf[i] = serial_header[i];      // 0 1
        }

        Length = 14; //4*3 + 1 + 1= 14
        Chassis_Buf[2] = Length;    //2
        for (i = 0; i < 4; i++) //数据填充
        {
            Chassis_Buf[i+3] = Swerve_Chassis_X.tmp_array[i];       //3 4 5 6
            Chassis_Buf[i+7] = Swerve_Chassis_Y.tmp_array[i];       //7 8 9 10
            Chassis_Buf[i+11] = Swerve_Chassis_W.tmp_array[i];      //11 12 13 14
        }

        // 预留控制指令
        Chassis_Buf[3 + Length - 2] = chassis_control_flag;             //buf[15]   
        Chassis_Buf[3 + Length - 1] = control_flag;                     //buf[16]

        // 设置校验值、消息尾
        Chassis_Buf[3 + Length] = serial_get_crc8_value(Chassis_Buf, 3 + Length);	//buf[17]
        Chassis_Buf[3 + Length + 1] = serial_ender[0];                      //buf[18]
        Chassis_Buf[3 + Length + 2] = serial_ender[1];                      //buf[19]

        //串口发送数据
        boost::asio::write(*chassis_boost_serial_point,boost::asio::buffer(Chassis_Buf));
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
