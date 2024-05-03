#ifndef SERIALPORT_HPP_
#define SERIALPORT_HPP_

#pragma once 
//linux
#include <atomic>
#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <dirent.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/netlink.h>
#include <boost/asio.hpp>
#include <fstream>
#include <time.h>
#include <sstream>
//c++
#include <iostream>
#include <vector>
#include <string>
//ros
#include <ros/ros.h>
#include <ros/time.h>

#include <serial/serial.h>

#include "crc_check.hpp"


using namespace boost::asio;
using namespace std;

namespace serialport
{

    #pragma pack(push, 1)
    struct  Receive_Date
    {
        uint8_t header = 0x55;
        uint8_t command;
        uint8_t data_len0;
        uint8_t data_len1;

        uint8_t plan; // 
        uint8_t yaw_up; // yaw轴
        uint8_t yaw_low;
        uint8_t armor_id; //自瞄识别的装甲板id
        uint8_t enemy_dist_up; //视觉距离
        uint8_t enemy_dist_low; //视觉距离
        uint8_t self_color; //己方颜色

        uint8_t Red1_HP_up;
        uint8_t Red1_HP_low;
        uint8_t Red2_HP_up;
        uint8_t Red2_HP_low;
        uint8_t Red3_HP_up;
        uint8_t Red3_HP_low;
        uint8_t Red4_HP_up;
        uint8_t Red4_HP_low;
        uint8_t Red5_HP_up;
        uint8_t Red5_HP_low;
        uint8_t Red7_HP_up;
        uint8_t Red7_HP_low;

        uint8_t Red_Base_HP_up;
        uint8_t Red_Base_HP_low;
        uint8_t Red_Outpost_HP_up; //前哨站
        uint8_t Red_Outpost_HP_low; //前哨站

        uint8_t Blue1_HP_up;
        uint8_t Blue1_HP_low;
        uint8_t Blue2_HP_up;
        uint8_t Blue2_HP_low;
        uint8_t Blue3_HP_up;
        uint8_t Blue3_HP_low;
        uint8_t Blue4_HP_up;
        uint8_t Blue4_HP_low;
        uint8_t Blue5_HP_up;
        uint8_t Blue5_HP_low;
        uint8_t Blue7_HP_up;
        uint8_t Blue7_HP_low;
        uint8_t Blue_Base_HP_up;
        uint8_t Blue_Base_HP_low;
        uint8_t Blue_Outpost_HP_up;
        uint8_t Blue_Outpost_HP_low;

        
        uint8_t Game_state; //比赛流程
        uint8_t Game_remain_time_up;  //剩余时间
        uint8_t Game_remain_time_low;  //剩余时间
        uint8_t projectile_allowance_17mm_up; //剩余允许发弹量
        uint8_t projectile_allowance_17mm_low; //剩余允许发弹量
        uint8_t remain_gold_up;
        uint8_t remain_gold_low;
        uint8_t find_flag;

        uint8_t crc1;
        uint8_t crc2;
        

        Receive_Date(){}
        Receive_Date(Receive_Date *pData) {
            // 构造函数不占内存
//            memcpy(this, pData, sizeof(SerialPortData));
        }
    };
    
    struct Send_Date
    {
        uint8_t header = 0x55;
        uint8_t command = 'c';
        uint8_t data_len0 = 7;  //数据长度
        uint8_t data_len1 = 0xff - data_len0;
        int16_t wheel_vx;
        int16_t wheel_vy;
        int16_t wheel_wz;
        uint8_t arrive_flag;
        // uint8_t restart_flag = 0;
        uint8_t crc1 ;
        uint8_t crc2 ;
        Send_Date(){};
        Send_Date(Send_Date *pdate){};
    };
    

    /**
     *@class  SerialPort
     *@brief  Set serialport, recieve and send data.
     *@param  int fd
    */
    class SerialPort
    {
    public:
        SerialPort(const string &port_name);
        // SerialPort();
        ~SerialPort();
        void serialPortRead(uint8_t *msg, uint8_t max_len);
        void serialPortWrite(uint8_t* msg, int len);
        bool receiveData(Receive_Date *_receive_date);
        void sendDate(Send_Date *_send_date);
        bool Serial_state();
        void serial_close();
        void serial_open();

    private:
        // 创建一个serial对象
        serial::Serial sp;
        // 创建timeout
        serial::Timeout to = serial::Timeout::simpleTimeout(100);
        //crc校验
        CrcCheck crc_check_;
    private:
        serial_port* _serial_port;
        boost::system::error_code _err;
        int _data_len_receive = sizeof(Receive_Date); // sizeof(SerialPortData)
        int _data_len_write = sizeof(Send_Date); // sizeof(SerialPortData)
        bool serial_isopen = false;
        unsigned char msg[sizeof(Send_Date)];
        // unsigned char msg[12];
    };


    //ros系统内读取参数服务器参数
    template<typename T>
    T getParam(const std::string& name,const T& defaultValue)//This name must be namespace+parameter_name
    {
        T v;
        if(ros::param::get(name,v))//get parameter by name depend on ROS.
        {
            ROS_INFO_STREAM("Found parameter: "<<name<<",\tvalue: "<<v);
            return v;
        }
        else 
            ROS_WARN_STREAM("Cannot find value for parameter: "<<name<<",\tassigning default: "<<defaultValue);
        return defaultValue;//if the parameter haven't been set,it's value will return defaultValue.
    }
} // namespace serialport

#endif // SERIALPORT_HPP_
