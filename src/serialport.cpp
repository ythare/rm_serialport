#include "../include/serialport.hpp"


namespace serialport
{
    SerialPort::SerialPort(const string &port_name)
    {
        try
        {
            // 设置要打开的串口名称
            sp.setPort(port_name);
            // 设置串口通信的波特率
            sp.setBaudrate(115200);
            // 串口设置timeout
            sp.setTimeout(to);
            // 打开串口
            sp.open();
            // 判断串口是否打开成功
            if (sp.isOpen())
            {
                ROS_INFO_STREAM("/dev/ttyACM* is opened!!");
                serial_isopen = true;
            }
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR_STREAM("Unable to open port.");
        }
        catch (...)
        {
            ROS_ERROR( "create serial port object error! ");
        }
    }

    // SerialPort::SerialPort()
    // {}

    SerialPort::~SerialPort()
    {
        sp.close();
        delete _serial_port;
        ROS_INFO_STREAM("/dev/ttyACM* is closed.");
    }

    void SerialPort::serialPortRead(uint8_t *msg, uint8_t max_len)
    {
        try
        {
            read(*_serial_port, boost::asio::buffer(msg, max_len), _err);
        }
        catch (...)
        {
            int aaa=11;
            ROS_ERROR("readData from serial port error! ");
            exit(0);
        }
    }

    void SerialPort::serialPortWrite(uint8_t *msg, int len)
    {
        // try
        // {
        //     write(*_serial_port, buffer(msg, (size_t)len));
        // }
        // catch (...)
        // {
        //     ROS_ERROR("write to serial port error! ");
        //     exit(0);
        // }
    }

    bool SerialPort::receiveData(Receive_Date *_receive_date)
    {
        unsigned char _date_temp[sizeof(Receive_Date)];
        int n = sp.read(_date_temp, _data_len_receive);
        // printf("date0:%d , date1:%d, date2:%d, crc1:%d, crc2:%d , _data_len_receive:%d, sp.read_len:%d \n ",
        // _date_temp[0], _date_temp[1], _date_temp[2], _date_temp[_data_len_receive-2], _date_temp[_data_len_receive-1], _data_len_receive, n);
        if (n == _data_len_receive)
        {
            if (_date_temp[0] == 0x55 
                && crc_check_.Verify_CRC16_Check_Sum(_date_temp, _data_len_receive-2))
            {
                memcpy( (unsigned char *)_receive_date, _date_temp, _data_len_receive);
                return true;
            }
            // else  ROS_ERROR("crc is false");
            //有时候会卡在这里
        }
        // else ROS_ERROR("date lenth  is false");
        
        return false;
    }

    void SerialPort::sendDate(Send_Date *_send_date)
    {
        memcpy(msg,_send_date,_data_len_write);
        // ROS_DEBUG("date_len:%d ,ressived_date_len:%d,  crc1:%d, crc2:%d", _data_len_write, _send_date[2],  _send_date[10], _send_date[11]);
        crc_check_.Append_CRC16_Check_Sum(msg,  11);
        // ROS_DEBUG("date_len:%d , crc1:%d, crc2:%d", _data_len_write, &_send_date[10], &_send_date[11]);
        sp.write(msg, 13);
    }
    bool SerialPort::Serial_state()
    {
        return serial_isopen;
    }
    void SerialPort::serial_open()
    {
        sp.open();
    }

    void SerialPort::serial_close()
    {
        sp.close();
    }

}