#include <ros/ros.h>
#include "../include/serialport.hpp"
#include <iostream>
#include <fstream>
#include <time.h>
#include <sstream>
#include <string>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include "rm_robotmsg/Stmdate.h"
#include "rm_robotmsg/RobotState.h"
#include "../../config.h"
#include <thread>
#include <mutex>
#include <unistd.h>
#include "std_msgs/Int8.h"
using namespace serialport;
using namespace std;

#ifdef serial_enable
    SerialPort serialPort("/dev/stm");
#endif

class serial_start
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_cmd;
    ros::Subscriber sub_robotstate;
    ros::Publisher STMdate_pub;
    ros::Subscriber suplly_17mm_sub;

    std::thread receive_STM;
    std::thread send_STM;

    Receive_Date receive_date;
    Send_Date date_send;

    rm_robotmsg::Stmdate STM_date;
    rm_robotmsg::RobotState Robotdate_msg_;

public:

    serial_start()
    {
        ROS_INFO_STREAM("\033[1;32m----> Serialport started....\033[0m");

#ifdef nav_with_rotation
        sub_cmd = nh.subscribe("/base_vel", 1, &serial_start::cmdCallback, this);
#elif
        sub_cmd = nh.subscribe("/cmd_vel_", 1, &serial_start::cmdCallback, this);
#endif
        STMdate_pub = nh.advertise<rm_robotmsg::Stmdate>("/STM_Date", 10);
        suplly_17mm_sub = nh.subscribe("/suplly_17mm", 1, &serial_start::suplly_17mmCallback, this);
        receive_STM = std::thread(&serial_start::receivefromSTM, this);
        send_STM = std::thread(&serial_start::send2STM, this);

        date_send.wheel_vx = 0;
        date_send.wheel_vy = 0;
        date_send.wheel_wz = 0;
   
    }
    ~serial_start()
    {
    }
    void cmdCallback(const geometry_msgs::Twist msg)
    {
        date_send.wheel_vx = msg.linear.x * 1000;  // 600
        date_send.wheel_vy = msg.linear.y * 1000;  //? 700
        date_send.wheel_wz = msg.angular.z * 1200; //  900

        // printf("vel_x= %f, vel_y = %f, vel_w= %f, arrive_flag= %d \n", msg.linear.x, msg.linear.y, msg.angular.z, date_send.arrive_flag);
    }
    void suplly_17mmCallback(const std_msgs::Int8 msg)
    {
        date_send.arrive_flag = msg.data;
    }

    void receivefromSTM()
    {
        while (1)
        {
#ifdef serial_enable
            // printf("1111 \n");
            serialPort.receiveData(&receive_date);

            STM_date.Blue1_HP = receive_date.Blue1_HP_up << 8 | receive_date.Blue1_HP_low;
            STM_date.Blue2_HP = receive_date.Blue2_HP_up << 8 | receive_date.Blue2_HP_low;
            STM_date.Blue3_HP = receive_date.Blue3_HP_up << 8 | receive_date.Blue3_HP_low;
            STM_date.Blue4_HP = receive_date.Blue4_HP_up << 8 | receive_date.Blue4_HP_low;
            STM_date.Blue5_HP = receive_date.Blue5_HP_up << 8 | receive_date.Blue5_HP_low;
            STM_date.Blue7_HP = receive_date.Blue7_HP_up << 8 | receive_date.Blue7_HP_low;
            STM_date.Blue_Base_HP = receive_date.Blue_Base_HP_up << 8 | receive_date.Blue_Base_HP_low;
            STM_date.Blue_Outpost_HP = receive_date.Blue_Outpost_HP_up << 8 | receive_date.Blue_Outpost_HP_low;

            STM_date.Red1_HP = receive_date.Red1_HP_up << 8 | receive_date.Red1_HP_low;
            STM_date.Red2_HP = receive_date.Red2_HP_up << 8 | receive_date.Red2_HP_low;
            STM_date.Red3_HP = receive_date.Red3_HP_up << 8 | receive_date.Red3_HP_low;
            STM_date.Red4_HP = receive_date.Red4_HP_up << 8 | receive_date.Red4_HP_low;
            STM_date.Red5_HP = receive_date.Red5_HP_up << 8 | receive_date.Red5_HP_low;
            STM_date.Red7_HP = receive_date.Red7_HP_up << 8 | receive_date.Red7_HP_low;
            STM_date.Red_Base_HP = receive_date.Red_Base_HP_up << 8 | receive_date.Red_Base_HP_low;
            STM_date.Red_Outpost_HP = receive_date.Red_Outpost_HP_up << 8 | receive_date.Red_Outpost_HP_low;
            
            STM_date.plan = receive_date.plan;
            STM_date.armor_id = receive_date.armor_id;
            STM_date.self_color = receive_date.self_color;
            STM_date.enemy_dist = (float)(((int16_t)(receive_date.enemy_dist_up << 8 | receive_date.enemy_dist_low)) * 0.001f);
            STM_date.yaw = (float) (((int16_t)(receive_date.yaw_up << 8 | receive_date.yaw_low)) * 0.001f);
            STM_date.Game_state = receive_date.Game_state;
            STM_date.Game_remain_time =  receive_date.Game_remain_time_up << 8 | receive_date.Game_remain_time_low;
            STM_date.remain_gold = receive_date.remain_gold_up << 8 | receive_date.remain_gold_low;
            STM_date.projectile_allowance_17mm = receive_date.projectile_allowance_17mm_up << 8 | receive_date.projectile_allowance_17mm_low;
            STM_date.find_flag = receive_date.find_flag;
#else

#endif
            STMdate_pub.publish(STM_date);
        }
    }

    void send2STM()
    {
        while (1)
        {
#ifdef serial_enable
#ifdef serial_send
            // date_send.wheel_vx = 0;
            // date_send.wheel_vy = 0;
            // date_send.wheel_wz = 700;
            // printf("vel_x= %d, vel_y = %d, vel_w= %d, arrive_flag= %d \n", date_send.wheel_vx, date_send.wheel_vy, date_send.wheel_wz, date_send.arrive_flag);
            usleep(10000);  //延时 否则莫名抖动
            serialPort.sendDate(&date_send);
#endif
#endif
        }
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_node");
    serial_start serial_Start_;
    ros::spin();
    return 0;
}
