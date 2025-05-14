#ifndef _ORIGINCAR_BASE_H_
#define _ORIGINCAR_BASE_H_

#include <memory>
#include <inttypes.h>
#include "rclcpp/rclcpp.hpp"	   // ROS 2的核心头文件
#include "std_msgs/msg/string.hpp" // 标准字符串消息
#include <csignal>				   // 信号处理
#include <thread>				   // 线程支持

#include <iostream>
#include <string.h>
#include <string>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <rcl/types.h> // ROS 2底层C API类型
#include <sys/stat.h>

#include <serial/serial.h> // 串口通信库
#include <fcntl.h>
#include <stdbool.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp> // 用于电压发布
#include <std_msgs/msg/int32.hpp>	// 用于开关信号
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // TF2坐标变换
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_broadcaster.h>
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp" // 阿克曼转向模型消息
#include "origincar_msg/msg/data.hpp"					  // 自定义消息类型
#include "origincar_msg/msg/sign.hpp"					  // 匹配信号发送
#include <sensor_msgs/msg/imu.hpp>						  // IMU传感器消息
#include <nav_msgs/msg/odometry.hpp>					  // 里程计消息
using namespace std;

// ===== 通信协议相关常量定义 =====
#define SEND_DATA_CHECK 1	 // 发送数据校验标志位
#define READ_DATA_CHECK 0	 // 接收数据校验标志位
#define FRAME_HEADER 0X7B	 // 通信协议帧头 (ASCII '{')
#define FRAME_TAIL 0X7D		 // 通信协议帧尾 (ASCII '}')
#define RECEIVE_DATA_SIZE 24 // 下位机发送过来的数据长度 (字节)
#define SEND_DATA_SIZE 11	 // ROS向下位机发送的数据长度 (字节)
#define PI 3.1415926f		 // 圆周率

// ===== 传感器数据转换比例 =====
#define GYROSCOPE_RATIO 0.00026644f // 陀螺仪数据转换系数: raw -> rad/s
#define ACCEl_RATIO 1671.84f		// 加速度计数据转换系数: raw -> m/s^2

// IMU数据全局变量声明
extern sensor_msgs::msg::Imu Mpu6050;

// ===== 里程计协方差矩阵 =====
// 位姿协方差矩阵 - 正常情况
const double odom_pose_covariance[36] = {1e-3, 0, 0, 0, 0, 0,
										 0, 1e-3, 0, 0, 0, 0,
										 0, 0, 1e6, 0, 0, 0,
										 0, 0, 0, 1e6, 0, 0,
										 0, 0, 0, 0, 1e6, 0,
										 0, 0, 0, 0, 0, 1e3};

// 位姿协方差矩阵 - 非常确定的情况
const double odom_pose_covariance2[36] = {1e-9, 0, 0, 0, 0, 0,
										  0, 1e-3, 1e-9, 0, 0, 0,
										  0, 0, 1e6, 0, 0, 0,
										  0, 0, 0, 1e6, 0, 0,
										  0, 0, 0, 0, 1e6, 0,
										  0, 0, 0, 0, 0, 1e-9};

// 速度协方差矩阵 - 正常情况
const double odom_twist_covariance[36] = {1e-3, 0, 0, 0, 0, 0,
										  0, 1e-3, 0, 0, 0, 0,
										  0, 0, 1e6, 0, 0, 0,
										  0, 0, 0, 1e6, 0, 0,
										  0, 0, 0, 0, 1e6, 0,
										  0, 0, 0, 0, 0, 1e3};

// 速度协方差矩阵 - 非常确定的情况
const double odom_twist_covariance2[36] = {1e-9, 0, 0, 0, 0, 0,
										   0, 1e-3, 1e-9, 0, 0, 0,
										   0, 0, 1e6, 0, 0, 0,
										   0, 0, 0, 1e6, 0, 0,
										   0, 0, 0, 0, 1e6, 0,
										   0, 0, 0, 0, 0, 1e-9};

/**
 * @brief 速度与位置数据结构
 * 用于存储机器人的线速度和角速度，以及位置和姿态
 */
typedef struct __Vel_Pos_Data_
{
	float X; // X方向分量 (位置:m 或 速度:m/s)
	float Y; // Y方向分量 (位置:m 或 速度:m/s)
	float Z; // Z方向分量 (位置时为yaw角:rad 或 角速度:rad/s)

} Vel_Pos_Data;

/**
 * @brief MPU6050 IMU原始传感器数据结构
 * 存储从下位机读取的原始IMU数据
 */
typedef struct __MPU6050_DATA_
{
	short accele_x_data; // X轴加速度原始数据
	short accele_y_data; // Y轴加速度原始数据
	short accele_z_data; // Z轴加速度原始数据
	short gyros_x_data;	 // X轴角速度原始数据
	short gyros_y_data;	 // Y轴角速度原始数据
	short gyros_z_data;	 // Z轴角速度原始数据

} MPU6050_DATA;

/**
 * @brief 发送给下位机的数据结构
 * 包含发送的命令数据和帧结构信息
 */
typedef struct _SEND_DATA_
{
	uint8_t tx[SEND_DATA_SIZE]; // 发送数据缓冲区，数据格式：
								// tx[0]: 帧头(0x7B)
								// tx[1-2]: 预留
								// tx[3-4]: X轴速度(高低字节)
								// tx[5-6]: Y轴速度(高低字节)
								// tx[7-8]: Z轴角速度(高低字节)
								// tx[9]: 校验和
								// tx[10]: 帧尾(0x7D)
	float X_speed;				// X方向线速度 (m/s)
	float Y_speed;				// Y方向线速度 (m/s)
	float Z_speed;				// Z方向角速度 (rad/s)
	unsigned char Frame_Tail;	// 帧尾标识
} SEND_DATA;

/**
 * @brief 从下位机接收的数据结构
 * 包含接收的传感器数据和状态信息
 */
typedef struct _RECEIVE_DATA_
{
	uint8_t rx[RECEIVE_DATA_SIZE]; // 接收数据缓冲区，数据格式：
								   // rx[0]: 帧头(0x7B)
								   // rx[1]: 停止标志
								   // rx[2-3]: X轴速度(高低字节)
								   // rx[4-5]: Y轴速度(高低字节)
								   // rx[6-7]: Z轴角速度(高低字节)
								   // rx[8-19]: IMU数据(6个2字节值)
								   // rx[20-21]: 电池电压
								   // rx[22]: 校验和
								   // rx[23]: 帧尾(0x7D)
	uint8_t Flag_Stop;			   // 停止标志位
	unsigned char Frame_Header;	   // 帧头标识
	float X_speed;				   // X方向线速度 (m/s)
	float Y_speed;				   // Y方向线速度 (m/s)
	float Z_speed;				   // Z方向角速度 (rad/s)
	float Power_Voltage;		   // 电池电压 (V)
	unsigned char Frame_Tail;	   // 帧尾标识
} RECEIVE_DATA;

/**
 * @brief OriginCar机器人基础控制类
 * 处理与下位机的通信、传感器数据处理、位置估计和控制命令发送
 */
class origincar_base : public rclcpp::Node

{
public:
	/**
	 * @brief 构造函数
	 * 初始化ROS2节点，设置参数、发布者和订阅者
	 */
	origincar_base();

	/**
	 * @brief 析构函数
	 * 清理资源
	 */
	~origincar_base();

	/**
	 * @brief 主控制循环
	 * 处理传感器数据，更新机器人状态，发布数据
	 */
	void Control();

	/**
	 * @brief 发布里程计数据
	 * 将机器人位置和速度发布为里程计消息
	 */
	void Publish_Odom();

public:
	serial::Serial Stm32_Serial; // 与STM32下位机通信的串口对象

private:
	/**
	 * @brief 声明ROS2参数
	 */
	void declare_parameters();

	/**
	 * @brief 获取ROS2参数
	 */
	void get_parameters();

	/**
	 * @brief 速度控制回调函数
	 * 处理ROS速度控制命令并转发给下位机
	 * @param twist_aux 速度控制消息
	 */
	void Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux);

	/**
	 * @brief 阿克曼控制回调函数
	 * 处理阿克曼模型控制命令并转发给下位机
	 * @param akm_ctl 阿克曼控制消息
	 */
	void Akm_Cmd_Vel_Callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr akm_ctl);

	/**
	 * @brief 发布IMU传感器数据
	 * 将处理后的IMU数据发布为ROS消息
	 */
	void Publish_ImuSensor();

	/**
	 * @brief 发布电池电压数据
	 * 将电池电压发布为ROS消息
	 */
	void Publish_Voltage();

	/**
	 * @brief 从偏航角创建四元数消息
	 * @param yaw 偏航角(rad)
	 * @return 四元数消息
	 */
	auto createQuaternionMsgFromYaw(double yaw);

	/**
	 * @brief 获取传感器数据
	 * 从下位机读取传感器数据并解析
	 * @return 数据获取是否成功
	 */
	bool Get_Sensor_Data();

	/**
	 * @brief 计算校验和
	 * 用于通信数据的完整性验证
	 * @param Count_Number 需要计算校验和的数据字节数
	 * @param mode 校验模式(0:接收数据校验, 1:发送数据校验)
	 * @return 校验和值
	 */
	unsigned char Check_Sum(unsigned char Count_Number, unsigned char mode);

	/**
	 * @brief IMU数据转换
	 * 将高低字节的IMU原始数据转换为16位短整型
	 * @param Data_High 高字节
	 * @param Data_Low 低字节
	 * @return 转换后的数值
	 */
	short IMU_Trans(uint8_t Data_High, uint8_t Data_Low);

	/**
	 * @brief 里程计数据转换
	 * 将高低字节的里程计原始数据转换为浮点型
	 * @param Data_High 高字节
	 * @param Data_Low 低字节
	 * @return 转换后的数值(米或弧度)
	 */
	float Odom_Trans(uint8_t Data_High, uint8_t Data_Low);

	/**
	 * @brief 标志位切换回调函数
	 * 处理特殊指令(如重置位置等)
	 * @param sign_switch 开关信号
	 */
	void Sign_Switch_Callback(const std_msgs::msg::Int32::SharedPtr sign_switch);

private:
	// ===== 时间相关变量 =====
	rclcpp::Time _Now, _Last_Time; // 时间戳
	float Sampling_Time;		   // 采样周期(秒)

	// ===== ROS2订阅者 =====
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Cmd_Vel_Sub;						 // 速度控制订阅者
	rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr Akm_Cmd_Vel_Sub; // 阿克曼控制订阅者

	// ===== ROS2发布者 =====
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;	// 里程计发布者
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_publisher; // 电池电压发布者
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;		// IMU数据发布者

	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr test_publisher; // 测试用发布者

	rclcpp::Publisher<origincar_msg::msg::Data>::SharedPtr robotpose_publisher; // 机器人位置发布者(自定义消息)
	rclcpp::Publisher<origincar_msg::msg::Data>::SharedPtr robotvel_publisher;	// 机器人速度发布者(自定义消息)

	// ===== TF2变换 =====
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_bro;			// TF广播器
	rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_; // TF消息发布者

	// ===== 计时器 =====
	rclcpp::TimerBase::SharedPtr test_timer; // 测试计时器

	rclcpp::TimerBase::SharedPtr odom_timer;	// 里程计计时器
	rclcpp::TimerBase::SharedPtr imu_timer;		// IMU计时器
	rclcpp::TimerBase::SharedPtr voltage_timer; // 电压计时器

	rclcpp::TimerBase::SharedPtr robotpose_timer; // 位置计时器
	rclcpp::TimerBase::SharedPtr robotvel_timer;  // 速度计时器

	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; // TF广播器(另一种实现)

	// ===== 其他订阅者 =====
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr Sign_Switch_Sub; // 开关信号订阅者

	// ===== 参数变量 =====
	string usart_port_name, robot_frame_id, gyro_frame_id, odom_frame_id, akm_cmd_vel, test;
	std::string cmd_vel;
	int serial_baud_rate; // 串口波特率

	// ===== 数据结构 =====
	RECEIVE_DATA Receive_Data; // 接收数据缓冲区
	SEND_DATA Send_Data;	   // 发送数据缓冲区

	Vel_Pos_Data Robot_Pos;	   // 机器人位置
	Vel_Pos_Data Robot_Vel;	   // 机器人速度
	MPU6050_DATA Mpu6050_Data; // IMU原始数据
	float Power_voltage;	   // 电池电压
	size_t count_;			   // 计数器
};

#endif //_ORIGINCAR_BASE_H_
