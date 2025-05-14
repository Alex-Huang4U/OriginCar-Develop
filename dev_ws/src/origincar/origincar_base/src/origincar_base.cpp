#include "origincar_base/origincar_base.h"
#include "rclcpp/rclcpp.hpp"
#include "origincar_base/Quaternion_Solution.h"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "origincar_msg/msg/data.hpp"

using std::placeholders::_1;
using namespace std;
void sigintHandler(int sig);
sensor_msgs::msg::Imu Mpu6050; // IMU传感器数据全局变量
rclcpp::Node::SharedPtr node_handle = nullptr;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  signal(SIGINT, sigintHandler); // 注册信号处理函数，用于优雅地关闭机器人
  origincar_base Robot_Control;
  Robot_Control.Control(); // 启动机器人控制循环
  rclcpp::shutdown();
  return 0;
}

/**
 * @brief 将IMU原始数据从高低字节转换为16位短整型
 * @param Data_High 高字节数据
 * @param Data_Low 低字节数据
 * @return 转换后的16位有符号整数值
 */
short origincar_base::IMU_Trans(uint8_t Data_High, uint8_t Data_Low)
{
  short transition_16;
  transition_16 = 0;
  transition_16 |= Data_High << 8; // 高字节左移8位
  transition_16 |= Data_Low;       // 合并低字节
  return transition_16;
}

/**
 * @brief 将里程计原始数据从高低字节转换为浮点型数据
 * @param Data_High 高字节数据
 * @param Data_Low 低字节数据
 * @return 转换后的浮点数值，单位为米或弧度
 */
float origincar_base::Odom_Trans(uint8_t Data_High, uint8_t Data_Low)
{
  float data_return;
  short transition_16;
  transition_16 = 0;
  transition_16 |= Data_High << 8;                                       // 高字节左移8位
  transition_16 |= Data_Low;                                             // 合并低字节
  data_return = (transition_16 / 1000) + (transition_16 % 1000) * 0.001; // 转换为小数形式，提供精度
  return data_return;
}

/**
 * @brief 阿克曼转向模型控制回调函数 - 处理转向角度和速度指令发送到下位机
 * @param akm_ctl 阿克曼控制消息
 */
void origincar_base::Akm_Cmd_Vel_Callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr akm_ctl)
{
  short transition;

  Send_Data.tx[0] = FRAME_HEADER; // 下位机通信帧头
  Send_Data.tx[1] = 0;            // 预留字节
  Send_Data.tx[2] = 0;            // 预留字节

  // 转换并发送速度指令
  transition = 0;
  transition = akm_ctl->drive.speed * 1000; // 将速度值放大1000倍以保持精度
  Send_Data.tx[4] = transition;             // 速度低字节
  Send_Data.tx[3] = transition >> 8;        // 速度高字节

  // 转换并发送转向角度指令
  transition = 0;
  transition = akm_ctl->drive.steering_angle * 1000 / 2; // 将转向角度值放大1000倍并除以2进行缩放
  Send_Data.tx[8] = transition;                          // 转向角度低字节
  Send_Data.tx[7] = transition >> 8;                     // 转向角度高字节

  Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK); // 计算校验和
  Send_Data.tx[10] = FRAME_TAIL;                   // 帧尾

  try
  {
    Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx)); // 通过串口发送数据到STM32下位机
  }
  catch (serial::IOException &e)
  {
    RCLCPP_ERROR(this->get_logger(), ("Unable to send data through serial port"));
  }
}

/**
 * @brief 处理速度控制回调函数 - 处理线速度和角速度指令发送到下位机
 * @param twist_aux Twist消息，包含线速度和角速度
 */
void origincar_base::Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{
  short transition;
  Send_Data.tx[0] = FRAME_HEADER; // 下位机通信帧头
  Send_Data.tx[1] = 0;            // 预留字节
  Send_Data.tx[2] = 0;            // 预留字节

  // 转换并发送X方向线速度
  transition = 0;
  transition = twist_aux->linear.x * 1000; // 将速度值放大1000倍以保持精度
  Send_Data.tx[4] = transition;            // X速度低字节
  Send_Data.tx[3] = transition >> 8;       // X速度高字节

  // 转换并发送Y方向线速度
  transition = 0;
  transition = twist_aux->linear.y * 1000; // 将速度值放大1000倍以保持精度
  Send_Data.tx[6] = transition;            // Y速度低字节
  Send_Data.tx[5] = transition >> 8;       // Y速度高字节

  // 转换并发送Z方向角速度
  transition = 0;
  transition = (twist_aux->angular.z) * 1000; // 将角速度值放大1000倍以保持精度
  Send_Data.tx[8] = transition;               // 角速度低字节
  Send_Data.tx[7] = transition >> 8;          // 角速度高字节

  Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK); // 计算校验和
  Send_Data.tx[10] = FRAME_TAIL;                   // 帧尾

  try
  {
    if (akm_cmd_vel == "none")
    {                                                         // 只有当阿克曼模式未启用时才发送普通速度指令
      Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx)); // 通过串口发送数据到STM32下位机
    }
  }
  catch (serial::IOException &e)
  {
    RCLCPP_ERROR(this->get_logger(), ("Unable to send data through serial port"));
  }
}

/**
 * @brief 标志位切换回调函数 - 暂未使用
 * @param sign_switch 开关消息
 */
void origincar_base::Sign_Switch_Callback(const std_msgs::msg::Int32::SharedPtr sign_switch)
{
  (void)sign_switch;
  /* if (sign_switch->data == -1) {
       memset(&Robot_Pos, 0, sizeof(Robot_Pos));
       Robot_Pos.X = 0.5;
       Robot_Pos.Y = 0.2;
       memset(&Robot_Vel, 0, sizeof(Robot_Vel));
   }
   else if (sign_switch->data == 6) {
       memset(&Robot_Pos, 0, sizeof(Robot_Pos));
       Robot_Pos.X = 2;
       Robot_Pos.Y = 2;
       memset(&Robot_Vel, 0, sizeof(Robot_Vel));
   }*/
}

/**
 * @brief 发布IMU传感器数据到ROS2话题
 */
void origincar_base::Publish_ImuSensor()
{
  sensor_msgs::msg::Imu Imu_Data_Pub;
  Imu_Data_Pub.header.stamp = rclcpp::Node::now();
  Imu_Data_Pub.header.frame_id = gyro_frame_id; // 使用配置的IMU坐标系

  Imu_Data_Pub.orientation.x = Mpu6050.orientation.x;
  Imu_Data_Pub.orientation.y = Mpu6050.orientation.y;
  Imu_Data_Pub.orientation.z = Mpu6050.orientation.z;
  Imu_Data_Pub.orientation.w = Mpu6050.orientation.w;
  Imu_Data_Pub.orientation_covariance[0] = 1e6; // 方向协方差设置
  Imu_Data_Pub.orientation_covariance[4] = 1e6;
  Imu_Data_Pub.orientation_covariance[8] = 1e-6;
  Imu_Data_Pub.angular_velocity.x = Mpu6050.angular_velocity.x;
  Imu_Data_Pub.angular_velocity.y = Mpu6050.angular_velocity.y;
  Imu_Data_Pub.angular_velocity.z = Mpu6050.angular_velocity.z;
  Imu_Data_Pub.angular_velocity_covariance[0] = 1e6; // 角速度协方差设置
  Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;
  Imu_Data_Pub.angular_velocity_covariance[8] = 1e-6;
  Imu_Data_Pub.linear_acceleration.x = Mpu6050.linear_acceleration.x;
  Imu_Data_Pub.linear_acceleration.y = Mpu6050.linear_acceleration.y;
  Imu_Data_Pub.linear_acceleration.z = Mpu6050.linear_acceleration.z;

  imu_publisher->publish(Imu_Data_Pub); // 发布IMU数据
}

/**
 * @brief 发布里程计（其中已经包含了位置和速度还有四元数信息）、自定义位置消息、自定义速度消息数据到ROS2话题
 */
void origincar_base::Publish_Odom()
{
  tf2::Quaternion q;
  q.setRPY(0, 0, Robot_Pos.Z); // 从欧拉角转换为四元数
  geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

  origincar_msg::msg::Data robotpose;
  origincar_msg::msg::Data robotvel;
  nav_msgs::msg::Odometry odom;

  odom.header.stamp = rclcpp::Node::now();
  odom.header.frame_id = odom_frame_id;
  odom.child_frame_id = robot_frame_id;

  // 设置机器人位置
  odom.pose.pose.position.x = Robot_Pos.X;
  odom.pose.pose.position.y = Robot_Pos.Y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  // 设置机器人速度
  odom.twist.twist.linear.x = Robot_Vel.X;
  odom.twist.twist.linear.y = Robot_Vel.Y;
  odom.twist.twist.angular.z = Robot_Vel.Z;

  // 设置自定义位置消息
  robotpose.x = Robot_Pos.X;
  robotpose.y = Robot_Pos.Y;
  robotpose.z = Robot_Pos.Z;

  // 设置自定义速度消息
  robotvel.x = Robot_Vel.X;
  robotvel.y = Robot_Vel.Y;
  robotvel.z = Robot_Vel.Z;

  // 发布各类消息
  odom_publisher->publish(odom);           // 发布标准里程计消息
  robotpose_publisher->publish(robotpose); // 发布自定义位置消息
  robotvel_publisher->publish(robotvel);   // 发布自定义速度消息
}

/**
 * @brief 发布电池电压数据到ROS2话题
 */
void origincar_base::Publish_Voltage()
{
  std_msgs::msg::Float32 voltage_msgs;
  static float Count_Voltage_Pub = 0;

  if (Count_Voltage_Pub++ > 10)
  { // 每10次循环发布一次电压数据
    Count_Voltage_Pub = 0;
    voltage_msgs.data = Power_voltage;
    voltage_publisher->publish(voltage_msgs);
  }
}

/**
 * @brief 计算数据校验和，用于通信数据的完整性验证
 * @param Count_Number 计算校验和的数据字节数
 * @param mode 模式选择(0:接收数据校验, 1:发送数据校验)
 * @return 计算得到的校验和
 */
unsigned char origincar_base::Check_Sum(unsigned char Count_Number, unsigned char mode)
{
  unsigned char check_sum = 0, k;

  if (mode == 0)
  { // 接收数据校验模式
    for (k = 0; k < Count_Number; k++)
    {
      check_sum = check_sum ^ Receive_Data.rx[k]; // 异或校验
    }
  }
  else if (mode == 1)
  { // 发送数据校验模式
    for (k = 0; k < Count_Number; k++)
    {
      check_sum = check_sum ^ Send_Data.tx[k]; // 异或校验
    }
  }

  return check_sum;
}

/**
 * @brief 从下位机获取传感器数据
 * @return 数据获取是否成功
 */
bool origincar_base::Get_Sensor_Data()
{
  short transition_16 = 0, j = 0, Header_Pos = 0, Tail_Pos = 0;
  uint8_t Receive_Data_Pr[RECEIVE_DATA_SIZE] = {0};            // 临时接收缓冲区
  Stm32_Serial.read(Receive_Data_Pr, sizeof(Receive_Data_Pr)); // 从串口读取数据

  // 查找帧头和帧尾位置
  for (j = 0; j < 24; j++)
  {
    if (Receive_Data_Pr[j] == FRAME_HEADER)
      Header_Pos = j;
    else if (Receive_Data_Pr[j] == FRAME_TAIL)
      Tail_Pos = j;
  }

  // 数据帧完整性检查和重新排列
  if (Tail_Pos == (Header_Pos + 23))
  { // 正常帧，直接复制
    memcpy(Receive_Data.rx, Receive_Data_Pr, sizeof(Receive_Data_Pr));
  }
  else if (Header_Pos == (1 + Tail_Pos))
  { // 帧错位，需要重排
    for (j = 0; j < 24; j++)
      Receive_Data.rx[j] = Receive_Data_Pr[(j + Header_Pos) % 24];
  }
  else
  {
    return false; // 数据帧不完整，放弃处理
  }

  // 提取帧头帧尾
  Receive_Data.Frame_Header = Receive_Data.rx[0];
  Receive_Data.Frame_Tail = Receive_Data.rx[23];

  // 验证帧头帧尾和校验和
  if (Receive_Data.Frame_Header == FRAME_HEADER)
  {
    if (Receive_Data.Frame_Tail == FRAME_TAIL)
    {
      if (Receive_Data.rx[22] == Check_Sum(22, READ_DATA_CHECK) || (Header_Pos == (1 + Tail_Pos)))
      {
        // 数据验证通过，开始解析数据
        Receive_Data.Flag_Stop = Receive_Data.rx[1]; // 停止标志位

        // 解析机器人速度数据
        Robot_Vel.X = Odom_Trans(Receive_Data.rx[2], Receive_Data.rx[3]); // X方向线速度
        Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4], Receive_Data.rx[5]); // Y方向线速度
        Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6], Receive_Data.rx[7]); // Z方向角速度

        // 解析IMU原始数据
        Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8], Receive_Data.rx[9]);   // X方向加速度
        Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10], Receive_Data.rx[11]); // Y方向加速度
        Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12], Receive_Data.rx[13]); // Z方向加速度
        Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[14], Receive_Data.rx[15]);  // X方向角速度
        Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16], Receive_Data.rx[17]);  // Y方向角速度
        Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18], Receive_Data.rx[19]);  // Z方向角速度

        // 将IMU原始数据转换为标准单位
        Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data / ACCEl_RATIO; // 转换为m/s^2
        Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data / ACCEl_RATIO;
        Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data / ACCEl_RATIO;

        Mpu6050.angular_velocity.x = Mpu6050_Data.gyros_x_data * GYROSCOPE_RATIO; // 转换为rad/s
        Mpu6050.angular_velocity.y = Mpu6050_Data.gyros_y_data * GYROSCOPE_RATIO;
        Mpu6050.angular_velocity.z = Mpu6050_Data.gyros_z_data * GYROSCOPE_RATIO;

        // 解析电池电压数据
        transition_16 = 0;
        transition_16 |= Receive_Data.rx[20] << 8;
        transition_16 |= Receive_Data.rx[21];
        Power_voltage = transition_16 / 1000 + (transition_16 % 1000) * 0.001; // 转换为伏特

        return true; // 数据解析成功
      }
    }
  }

  return false; // 数据验证失败
}

/**
 * @brief 主控制循环，处理数据收发和位置计算
 */
void origincar_base::Control()
{
  rclcpp::Time current_time, last_time;
  current_time = rclcpp::Node::now();
  last_time = rclcpp::Node::now();
  while (rclcpp::ok())
  {
    current_time = rclcpp::Node::now();
    Sampling_Time = (current_time - last_time).seconds(); // 计算采样周期
    if (true == Get_Sensor_Data())
    { // 成功获取传感器数据
      // 根据速度更新机器人位置 - 运动学模型
      Robot_Pos.X += 1.03 * (Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time;
      Robot_Pos.Y += 1.125 * (Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time;
      Robot_Pos.Z += Robot_Vel.Z * Sampling_Time;

      // 计算IMU四元数
      Quaternion_Solution(Mpu6050.angular_velocity.x, Mpu6050.angular_velocity.y, Mpu6050.angular_velocity.z,
                          Mpu6050.linear_acceleration.x, Mpu6050.linear_acceleration.y, Mpu6050.linear_acceleration.z);
      // 发布各类数据
      Publish_ImuSensor();                                // 发布IMU数据
      Publish_Voltage();                                  // 发布电压数据
      Publish_Odom();                                     // 发布里程计数据
      rclcpp::spin_some(this->get_node_base_interface()); // 处理回调
    }
    last_time = current_time;
  }
}

/**
 * @brief 构造函数，初始化ROS2节点和各种参数
 */
origincar_base::origincar_base()
    : rclcpp::Node("origincar_base")
{
  // 初始化各种数据结构
  memset(&Robot_Pos, 0, sizeof(Robot_Pos));
  memset(&Robot_Vel, 0, sizeof(Robot_Vel));
  memset(&Receive_Data, 0, sizeof(Receive_Data));
  memset(&Send_Data, 0, sizeof(Send_Data));
  memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));

  int serial_baud_rate = 115200; // 默认串口波特率

  // 声明ROS2参数
  this->declare_parameter<std::string>("usart_port_name", "/dev/ttyACM0"); // 串口设备名
  this->declare_parameter<std::string>("cmd_vel", "cmd_vel");              // 速度控制话题
  this->declare_parameter<std::string>("akm_cmd_vel", "ackermann_cmd");    // 阿克曼控制话题
  this->declare_parameter<std::string>("odom_frame_id", "odom");           // 里程计坐标系ID
  this->declare_parameter<std::string>("robot_frame_id", "base_link");     // 机器人坐标系ID
  this->declare_parameter<std::string>("gyro_frame_id", "gyro_link");      // 陀螺仪坐标系ID

  // 获取ROS2参数
  this->get_parameter("serial_baud_rate", serial_baud_rate);
  this->get_parameter("usart_port_name", usart_port_name);
  this->get_parameter("cmd_vel", cmd_vel);
  this->get_parameter("akm_cmd_vel", akm_cmd_vel);
  this->get_parameter("odom_frame_id", odom_frame_id);
  this->get_parameter("robot_frame_id", robot_frame_id);
  this->get_parameter("gyro_frame_id", gyro_frame_id);

  // 创建各种发布者
  odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 10);            // 里程计发布者
  imu_publisher = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);       // IMU数据发布者
  voltage_publisher = create_publisher<std_msgs::msg::Float32>("PowerVoltage", 1);   // 电压发布者
  robotpose_publisher = create_publisher<origincar_msg::msg::Data>("robotpose", 10); // 机器人位置发布者
  robotvel_publisher = create_publisher<origincar_msg::msg::Data>("robotvel", 10);   // 机器人速度发布者

  tf_bro = std::make_shared<tf2_ros::TransformBroadcaster>(this); // TF广播器

  // 创建订阅者
  Cmd_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel, 1, std::bind(&origincar_base::Cmd_Vel_Callback, this, _1)); // 速度控制订阅者
  Akm_Cmd_Vel_Sub = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      akm_cmd_vel, 1, std::bind(&origincar_base::Akm_Cmd_Vel_Callback, this, _1)); // 阿克曼控制订阅者

  // 初始化串口通信
  try
  {
    Stm32_Serial.setPort("/dev/ttyACM0");                         // 设置串口设备
    Stm32_Serial.setBaudrate(serial_baud_rate);                   // 设置波特率
    serial::Timeout _time = serial::Timeout::simpleTimeout(2000); // 设置超时时间为2秒
    Stm32_Serial.setTimeout(_time);
    Stm32_Serial.open(); // 打开串口
  }
  catch (serial::IOException &e)
  {
    RCLCPP_ERROR(this->get_logger(), "origincar_base can not open serial port,Please check the serial port cable! ");
  }
  if (Stm32_Serial.isOpen())
  {
    RCLCPP_INFO(this->get_logger(), "origincar_base serial port opened");
  }
}

/**
 * @brief 信号处理函数，用于优雅地关闭机器人
 * @param sig 信号值
 */
void sigintHandler(int sig)
{
  sig = sig;
  printf("OriginBot shutdown...\n");
  // 初始化一个新的串口连接
  serial::Serial Stm32_Serial;
  Stm32_Serial.setPort("/dev/ttyACM0");
  Stm32_Serial.setBaudrate(115200);
  serial::Timeout _time = serial::Timeout::simpleTimeout(2000);
  Stm32_Serial.setTimeout(_time);
  Stm32_Serial.open();
  SEND_DATA Send_Data;
  if (Stm32_Serial.isOpen())
  {
    // 发送停止命令给下位机 - 将所有速度值置为0
    Send_Data.tx[0] = FRAME_HEADER; // 帧头
    Send_Data.tx[1] = 0;
    Send_Data.tx[2] = 0;

    Send_Data.tx[4] = 0; // X速度低字节
    Send_Data.tx[3] = 0; // X速度高字节

    Send_Data.tx[6] = 0; // Y速度低字节
    Send_Data.tx[5] = 0; // Y速度高字节

    Send_Data.tx[7] = 0; // 角速度高字节
    Send_Data.tx[8] = 0; // 角速度低字节

    // 计算校验和
    int check_sum = 0;
    for (int k = 0; k < 9; k++)
    {
      check_sum = check_sum ^ Send_Data.tx[k];
    }
    Send_Data.tx[9] = check_sum;
    Send_Data.tx[10] = FRAME_TAIL; // 帧尾

    try
    {
      Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx)); // 发送停止命令
    }
    catch (serial::IOException &e)
    {
    }
  }
  // 关闭ROS2接口，清除资源
  rclcpp::shutdown();
}

/**
 * @brief 析构函数，关闭节点
 */
origincar_base::~origincar_base()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down");
}