# Livelybot_Control_Broad SDK 说明文档

## 功能：
提供一个类接口，实现和主控板进行通讯
## Motor_Status 
介绍：电机状态类，用于保存各个电机的状态
#### 公共参数

##### motor_fb1[]， motor_fb2[]
分别用于保存can1线和can2线上各个电机的状态，类型是motor_fb_s结构体类型
``` C++
typedef struct
{
    uint8_t motor_id;   // 电机id
    int32_t motor_cmd;  // 电机当前所处的模式
    int32_t position;   // 电机当前位置  (双编)单位：0.001圈      (单编)：0.001 / 20圈
    int32_t velocity;   // 电机速度      (双编)单位：0.001圈/s    (单编)：0.001 / 20圈/s
    int32_t torque;     // 电机扭矩      (双编)单位：0.001N/m
}motor_fb_space_s;

typedef struct 
{
    union 
    {
        motor_fb_space_s motor;
        uint8_t data[17];
    };
}motor_fb_s;
```
##### imudata（基础功能款没有包含imu传感器）
主控板上的IMU模块参数， 类型是imu_s结构体类型

``` C++
typedef struct
{
    //加速度
    int16_t accX;    
    int16_t accY;
    int16_t accZ;
    //角速度
    int16_t angVelX;
    int16_t angVelY;
    int16_t angVelZ;
    //姿态角
    int16_t angle_roll;
    int16_t angle_pitch;
    int16_t angle_yaw;
    //磁场强度
    int16_t magX;
    int16_t magY;
    int16_t magZ;
}imu_space_s;

typedef struct
{
    union
    {
        imu_space_s imu_data;
        uint8_t data[24];
    };
}imu_s;
```
##### foot_sensor1[3],foot_sensor2[3] （存在于双足机器人版本，基础功能款里没有这部分数据）
分别返回双足机器人两只脚的足底传感器数值，每只脚有三个传感器

## Livelybot_Driver 
介绍：给用户提供的跟主控板进行通讯的类

#### 构造函数 Livelybot_Driver(string spi_dev)
``` C++
string spi_dev,            //SPI设备
```

#### motor_fb_space_s get_motor_state(int8_t motor_id)

功能：用于读取电机的状态 \
入参：\
motor_id:   电机所在的Can线和电机的id \
返回值：motor_fb_space_s类型

#### imu_space_s get_imu_data(void)

功能：用于读取imu的数值 \
返回值：imu_space_s类型

#### uint8_t* get_footsensor_data(uint8_t switch_can)

功能：用于读取足底传感器的数值 \
入参：\
switch_can:   那只脚的can线 \
返回值：uint8_t的长度为3的数组，为足底传感器的值

#### bool spi_send(void)

功能：spi发送函数 \
返回值：bool值，true为通讯成功， false为通讯失败

#### void set_motor_position(int8_t motor_id, int32_t position)

功能：设置电机位置 （1圈为 0 ~ 100000）单编需要除以20 \
入参：\
motor_id：  电机所在的Can线和电机的id \
position：  位置 \
返回值：无

#### void set_motor_position(int8_t motor_id, int32_t position, int32_t velocity,int32_t torque, float kp, float kd)

功能：设置电机带速度kpkd的位置 （1圈为 0 ~ 100000）单编需要除以20 \
入参：\
motor_id：  电机所在的Can线和电机的id \
position：  位置 \
velocity：  速度 \
torque：    扭矩 \
kp：        电机的kp \
kd：        电机的kd \
返回值：无

#### void set_motor_velocity(int8_t motor_id, int32_t velocity)

功能：设置电机转速 （100000为1圈每秒）单编需要除以20 \
入参：\
motor_id：  电机所在的Can线和电机的id \
velocity：  速度 \
返回值：无


#### void set_motor_torque(int8_t motor_id, int32_t torque)

功能：电机设置电机扭矩 （0.01 N-M ）单编需要除以20 \
入参：\
motor_id：  电机所在的Can线和电机的id \
torque：    扭矩 \
返回值：无

#### int32_t transfer_send(tranfer_send_type_e type, float data)

功能：发送转换函数 \
入参：\
type：  转换类型 \
data：  转换数据 \
返回值：0xffffaaaa为转换失败，别的值为正常值

#### float transfer_rec(tranfer_send_type_e type, int32_t data)

功能：接收转换函数 \
入参：\
type：  转换类型 \
data：  转换数据 \
返回值：0xffffaaaa为转换失败，别的值为正常值

