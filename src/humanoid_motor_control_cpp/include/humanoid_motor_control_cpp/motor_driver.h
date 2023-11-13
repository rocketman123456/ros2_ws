#pragma once
#include "humanoid_motor_control_cpp/motor_data.h"

#include <chrono>
#include <cstring>
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
//#include <string.h>

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

#define SPI_SPEED 6000000
#define PI 3.1415926

#define ERROR_TRANSFER_DATA 0xffffaaaa

namespace pi
{

    // 定义一个简单的类
    class MotorDriver
    {
    public:
        // 构造函数
        MotorDriver(const std::string spi_dev);
        ~MotorDriver() = default;

        motor_t get_motor_state(int8_t motor_id);
        imu_t   get_imu_data();

        uint8_t* get_footsensor_data(uint8_t switch_can);
        bool     open_spi();
        bool     spi_send();

        void set_motor_position(int8_t motor_id, int32_t position);
        void set_motor_position(int8_t motor_id, int32_t position, int32_t velocity, int32_t torque, float kp, float kd);
        void set_motor_velocity(int8_t motor_id, int32_t velocity);
        void set_motor_torque(int8_t motor_id, int32_t torque);

        int32_t transfer_send(tranfer_send_type_e type, float data);
        float   transfer_rec(tranfer_send_type_e type, int32_t data);

    private:
        void set_robot_param(int8_t motor_type, int8_t can1_motor_num, int8_t can2_motor_num, uint8_t isenable_imu, uint8_t isenable_footsensor);
        void motor_set(uint8_t motor_id, int32_t cmd, int32_t posorvolt, int32_t vel, int32_t torque, float kp, float kd);
        bool parse_datas(uint8_t* rx_buf);
        void clear_tx_buffer();

        std::string m_spi_dev;

        int8_t m_motor_type;
        int8_t m_can1_motor_num;
        int8_t m_can2_motor_num;

        int8_t m_isenable_imu;
        int8_t m_isenable_footsensor;

        motor_status_t all_motor_status;

        int32_t  m_spi_fd;
        uint32_t m_speed;
        uint16_t m_delay;
        uint8_t  m_bits_per_word;
        uint8_t  m_mode;

        uint8_t m_spi_tx_databuffer[DATA_PKG_SIZE] = {0};
        uint8_t m_spi_tx_motor_num                 = 0;

        bool m_spi_stop_flag;

        std::chrono::high_resolution_clock::time_point t1, t2;
    };
} // namespace pi
