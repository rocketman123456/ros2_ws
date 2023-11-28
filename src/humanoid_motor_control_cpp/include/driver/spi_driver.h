#pragma once
#include "motor_control/motor_data.h"

#include <chrono>
#include <string>

namespace pi
{

    // 定义一个简单的类
    class SpiDriver
    {
    public:
        // 构造函数
        SpiDriver(const std::string& spi_dev);
        ~SpiDriver() = default;

        void initialize();
        void finalize();

        bool open_spi();
        bool send_spi();
        void close_spi();

        const motor_t& get_motor_state(int8_t motor_id) const;
        const imu_t&   get_imu_data() const;
        const uint8_t* get_footsensor_data(uint8_t switch_can) const;

        // void set_can1_motor_pos(int8_t motor_id, int32_t position);
        // void set_can1_motor_pos(int8_t motor_id, int32_t position, int32_t velocity, int32_t torque, float kp, float kd);
        // void set_can1_motor_vel(int8_t motor_id, int32_t velocity);
        // void set_can1_motor_torque(int8_t motor_id, int32_t torque);

        // void set_can2_motor_pos(int8_t motor_id, int32_t position);
        // void set_can2_motor_pos(int8_t motor_id, int32_t position, int32_t velocity, int32_t torque, float kp, float kd);
        // void set_can2_motor_vel(int8_t motor_id, int32_t velocity);
        // void set_can2_motor_torque(int8_t motor_id, int32_t torque);

        void set_motor_position(int8_t motor_id, int32_t position);
        void set_motor_position(int8_t motor_id, int32_t position, int32_t velocity, int32_t torque, float kp, float kd);
        void set_motor_velocity(int8_t motor_id, int32_t velocity);
        void set_motor_torque(int8_t motor_id, int32_t torque);

    private:
        void set_robot_param(int8_t motor_type, int8_t can1_motor_num, int8_t can2_motor_num, uint8_t isenable_imu, uint8_t isenable_footsensor);
        void motor_set(uint8_t motor_id, int32_t cmd, int32_t posorvolt, int32_t vel, int32_t torque, float kp, float kd);
        bool parse_datas(uint8_t* rx_buf);
        void clear_tx_buffer();
        void print_buffer_data(const char* name, uint8_t* data, size_t size);

        bool m_print_debug_info  = false;
        bool m_print_buffer_data = false;

        std::string m_spi_dev;

        int8_t m_motor_type;
        int8_t m_can1_motor_num;
        int8_t m_can2_motor_num;

        int8_t m_enable_imu;
        int8_t m_enable_footsensor;

        motor_status_t m_all_motor_status;

        int32_t  m_spi_fd;
        uint32_t m_speed;
        uint16_t m_delay;
        uint8_t  m_bits_per_word;
        uint8_t  m_mode;

        uint8_t m_spi_tx_databuffer[k_data_pkg_size] = {0};
        uint8_t m_spi_rx_databuffer[k_data_pkg_size] = {0};
        uint8_t m_spi_tx_motor_num                   = 0;

        bool m_spi_stop_flag;

        std::chrono::high_resolution_clock::time_point t1, t2;
    };
} // namespace pi
