#include "motor_control/motor_driver.h"
#include "motor_control/spi_utils.h"

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

namespace pi
{
    // 构造函数
    MotorDriver::MotorDriver(const std::string& spi_dev) : m_spi_dev(spi_dev) {}

    void MotorDriver::initialize()
    {
        printf("init_class spi_dev: %s\n", m_spi_dev.c_str());
        printf("motor_type:%u, can1_motor_num:%u, can2_motor_num:%u\n", k_motor_type, k_can1_num, k_can2_num);
        printf("k_motor_set_len: %d\n", k_motor_set_len);
        printf("k_motor_status_len: %d\n", k_motor_status_len);
        printf("k_yj901s_data_len: %d\n", k_yj901s_data_len);

        m_spi_stop_flag    = false;
        m_spi_tx_motor_num = 0;

        //配置SPI参数
        m_speed         = k_spi_speed;
        m_delay         = 0;
        m_bits_per_word = 8;
        m_mode          = 0;

        set_robot_param(k_motor_type, k_can1_num, k_can2_num, k_enable_imu, k_enable_foot_sensor);
        t1 = std::chrono::high_resolution_clock::now();

        usleep(1000);

        if (!open_spi())
        {
            printf("open spi fail!!!!!!!!!\n");
            exit(0);
        }
    }

    void MotorDriver::finalize() { close_spi(); }

    const motor_t& MotorDriver::get_motor_state(int8_t motor_id) const
    {
        int8_t switch_can = motor_id & 0xf0;
        int8_t id         = motor_id & 0x0f;
        if (switch_can == 0x10)
            return m_all_motor_status.motor_fb1[id - 1];
        else if (switch_can == 0x20)
            return m_all_motor_status.motor_fb2[id - 1];
        else
            return *(motor_t*)nullptr; // may cause bug
    }

    const imu_t& MotorDriver::get_imu_data() const { return m_all_motor_status.imu_data; }

    const uint8_t* MotorDriver::get_footsensor_data(uint8_t id) const
    {
        if (id == k_foot_sensor_id_1)
            return m_all_motor_status.foot_sensor1;
        else if (id == k_foot_sensor_id_2)
            return m_all_motor_status.foot_sensor2;
        return nullptr;
    }

    bool MotorDriver::open_spi(void)
    {
        m_spi_fd = spi_open(m_spi_dev, m_speed, m_delay, m_bits_per_word, m_mode);

        if (m_spi_fd < 0)
        {
            perror("Error: Cann't open SPI Dev.\n");
            return false;
        }

        return true;
    }

    bool MotorDriver::send_spi(void)
    {
        t2                = std::chrono::high_resolution_clock::now();
        uint64_t time_use = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
        if (m_spi_stop_flag)
        {
            if (time_use < 10000)
            {
                clear_tx_buffer();
                return false;
            }
            else
            {
                m_spi_stop_flag = false;
                open_spi();
                t1 = t2;
            }
        }
        else
        {
            if (time_use < 1000)
            {
                clear_tx_buffer();
                return false;
            }
            else
            {
                t1 = t2;
            }
        }

        //发送数据
        m_spi_tx_databuffer[2] = m_spi_tx_motor_num | 0x20;

        // clear rx buffer
        memset(m_spi_rx_databuffer, 0, k_data_pkg_size);
        int32_t ret = spi_send(m_spi_fd, m_spi_tx_databuffer, m_spi_rx_databuffer, k_data_pkg_size, m_speed, m_delay, m_bits_per_word);

        if (ret < 1)
        {
            perror("Error: SPI_IOC_MESSAGE fault.\n");
            return false;
        }

        if (m_print_debug_info)
        {
            print_buffer_data("send", m_spi_tx_databuffer, k_data_pkg_size);
            printf("transmit suc!\n");
            print_buffer_data("recv", m_spi_rx_databuffer, k_data_pkg_size);
        }

        //解析数据
        if (!parse_datas(m_spi_rx_databuffer))
        {
            m_spi_stop_flag = true;
            clear_tx_buffer();
            close_spi();
            return false;
        }
        // close_spi();
        return true;
    }

    void MotorDriver::close_spi() { spi_close(m_spi_fd); }

    // ---------------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------------

    void MotorDriver::print_buffer_data(const char* name, uint8_t* data, size_t size)
    {
        if (m_print_buffer_data)
        {
            printf("%s data: ", name);
            for (uint16_t i = 0; i < size; i++)
            {
                printf("0x%02x,", data[i]);
            }
            printf("\n");
        }
    }

    void MotorDriver::set_robot_param(int8_t motor_type, int8_t can1_motor_num, int8_t can2_motor_num, uint8_t isenable_imu, uint8_t isenable_footsensor)
    {
        if (m_print_debug_info)
        {
            printf("spi_dev:%s\n", m_spi_dev.c_str());
            printf("motor_type:%d\n", motor_type);
            printf("can1_motor_num:%d\n", can1_motor_num);
            printf("can2_motor_num:%d\n", can2_motor_num);
            printf("enable_imu:%d\n", isenable_imu);
            printf("footsensor:%d\n", isenable_footsensor);
        }

        m_motor_type        = motor_type;
        m_can1_motor_num    = can1_motor_num;
        m_can2_motor_num    = can2_motor_num;
        m_enable_imu        = isenable_imu;
        m_enable_footsensor = isenable_footsensor;

        m_spi_fd = spi_open(m_spi_dev, m_speed, m_delay, m_bits_per_word, m_mode);

        if (m_spi_fd < 0)
        {
            perror("Error: Cann't open SPI Dev.\n");
            return;
        }

        //发送数据
        m_spi_tx_databuffer[0] = 0xA5;
        m_spi_tx_databuffer[1] = 0x5A;
        m_spi_tx_databuffer[2] = 0x20;
        m_spi_tx_databuffer[3] = m_motor_type;
        m_spi_tx_databuffer[4] = m_can1_motor_num;
        m_spi_tx_databuffer[5] = m_can2_motor_num;
        m_spi_tx_databuffer[6] = m_enable_imu;
        m_spi_tx_databuffer[7] = m_enable_footsensor;
        m_spi_tx_databuffer[8] = k_enable_stop;

        memset(m_spi_rx_databuffer, 0, k_data_pkg_size);
        int32_t ret = spi_send(m_spi_fd, m_spi_tx_databuffer, m_spi_rx_databuffer, k_data_pkg_size, m_speed, m_delay, m_bits_per_word);

        if (ret < 1)
        {
            perror("Error: SPI_IOC_MESSAGE fault.\n");
            m_spi_tx_motor_num = 0;
            return;
        }

        if (m_print_debug_info)
        {
            print_buffer_data("send", m_spi_tx_databuffer, k_data_pkg_size);
            printf("transmit suc!\n");
            print_buffer_data("recv", m_spi_rx_databuffer, k_data_pkg_size);
        }

        parse_datas(m_spi_rx_databuffer);
        spi_close(m_spi_fd);
    }

    void MotorDriver::motor_set(uint8_t motor_id, int32_t cmd, int32_t posorvolt, int32_t vel, int32_t torque, float kp, float kd)
    {
        motor_set_t motor_state {};

        motor_state.motor_id  = motor_id;
        motor_state.motor_cmd = cmd;
        motor_state.position  = posorvolt;
        motor_state.velocity  = vel;
        motor_state.torque    = torque;
        motor_state.kp        = kp;
        motor_state.kd        = kd;

        if (m_spi_tx_motor_num >= (m_can1_motor_num + m_can2_motor_num))
        {
            printf("the motor num overflow!!!!!!\n");
            return;
        }
        memcpy(&m_spi_tx_databuffer[3 + m_spi_tx_motor_num * sizeof(motor_set_t)], &motor_state, sizeof(motor_set_t));
        m_spi_tx_motor_num++;
    }

    bool MotorDriver::parse_datas(uint8_t* rx_buf)
    {
        uint8_t temp_id  = 0x00;
        uint8_t temp_can = 0x00;
        //解析数据
        if (rx_buf[0] != 0xA6 || rx_buf[1] != 0x6A)
        {
            clear_tx_buffer();
            return false;
        }

        uint8_t motor_nums = rx_buf[2];
        if (motor_nums != (m_can1_motor_num + m_can2_motor_num))
        {
            clear_tx_buffer();
            return false;
        }

        for (uint8_t i = 0; i < motor_nums; i++)
        {
            temp_can = rx_buf[3 + i * k_motor_status_len] & 0xf0;
            temp_id  = rx_buf[3 + i * k_motor_status_len] & 0x0f;

            if (temp_can == 0x10)
            {
                memcpy(&m_all_motor_status.motor_fb1[temp_id - 1], &rx_buf[3 + i * k_motor_status_len], k_motor_status_len);
                m_all_motor_status.motor_fb1[temp_id - 1].motor_id = temp_id;
            }
            else if (temp_can == 0x20)
            {
                memcpy(&m_all_motor_status.motor_fb2[temp_id - 1], &rx_buf[3 + i * k_motor_status_len], k_motor_status_len);
                m_all_motor_status.motor_fb2[temp_id - 1].motor_id = temp_id;
            }
        }

        uint16_t imu_index        = 3 + motor_nums * k_motor_status_len;
        uint16_t footsensor_index = 3 + motor_nums * k_motor_status_len;

        if (m_enable_imu && rx_buf[imu_index] == 0xAA)
        {
            memcpy(&m_all_motor_status.imu_data, &rx_buf[imu_index + 1], k_yj901s_data_len);
            footsensor_index = imu_index + k_yj901s_data_len;
        }

        if (m_enable_footsensor && rx_buf[footsensor_index] == 0xBB)
        {
            memcpy(m_all_motor_status.foot_sensor1, &rx_buf[footsensor_index + 1], 3);
            memcpy(m_all_motor_status.foot_sensor2, &rx_buf[footsensor_index + 4], 3);
        }

        clear_tx_buffer();
        return true;
    }

    void MotorDriver::clear_tx_buffer(void)
    {
        memset(m_spi_tx_databuffer, 0, k_data_pkg_size);
        m_spi_tx_databuffer[0] = 0xA5;
        m_spi_tx_databuffer[1] = 0x5A;
        m_spi_tx_motor_num     = 0;
    }

    // ---------------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------------

    void MotorDriver::set_can1_motor_pos(int8_t motor_id, int32_t position)
    {
        uint8_t temp_id = 0x10 | (motor_id + 1);
        set_motor_position(temp_id, position);
    }

    void MotorDriver::set_can1_motor_pos(int8_t motor_id, int32_t position, int32_t velocity, int32_t torque, float kp, float kd)
    {
        uint8_t temp_id = 0x10 | (motor_id + 1);
        set_motor_position(temp_id, position, velocity, torque, kp, kd);
    }

    void MotorDriver::set_can1_motor_vel(int8_t motor_id, int32_t velocity)
    {
        uint8_t temp_id = 0x10 | (motor_id + 1);
        set_motor_velocity(temp_id, velocity);
    }

    void MotorDriver::set_can1_motor_torque(int8_t motor_id, int32_t torque)
    {
        uint8_t temp_id = 0x10 | (motor_id + 1);
        set_motor_torque(temp_id, torque);
    }

    void MotorDriver::set_can2_motor_pos(int8_t motor_id, int32_t position)
    {
        uint8_t temp_id = 0x20 | (motor_id + 1);
        set_motor_position(temp_id, position);
    }

    void MotorDriver::set_can2_motor_pos(int8_t motor_id, int32_t position, int32_t velocity, int32_t torque, float kp, float kd)
    {
        uint8_t temp_id = 0x20 | (motor_id + 1);
        set_motor_position(temp_id, position, velocity, torque, kp, kd);
    }

    void MotorDriver::set_can2_motor_vel(int8_t motor_id, int32_t velocity)
    {
        uint8_t temp_id = 0x20 | (motor_id + 1);
        set_motor_velocity(temp_id, velocity);
    }

    void MotorDriver::set_can2_motor_torque(int8_t motor_id, int32_t torque)
    {
        uint8_t temp_id = 0x20 | (motor_id + 1);
        set_motor_torque(temp_id, torque);
    }

    // ---------------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------------

    //位置模式
    void MotorDriver::set_motor_position(int8_t motor_id, int32_t position) { motor_set(motor_id, 5, position, 0, 0, 0, 0); }

    //位置PD模式
    void MotorDriver::set_motor_position(int8_t motor_id, int32_t position, int32_t velocity, int32_t torque, float kp, float kd)
    {
        motor_set(motor_id, 7, position, velocity, torque, kp, kd);
    }

    //速度模式
    void MotorDriver::set_motor_velocity(int8_t motor_id, int32_t velocity) { motor_set(motor_id, 6, velocity, 0, 0, 0, 0); }

    //力矩模式（单边）
    void MotorDriver::set_motor_torque(int8_t motor_id, int32_t torque) { motor_set(motor_id, 2, torque, 0, 0, 0, 0); }
} // namespace pi
