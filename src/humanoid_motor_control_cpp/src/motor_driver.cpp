#include "humanoid_motor_control_cpp/motor_driver.h"

// using namespace std;

namespace pi
{
    // 构造函数
    /*
    spi_dev：SPI设备
    */
    MotorDriver::MotorDriver(const std::string spi_dev) : m_spi_dev(spi_dev)
    {
        printf("init_class spi_dev: %s\n", spi_dev.c_str());
        printf("motor_type:%u, can1_motor_num:%u, can2_motor_num:%u\n", MOTOR_TYPE, CAN1_NUM, CAN2_NUM);

        m_spi_stop_flag        = false;
        m_spi_tx_motor_num     = 0;
        m_spi_tx_databuffer[0] = 0xA5;
        m_spi_tx_databuffer[1] = 0x5A;
        set_robot_param(MOTOR_TYPE, CAN1_NUM, CAN2_NUM, ENABLE_IMU, ENABLE_FOOTSENSOR);
        t1 = std::chrono::high_resolution_clock::now();
        usleep(1000);
        if (!open_spi())
        {
            printf("open spi fail!!!!!!!!!\n");
            exit(0);
        }
    }

    motor_t MotorDriver::get_motor_state(int8_t motor_id)
    {
        int8_t switch_can = motor_id & 0xf0;
        int8_t id         = motor_id & 0x0f;
        if (switch_can == 0x10)
        {
            return all_motor_status.motor_fb1[id - 1].motor;
        }
        else if (switch_can == 0x20)
        {
            return all_motor_status.motor_fb2[id - 1].motor;
        }
        else
        {
            return *(motor_t*)nullptr;
        }
    }

    imu_t MotorDriver::get_imu_data() { return all_motor_status.imu_data.imu_data; }

    uint8_t* MotorDriver::get_footsensor_data(uint8_t switch_can)
    {
        uint8_t zero_sensor[3] = {0, 0, 0};
        if (switch_can == FOOTSENSOR1)
        {
            return all_motor_status.foot_sensor1;
        }
        else
        {
            return all_motor_status.foot_sensor2;
        }
    }

    bool MotorDriver::open_spi(void)
    {
        int ret;
        //使用SPI接口
        m_spi_fd = open(m_spi_dev.data(), O_RDWR);
        if (m_spi_fd < 0)
        {
            perror("Error: Cann't open SPI Dev.\n");
            return false;
        }

        //配置SPI参数
        m_speed         = SPI_SPEED;
        m_delay         = 0;
        m_bits_per_word = 8;
        m_mode          = 0;

        ret = ioctl(m_spi_fd, SPI_IOC_WR_MODE, &m_mode);
        if (ret == -1)
        {
            perror("Error: SPI_IOC_WR_MODE fault.\n");
            return false;
        }

        ret = ioctl(m_spi_fd, SPI_IOC_WR_BITS_PER_WORD, &m_bits_per_word);
        if (ret == -1)
        {
            perror("Error: SPI_IOC_WR_BITS fault.\n");
            return false;
        }

        ret = ioctl(m_spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &m_speed);
        if (ret == -1)
        {
            perror("Error: SPI_IOC_WR_MAX_SPEED fault.\n");
            return false;
        }
        return true;
    }

    void MotorDriver::set_robot_param(int8_t motor_type, int8_t can1_motor_num, int8_t can2_motor_num, uint8_t isenable_imu, uint8_t isenable_footsensor)
    {
        m_motor_type          = motor_type;
        m_can1_motor_num      = can1_motor_num;
        m_can2_motor_num      = can2_motor_num;
        m_isenable_imu        = isenable_imu;
        m_isenable_footsensor = isenable_footsensor;

#ifdef DEBUG
        printf("spi_dev:%\n", m_spi_dev.c_str());
        printf("motor_type:%u, can1_motor_num:%u, can2_motor_num:%u, enable_imu:%u, footsensor:%u\n",
               motor_type,
               can1_motor_num,
               can2_motor_num,
               isenable_imu,
               isenable_footsensor);
#endif // DEBUG

        int ret;

        //使用SPI1接口
        m_spi_fd = open(m_spi_dev.c_str(), O_RDWR);
        if (m_spi_fd < 0)
        {
            perror("Error: Cann't open SPI Dev.\n");
            return;
        }

        //配置SPI参数
        m_speed         = SPI_SPEED;
        m_delay         = 0;
        m_bits_per_word = 8;
        m_mode          = 0;

        ret = ioctl(m_spi_fd, SPI_IOC_WR_MODE, &m_mode);
        if (ret == -1)
        {
            perror("Error: SPI_IOC_WR_MODE fault.\n");
            m_spi_tx_motor_num = 0;
            return;
        }

        ret = ioctl(m_spi_fd, SPI_IOC_WR_BITS_PER_WORD, &m_bits_per_word);
        if (ret == -1)
        {
            perror("Error: SPI_IOC_WR_BITS fault.\n");
            m_spi_tx_motor_num = 0;
            return;
        }

        ret = ioctl(m_spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &m_speed);
        if (ret == -1)
        {
            perror("Error: SPI_IOC_WR_MAX_SPEED fault.\n");
            m_spi_tx_motor_num = 0;
            return;
        }

        //发送数据
        m_spi_tx_databuffer[2] = 0x20;
        m_spi_tx_databuffer[3] = m_motor_type;
        m_spi_tx_databuffer[4] = m_can1_motor_num;
        m_spi_tx_databuffer[5] = m_can2_motor_num;
        m_spi_tx_databuffer[6] = m_isenable_imu;
        m_spi_tx_databuffer[7] = m_isenable_footsensor;
        m_spi_tx_databuffer[8] = ENABLE_STOP;

#ifdef TX_DEBUG
        printf("tx_data: ");
        for (uint16_t i = 0; i < DATA_PKG_SIZE; i++)
        {
            printf("0x%02x,", m_spi_tx_databuffer[i]);
        }
        printf("\n");
#endif //

        uint8_t rx_buf[DATA_PKG_SIZE] = {};

        struct spi_ioc_transfer spi = {};

        spi.tx_buf        = (unsigned long)m_spi_tx_databuffer;
        spi.rx_buf        = (unsigned long)rx_buf;
        spi.len           = sizeof(m_spi_tx_databuffer);
        spi.delay_usecs   = m_delay;
        spi.speed_hz      = m_speed;
        spi.bits_per_word = m_bits_per_word;
        // Send wr_addr
        ret = ioctl(m_spi_fd, SPI_IOC_MESSAGE(1), &spi);
        if (ret < 1)
        {
            perror("Error: SPI_IOC_MESSAGE fault.\n");
            m_spi_tx_motor_num = 0;
            return;
        }

#ifdef DEBUG
        printf("transmit suc!\n");
#endif // DEBUG
#ifdef RX_DEBUG
        printf("recv data: ");
        for (uint16_t i = 0; i < DATA_PKG_SIZE; i++)
        {
            printf("0x%02x,", rx_buf[i]);
        }
        printf("\n");
#endif // DEBUG

        parse_datas(rx_buf);
        close(m_spi_fd);
    }

    void MotorDriver::motor_set(uint8_t motor_id, int32_t cmd, int32_t posorvolt, int32_t vel, int32_t torque, float kp, float kd)
    {
        motor_set_data_t motor_state;

        motor_state.motor.motor_id  = motor_id;
        motor_state.motor.motor_cmd = cmd;
        motor_state.motor.position  = posorvolt;
        motor_state.motor.velocity  = vel;
        motor_state.motor.torque    = torque;
        motor_state.motor.kp        = kp;
        motor_state.motor.kd        = kd;

        if (m_spi_tx_motor_num >= (m_can1_motor_num + m_can2_motor_num))
        {
            printf("the motor num overflow!!!!!!\n");
            return;
        }
        memcpy(&m_spi_tx_databuffer[3 + m_spi_tx_motor_num * MOTOR_SET_LENGTH], motor_state.data, MOTOR_SET_LENGTH);
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

        // printf("pasre_data start!\n");

        uint8_t motor_nums = rx_buf[2];
        if (motor_nums != (m_can1_motor_num + m_can2_motor_num))
        {
            clear_tx_buffer();
            return false;
        }
        for (uint8_t i = 0; i < motor_nums; i++)
        {
            temp_can = rx_buf[3 + i * MOTOR_STATUS_LENGTH] & 0xf0;
            temp_id  = rx_buf[3 + i * MOTOR_STATUS_LENGTH] & 0x0f;

            if (temp_can == 0x10)
            {
                all_motor_status.motor_fb1[temp_id - 1].motor.motor_id = temp_id;
                memcpy(&all_motor_status.motor_fb1[temp_id - 1].data[1], &rx_buf[3 + i * MOTOR_STATUS_LENGTH + 1], MOTOR_STATUS_LENGTH - 1);
            }
            else if (temp_can == 0x20)
            {
                all_motor_status.motor_fb2[temp_id - 1].motor.motor_id = temp_id;
                memcpy(&all_motor_status.motor_fb2[temp_id - 1].data[1], &rx_buf[3 + i * MOTOR_STATUS_LENGTH + 1], MOTOR_STATUS_LENGTH - 1);
            }
        }
        uint16_t imu_index        = 3 + motor_nums * MOTOR_STATUS_LENGTH;
        uint16_t footsensor_index = 3 + motor_nums * MOTOR_STATUS_LENGTH;
        if (m_isenable_imu && rx_buf[imu_index] == 0xAA)
        {
            memcpy(all_motor_status.imu_data.data, &rx_buf[imu_index + 1], YJ901S_DATA_SIZE);
            footsensor_index = imu_index + YJ901S_DATA_SIZE;
        }
        if (m_isenable_footsensor && rx_buf[footsensor_index] == 0xBB)
        {
            memcpy(all_motor_status.foot_sensor1, &rx_buf[footsensor_index + 1], 3);
            memcpy(all_motor_status.foot_sensor2, &rx_buf[footsensor_index + 4], 3);
        }
        clear_tx_buffer();
        return true;
    }

    void MotorDriver::clear_tx_buffer(void)
    {
        for (uint16_t i = 0; i < DATA_PKG_SIZE; i++)
        {
            m_spi_tx_databuffer[i] = 0x00;
        }
        m_spi_tx_databuffer[0] = 0xA5;
        m_spi_tx_databuffer[1] = 0x5A;
        m_spi_tx_motor_num     = 0;
    }

    bool MotorDriver::spi_send(void)
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
        m_spi_tx_databuffer[2] = m_spi_tx_motor_num | 0x10;

#ifdef TX_DEBUG
        printf("tx_data: ");
        for (uint16_t i = 0; i < DATA_PKG_SIZE; i++)
        {
            printf("0x%02x,", spi_tx_databuffer[i]);
        }
        printf("\n");
#endif //

        uint8_t rx_buf[DATA_PKG_SIZE] = {};

        struct spi_ioc_transfer spi = {};

        spi.tx_buf        = (unsigned long)m_spi_tx_databuffer;
        spi.rx_buf        = (unsigned long)rx_buf;
        spi.len           = sizeof(m_spi_tx_databuffer);
        spi.delay_usecs   = m_delay;
        spi.speed_hz      = m_speed;
        spi.bits_per_word = m_bits_per_word;
        // Send wr_addr
        int ret = ioctl(m_spi_fd, SPI_IOC_MESSAGE(1), &spi);
        if (ret < 1)
        {
            perror("Error: SPI_IOC_MESSAGE fault.\n");
            return false;
        }

#ifdef DEBUG
        printf("transmit suc!\n");
#endif // DEBUG
#ifdef RX_DEBUG
        printf("recv data: ");
        for (uint16_t i = 0; i < DATA_PKG_SIZE; i++)
        {
            printf("0x%02x,", rx_buf[i]);
        }
        printf("\n");
#endif // DEBUG

        //解析数据
        if (!parse_datas(rx_buf))
        {
            m_spi_stop_flag = true;
            clear_tx_buffer();
            close(m_spi_fd);
            return false;
        }
        // 关闭spi
        // close(spi_fd);
        return true;
    }

    //位置模式
    /*
    motor_id:   电机所在的can和id
    position：  位置
    */
    void MotorDriver::set_motor_position(int8_t motor_id, int32_t position) { motor_set(motor_id, 5, position, 0, 0, 0, 0); }

    //位置PD模式
    /*
    motor_id:    电机所在的can和id
    position：   位置
    velocity:    速度
    kp：         kp值
    kd：         kd值
    */
    void MotorDriver::set_motor_position(int8_t motor_id, int32_t position, int32_t velocity, int32_t torque, float kp, float kd)
    {
        motor_set(motor_id, 7, position, velocity, torque, kp, kd);
    }

    //速度模式
    /*
    motor_id: 电机所在的can和id
    velocity：速度
    */
    void MotorDriver::set_motor_velocity(int8_t motor_id, int32_t velocity) { motor_set(motor_id, 6, velocity, 0, 0, 0, 0); }

    //力矩模式（单边）
    /*
    switch_can：选择can线
    motor_id: 电机id
    torque：力矩   单位：0.01N-M
    */
    void MotorDriver::set_motor_torque(int8_t motor_id, int32_t torque) { motor_set(motor_id, 2, torque, 0, 0, 0, 0); }

    int32_t MotorDriver::transfer_send(tranfer_send_type_e type, float data)
    {
        int32_t res = 0;
        switch (type)
        {
            case ANG2POS:
                res = data / 360.0 * 100000;
                return res;
            case RAD2POS:
                res = data / (2 * PI) * 100000;
                return res;
            case TOR2TOR:
                res = data * 100;
                return res;
            default:
                break;
        }
        return ERROR_TRANSFER_DATA;
    }

    float MotorDriver::transfer_rec(tranfer_send_type_e type, int32_t data)
    {
        float res = 0;
        switch (type)
        {
            case POS2ANG:
                res = (float)(data / 100000.0) * 360.0;
                return res;
            case POS2RAD:
                res = data / 100000 * (2 * PI);
                return res;
            default:
                break;
        }
        return ERROR_TRANSFER_DATA;
    }
} // namespace pi
