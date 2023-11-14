#include "Livelybot_Driver.h"
#include <time.h>

//例子
void delay_ms(int milliseconds)
{
    struct timespec ts;
    ts.tv_sec  = milliseconds / 1000;
    ts.tv_nsec = ((milliseconds) % 1000) * 1000000;
    nanosleep(&ts, NULL);
}

#define ALL_MOTOR_NUM (CAN1_NUM + CAN2_NUM)

#define MYTIMES 100000
int main()
{
    int32_t          add_pos = 0;
    imu_space_s      imu_data;
    motor_fb_space_s motor_state;
    bool             spi_flag = false;

    //创建Liveltbot_Driver对象,传入参数
    Livelybot_Driver my_Driver("/dev/spidev4.1");

    int32_t init_pos[ALL_MOTOR_NUM] = {0};

    my_Driver.spi_send();

    for (uint8_t i = 0; i < CAN1_NUM; i++)
    {
        uint8_t temp_id = 0x10 | (i + 1);
        init_pos[i]     = my_Driver.get_motor_state(temp_id).position;
        printf("m0x%02x: %d\t", temp_id, init_pos[i]);
        my_Driver.set_motor_position(temp_id, init_pos[i] + add_pos);
    }

    for (uint8_t i = 0; i < CAN2_NUM; i++)
    {
        uint8_t temp_id        = 0x20 | (i + 1);
        init_pos[i + CAN1_NUM] = my_Driver.get_motor_state(temp_id).position;
        printf("m0x%02x: %d\t", temp_id, init_pos[i + CAN1_NUM]);
        my_Driver.set_motor_position(temp_id, init_pos[i + CAN1_NUM] + add_pos);
    }
    printf("\n");

    delay_ms(2000);
    spi_flag = my_Driver.spi_send();

    int32_t target_pos = my_Driver.transfer_send(ANG2POS, 10);

    for (int times = 0; times < 10; times++)
    {

        while (add_pos < target_pos)
        {
            // delay_ms(1);
            // 设置can1线id为1的电机的位置
            for (uint8_t i = 0; i < CAN1_NUM; i++)
            {
                uint8_t temp_id = 0x10 | (i + 1);
                printf("motor0x%02x pos:%d\n", temp_id, my_Driver.get_motor_state(temp_id).position);
                my_Driver.set_motor_position(temp_id, init_pos[i] + add_pos);
            }

            for (uint8_t i = 0; i < CAN2_NUM; i++)
            {
                uint8_t temp_id = 0x20 | (i + 1);
                printf("motor0x%02x pos:%d\n", temp_id, my_Driver.get_motor_state(temp_id).position);
                my_Driver.set_motor_position(temp_id, init_pos[i + CAN1_NUM] + add_pos);
            }

            //发送指令
            spi_flag = my_Driver.spi_send();
            if (spi_flag)
                add_pos += 5;
        }

        while (add_pos > 0)
        {
            // delay_ms(1);
            // 设置can1线id为1的电机的位置
            for (uint8_t i = 0; i < CAN1_NUM; i++)
            {
                uint8_t temp_id = 0x10 | (i + 1);
                printf("motor0x%02x pos:%d\n", temp_id, my_Driver.get_motor_state(temp_id).position);
                my_Driver.set_motor_position(temp_id, init_pos[i] + add_pos);
            }

            for (uint8_t i = 0; i < CAN2_NUM; i++)
            {
                uint8_t temp_id = 0x20 | (i + 1);
                printf("motor0x%02x pos:%d\n", temp_id, my_Driver.get_motor_state(temp_id).position);
                my_Driver.set_motor_position(temp_id, init_pos[i + CAN1_NUM] + add_pos);
            }

            //发送指令
            spi_flag = my_Driver.spi_send();
            if (spi_flag)
                add_pos -= 5;
        }
    }

    return 0;
}