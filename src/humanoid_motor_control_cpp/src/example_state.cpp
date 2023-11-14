#include "Livelybot_Driver.h"
#include <time.h>

//例子
void delay_ms(int milliseconds) {
    struct timespec ts;
    ts.tv_sec = milliseconds / 1000;
    ts.tv_nsec = ((milliseconds) % 1000) * 1000000;
    nanosleep(&ts, NULL);
}


#define ALL_MOTOR_NUM (CAN1_NUM+CAN2_NUM)

#define MYTIMES 100000
int main()
{
    int32_t add_pos = 0;
    imu_space_s imu_data;
    motor_fb_space_s motor_state;
    bool spi_flag = false;

    //创建Liveltbot_Driver对象,传入参数
    Livelybot_Driver my_Driver("/dev/spidev4.1");
    
    int32_t init_pos[ALL_MOTOR_NUM] = {0};
    while(true)
    {
        //判断SPI收发是否已经完成，内部制定两次通讯时间间隔超过1ms才能进行下一次通讯
        if(!my_Driver.spi_send())
        {
            continue;
        }

        //获取电机状态
        for(uint8_t i = 0; i < CAN1_NUM; i++)
        {
            uint8_t temp_id = 0x10 | (i+1); //当电机为can1线时，则要获取的电机id为  0x10|电机真实的id 
            init_pos[i] =  my_Driver.get_motor_state(temp_id).position; //获取电机的当前的位置
            printf("mot0x%02x pos:%d\t", temp_id, init_pos[i]);
        }
        printf("\n");

        for(uint8_t i = 0; i < CAN2_NUM; i++)
        {
            uint8_t temp_id = 0x20 | (i+1); //当电机为can2线时，则要获取的电机id为  0x20|电机真实的id 
            init_pos[i + CAN1_NUM] =  my_Driver.get_motor_state(temp_id).position;
            printf("mot0x%02x pos:%d\t", temp_id, init_pos[i + CAN1_NUM]);
        }
        printf("\n");

        //获取通信板IMU值
        if(ENABLE_IMU)
        {
            imu_data = my_Driver.get_imu_data();
            printf("acc: %d, %d, %d; angVel: %d, %d, %d; angle %d, %d, %d; mag %d, %d, %d\n", imu_data.accX, imu_data.accY, imu_data.accZ, imu_data.angVelX, imu_data.angVelY, imu_data.angVelZ,
                imu_data.angle_roll, imu_data.angle_pitch, imu_data.angle_yaw, imu_data.magX, imu_data.magY, imu_data.magZ);
        }
        
        //获取足底传感器值
        if(ENABLE_FOOTSENSOR)
        {
            uint8_t *foot_sensor = my_Driver.get_footsensor_data(FOOTSENSOR1);
            printf("foot_sensor 1: %d, %d, %d;", foot_sensor[0], foot_sensor[1], foot_sensor[2]);
            foot_sensor = my_Driver.get_footsensor_data(FOOTSENSOR2);
            printf("foot_sensor 2: %d, %d, %d;\n", foot_sensor[0], foot_sensor[1], foot_sensor[2]);
        }

        delay_ms(1000);
    }

    return 0;
}
