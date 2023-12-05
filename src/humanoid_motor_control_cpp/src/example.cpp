#include "rclcpp/rclcpp.hpp"

#include "driver/spi_driver.h"
#include "humanoid_control/arm_control.h"
#include "humanoid_control/kinematic/ankle_ik.h"
#include "humanoid_control/leg_control.h"
#include "low_level_control/motor_control.h"

#include <eigen3/Eigen/Dense>

#include <string>
#include <time.h>
#include <vector>

//例子
void delay_ms(int milliseconds)
{
    struct timespec ts;
    ts.tv_sec  = milliseconds / 1000;
    ts.tv_nsec = ((milliseconds) % 1000) * 1000000;
    nanosleep(&ts, NULL);
}

#define ALL_MOTOR_NUM (CAN1_NUM + CAN2_NUM)

class HumanoidControl : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    HumanoidControl(std::string name) : Node(name), m_driver("/dev/spidev4.1")
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
        // 创建发布者
        // m_command_publisher = this->create_publisher<std_msgs::msg::String>("command", 10);

        // 创建定时器，2ms为周期，定时发布
        m_timer = this->create_wall_timer(std::chrono::milliseconds(2), std::bind(&HumanoidControl::timer_callback, this));

        m_driver.initialize();

        std::vector<pi::motor_init_info_t> create_infos;
    }

private:
    void timer_callback()
    {
        // 创建消息
        std::string message = "forward";
        // 日志打印
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.c_str());

        // if (_motor_name == "L_toe" || _motor_name == "R_toe")
        // {
        //     // little motor
        //     (abs(motor_fb_space.position / 2000000.0 * 360.0) > 1000.0 ? motorState->q = motorState->q : motorState->q = motor_fb_space.position / 2000000.0
        //     * 360.0); (abs(motor_fb_space.velocity / 2000000.0 * 360.0) > 1000.0 ? motorState->dq = motorState->dq : motorState->dq = motor_fb_space.velocity
        //     / 2000000.0 *(2 * PI)); (abs(motor_fb_space.torque / 100.0) > 10.0 ? motorState->tauEst = motorState->tauEst : motorState->tauEst =
        //     motor_fb_space.torque / 100.0);
        // }
        // else
        // {
        //     // big motor
        //     (abs(motor_fb_space.position / 100000.0 * 360.0) > 1000.0 ? motorState->q = motorState->q : motorState->q = motor_fb_space.position / 100000.0 *
        //     360.0); (abs(motor_fb_space.velocity / 100000.0 * 360.0) > 1000.0 ? motorState->dq = motorState->dq : motorState->dq = motor_fb_space.velocity /
        //     100000.0 * (2 * PI)); (abs(motor_fb_space.torque / 2000.0) > 10.0 ? motorState->tauEst = motorState->tauEst : motorState->tauEst =
        //     motor_fb_space.torque / 2000.0);
        // }

        double d  = 64.3 / 2.0;
        double L1 = 25.0;
        double h1 = 112.0;
        double h2 = 65.0;

        double tx = 10.0 / 180.0 * M_PI * cos(m_t);
        double ty = 20.0 / 180.0 * M_PI * sin(m_t);

        double angle = 5.0 / 180.0 * M_PI * sin(m_t);

        std::array<double, 2> result = pi::ankle_ik(d, L1, h1, h2, tx, ty);

        RCLCPP_INFO(this->get_logger(), "angle: %lf", angle);
        RCLCPP_INFO(this->get_logger(), "tx: %lf, ty: %lf, m1: %lf, m2: %lf", tx, ty, result[0], result[1]);

        angle     = angle / (M_PI * 2.0) * 100000.0;
        result[0] = result[0] / (M_PI * 2.0) * 2000000.0;
        result[1] = result[1] / (M_PI * 2.0) * 2000000.0;

        // m_driver.set_can1_motor_pos(1, -result[0]);
        // m_driver.set_can1_motor_pos(0, result[1]);

        // m_driver.set_can2_motor_pos(1, -result[0]);
        // m_driver.set_can2_motor_pos(0, result[1]);

        // m_driver.set_can1_motor_pos(2, angle);

        m_driver.send_spi();

        m_t += 0.01;
    }

private:
    // 声名定时器指针
    rclcpp::TimerBase::SharedPtr m_timer;

    pi::SpiDriver  m_driver;
    pi::LegControl m_leg_left;
    pi::LegControl m_leg_right;

    double m_t = 0.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // 创建对应节点的共享指针对象
    auto node = std::make_shared<HumanoidControl>("humanoid_control");
    // 运行节点，并检测退出信号
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}