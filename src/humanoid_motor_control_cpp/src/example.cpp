#include "rclcpp/rclcpp.hpp"

#include "humanoid_motor_control_cpp/kinematic/ankle_ik.h"
#include "humanoid_motor_control_cpp/motor_driver.h"

#include <string>
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

class HumanoidControl : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    HumanoidControl(std::string name) : Node(name), m_driver("/dev/spidev4.1")
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
        // 创建发布者
        // m_command_publisher = this->create_publisher<std_msgs::msg::String>("command", 10);
        // 创建定时器，500ms为周期，定时发布
        m_timer = this->create_wall_timer(std::chrono::milliseconds(2), std::bind(&HumanoidControl::timer_callback, this));

        m_driver.initialize();
    }

private:
    void timer_callback()
    {
        // 创建消息
        std::string message = "forward";
        // 日志打印
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.c_str());

        double d  = 64.3 / 2.0;
        double L1 = 25.0;
        double h1 = 112.0;
        double h2 = 65.0;

        double tx = 10.0 / 180.0 * M_PI * sin(m_t);
        double ty = 20.0 / 180.0 * M_PI * sin(m_t);

        std::array<double, 2> result = pi::ankle_ik(d, L1, h1, h2, tx, ty);

        RCLCPP_INFO(this->get_logger(), "tx: %lf, ty: %lf, m1: %lf, m2: %lf", tx, ty, result[0], result[1]);

        result[0] = result[0] / (M_PI * 2.0) * 2000000.0;
        result[1] = result[1] / (M_PI * 2.0) * 2000000.0;

        m_driver.set_can1_motor_pos(1, result[0]);
        m_driver.set_can1_motor_pos(0, -result[1]);

        m_driver.send_spi();

        m_t += 0.01;
    }

private:
    // 声名定时器指针
    rclcpp::TimerBase::SharedPtr m_timer;

    pi::MotorDriver m_driver;

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