#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TopicPublisher01 : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    TopicPublisher01(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
        // 创建发布者
        m_command_publisher = this->create_publisher<std_msgs::msg::String>("command", 10);
        // 创建定时器，500ms为周期，定时发布
        m_timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&TopicPublisher01::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // 创建消息
        std_msgs::msg::String message;
        message.data = "forward";
        // 日志打印
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        // 发布消息
        m_command_publisher->publish(message);
    }

private:
    // 声名定时器指针
    rclcpp::TimerBase::SharedPtr m_timer;
    // 声明话题发布者
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_command_publisher;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // 创建对应节点的共享指针对象
    auto node = std::make_shared<TopicPublisher01>("topic_publisher_01");
    // 运行节点，并检测退出信号
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

