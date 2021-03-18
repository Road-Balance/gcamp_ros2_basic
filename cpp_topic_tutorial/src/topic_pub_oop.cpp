#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class CounterPub : public rclcpp::Node {
private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_publisher;

    rclcpp::TimerBase::SharedPtr m_timer;

    // declaring non-static data members as 'auto'
    // https://stackoverflow.com/questions/11302981/c11-declaring-non-static-data-members-as-auto
    std_msgs::msg::Int32 msg = std_msgs::msg::Int32();

    void timer_callback(){
        msg.data++;
        m_publisher->publish(msg);
    }

public:
    CounterPub(): Node("topic_pub_oop_node") {
        m_publisher = this->create_publisher<std_msgs::msg::Int32>("/counter", 10);
        m_timer = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&CounterPub::timer_callback, this)
        );
        msg.data = 0;
    }

};

int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CounterPub>());
    rclcpp::shutdown();

    return 0;
}