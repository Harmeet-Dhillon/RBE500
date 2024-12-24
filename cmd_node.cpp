#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
using std::placeholders::_1;

class MySubscriber : public rclcpp::Node
{
  public:
    MySubscriber()
    : Node("my_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "topic", 10, std::bind(&MySubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::Int32 & msg) const
    {
     
      if(msg.data % 2==0){
       RCLCPP_INFO(this->get_logger(), "I received: '%d'.It is an even number",msg.data);
    }
    else{RCLCPP_INFO(this->get_logger(), "I received: '%d'.It is an odd number",msg.data);}}
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MySubscriber>());
  rclcpp::shutdown();
  return 0;
}
