#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp" // 올바른 헤더 파일 사용


class TFModifierNode : public rclcpp::Node
{
public:
    TFModifierNode() : Node("tf_modifier_node")
    {
        // Subscribe to the /tf topic
        tf_subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", 10, std::bind(&TFModifierNode::tfCallback, this, std::placeholders::_1));

        // Advertise modified tf topic
        tf_publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>(
            "/tf_modified", 10);
    }

private:
    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        // Modify the z value of each transform
        tf2_msgs::msg::TFMessage modified_msg;
        for (const auto& transform : msg->transforms) {
            geometry_msgs::msg::TransformStamped modified_transform = transform;
            modified_transform.transform.translation.z += 18.3031;
            modified_msg.transforms.push_back(modified_transform);
        }

        // Publish the modified transform
        tf_publisher_->publish(modified_msg);
    }

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFModifierNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
