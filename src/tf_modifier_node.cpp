#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

class TFModifierNode : public rclcpp::Node
{
public:
    TFModifierNode() : Node("tf_modifier_node")
    {
        tf_subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", 10, std::bind(&TFModifierNode::tfCallback, this, std::placeholders::_1));

        tf_publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>(
            "/tf_modified", 10);
    }

private:
    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        tf2_msgs::msg::TFMessage modified_msg;
        for (const auto& transform : msg->transforms) {
            geometry_msgs::msg::TransformStamped modified_transform = transform;
            if (transform.child_frame_id == "base_link") {
                // "base_link" 프레임에 대해서만 처리
                modified_transform.transform.translation.z += 18.3031;
                modified_msg.transforms.push_back(modified_transform);
            }
            // "gnss_base_link" 프레임에 대해서는 처리하지 않음
        }
        if (!modified_msg.transforms.empty()) {
            // 수정된 메시지가 비어있지 않으면 발행
            tf_publisher_->publish(modified_msg);
        }
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
