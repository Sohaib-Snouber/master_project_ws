#include "full_drive/client.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/color_rgba.hpp>

class ClientNode : public rclcpp::Node {
public:
    ClientNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("client_node", options), client_actions_(std::make_shared<ClientActions>()) {
        executeActions();
    }

private:
    std::shared_ptr<ClientActions> client_actions_;

    void executeActions() {
        if (!addCollisionObject()) return;
        if (!moveTo()) return;
        if (!moveLinear()) return;
        if (!attachObject()) return;
        if (!detachObject()) return;
        if (!deleteObject()) return;
        if (!checkRobotStatus()) return;
        if (!allowCollision("hand", "target1")) return;
        if (!allowCollision("target1", "table1")) return;
        if (!reenableCollision("target1", "table1")) return;

        RCLCPP_INFO(this->get_logger(), "All actions completed successfully.");
    }

    bool addCollisionObject() {
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {0.1, 0.1, 0.1}; // Example dimensions

        geometry_msgs::msg::Pose pose;
        pose.position.x = 0.5;
        pose.position.y = 0.5;
        pose.position.z = 0.5;
        pose.orientation.w = 1.0;

        std_msgs::msg::ColorRGBA color;
        color.r = 0.5;
        color.g = 0.5;
        color.b = 0.5;
        color.a = 1.0;

        return client_actions_->addCollisionObject("example_object", primitive, pose, color);
    }

    bool moveTo() {
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "world";
        target_pose.pose.position.x = 0.6;
        target_pose.pose.position.y = 0.0;
        target_pose.pose.position.z = 0.5;
        target_pose.pose.orientation.w = 1.0;

        return client_actions_->moveTo(target_pose);
    }

    bool moveLinear() {
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose.position.x = 0.7;
        target_pose.pose.position.y = 0.0;
        target_pose.pose.position.z = 0.5;
        target_pose.pose.orientation.w = 1.0;

        return client_actions_->moveLinear(target_pose);
    }

    bool attachObject() {
        return client_actions_->attachObject("example_object");
    }

    bool detachObject() {
        return client_actions_->detachObject("example_object");
    }

    bool deleteObject() {
        return client_actions_->deleteObject("example_object");
    }

    bool checkRobotStatus() {
        return client_actions_->checkRobotStatus();
    }

    bool allowCollision(const std::string &object1, const std::string &object2) {
        return client_actions_->allowCollision(object1, object2);
    }

    bool reenableCollision(const std::string &object1, const std::string &object2) {
        return client_actions_->reenableCollision(object1, object2);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
