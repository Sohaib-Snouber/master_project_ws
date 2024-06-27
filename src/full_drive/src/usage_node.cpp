#include "full_drive/client.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class ClientNode : public rclcpp::Node {
public:
    ClientNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("client_node", options), client_actions_(std::make_shared<ClientActions>()) {
        executeActions();
    }

private:
    std::shared_ptr<ClientActions> client_actions_;

    void executeActions() {
        tf2::Quaternion q1;
        q1.setRPY(M_PI, 0, 0);
        // Second moveTo with additional 90 degrees rotation on y-axis
        tf2::Quaternion q2 = q1 * tf2::Quaternion(tf2::Vector3(0, 1, 0), M_PI_2);
        // First moveLinear with additional 90 degrees rotation on z-axis
        tf2::Quaternion q3 = q2 * tf2::Quaternion(tf2::Vector3(0, 0, 1), M_PI_2);
        
        
        //if (!addCollisionObject("example_object1", {0.1, 0.1, 0.1}, {0.5, 0.5, 0.5}, createColor(0.5, 0.5, 0.5, 1.0))) return;
        //if (!addCollisionObject("example_object2", {0.2, 0.2, 0.2}, {0.7, 0.5, 0.5}, createColor(0.8, 0.2, 0.2, 1.0))) return;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        //if (!deleteObject("example_object2")) return;
        //if (!attachObject()) return;
        //if (!detachObject()) return;
        //if (!currentState()) return;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (!moveTo(createTargetPose(0.5, 0.4, 0.6, q1))) return;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (!checkRobotStatus()) return;
        if (!moveTo(createTargetPose(0.5, 0.4, 0.3, q1))) return;
        if (!checkRobotStatus()) return;
        if (!moveLinear(createTargetPose(0.5, 0.6, 0.3, q1))) return;
        if (!checkRobotStatus()) return;
        if (!moveLinear(createTargetPose(0.5, 0.2, 0.3, q1))) return;
        if (!checkRobotStatus()) return;
        if (!moveLinear(createTargetPose(0.5, 0.2, 0.6, q1))) return;
        //if (!checkRobotStatus()) return;
        //if (!allowCollision("hand", "target1")) return;
        //if (!reenableCollision("target1", "table1")) return;
        //if (!setGripper()) return;

        RCLCPP_INFO(this->get_logger(), "All actions completed successfully.");
    }

    bool addCollisionObject(const std::string &name, const std::vector<double> &dimensions, const std::vector<double> &position, const std_msgs::msg::ColorRGBA &color) {
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.assign(dimensions.begin(), dimensions.end());

        geometry_msgs::msg::Pose pose;
        pose.position.x = position[0];
        pose.position.y = position[1];
        pose.position.z = position[2];
        pose.orientation.w = 1.0;

        return client_actions_->addCollisionObject(name, primitive, pose, color);
    }
    // need modifiying
    bool deleteObject(const std::string &name) {
        return client_actions_->deleteObject(name);
    }
    bool attachObject() {
        return client_actions_->attachObject("example_object");
    }

    bool detachObject() {
        return client_actions_->detachObject("example_object");
    }

    bool moveTo(const geometry_msgs::msg::PoseStamped& target_pose) {
        return client_actions_->moveTo(target_pose);
    }

    bool moveLinear(const geometry_msgs::msg::PoseStamped& target_pose) {
        return client_actions_->moveLinear(target_pose);
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
    bool currentState() {
        return client_actions_->currentState();
    }

    bool setGripper() {
        float finger_joint_position;
        finger_joint_position = 0.7;
        return client_actions_->setGripper(finger_joint_position);
    }
    std_msgs::msg::ColorRGBA createColor(float r, float g, float b, float a) {
        std_msgs::msg::ColorRGBA color;
        color.r = r;
        color.g = g;
        color.b = b;
        color.a = a;
        return color;
    }
    geometry_msgs::msg::PoseStamped createTargetPose(double px, double py, double pz, const tf2::Quaternion& q) {
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.stamp = this->now();
        target_pose.header.frame_id = "world";
        target_pose.pose.position.x = px;
        target_pose.pose.position.y = py;
        target_pose.pose.position.z = pz;
        target_pose.pose.orientation = tf2::toMsg(q);
        return target_pose;
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
