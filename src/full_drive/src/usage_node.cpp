#include "full_drive/client.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <functional>
#include <Eigen/Dense>


class ClientNode : public rclcpp::Node {
public:
    ClientNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("client_node", options), client_actions_(std::make_shared<ClientActions>()) {
        // Fit the polynomial coefficients once during initialization
        fitPolynomialCoefficients();
        executeActions();
    }

private:
    std::shared_ptr<ClientActions> client_actions_;
    const int max_attempts = 5;
    Eigen::VectorXd coeffs;

    void fitPolynomialCoefficients() {
        // Provided data points
        std::vector<double> cm_values = {14.0, 13.5, 11.5, 9.0, 6.5, 4.0, 2.0, 0.0};
        std::vector<double> gripper_values = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};

        // Fit a 2nd-degree polynomial to the data
        coeffs = polyfit(cm_values, gripper_values, 2);

        // Print the polynomial coefficients
        RCLCPP_INFO(this->get_logger(), "Polynomial coefficients: %f %f %f", coeffs[0], coeffs[1], coeffs[2]);
    }

    Eigen::VectorXd polyfit(const std::vector<double>& x, const std::vector<double>& y, int degree) {
        Eigen::MatrixXd A(x.size(), degree + 1);
        Eigen::VectorXd b(y.size());

        for (size_t i = 0; i < x.size(); ++i) {
            b(i) = y[i];
            for (int j = 0; j < degree + 1; ++j) {
                A(i, j) = pow(x[i], j);
            }
        }

        // Solve for coefficients
        Eigen::VectorXd coeffs = A.colPivHouseholderQr().solve(b);
        return coeffs;
    }
    double polynomialRegression(double cm) {
        double result = 0.0;
        for (int i = 0; i < coeffs.size(); ++i) {
            result += coeffs[i] * pow(cm, i);
        }
        return result;
    }

    bool tryAction(std::function<bool()> action, const std::string& action_name) {
        for (int attempt = 0; attempt < max_attempts; ++attempt) {
            if (action()) {
                return true;
            }
            RCLCPP_WARN(this->get_logger(), "Attempt %d of %d for %s failed. Retrying...", attempt + 1, max_attempts, action_name.c_str());
        }
        RCLCPP_ERROR(this->get_logger(), "All %d attempts for %s failed.", max_attempts, action_name.c_str());
        return false;
    }

    void executeActions() {
        tf2::Quaternion q1;
        q1.setRPY(M_PI, 0, 0);
        // Second moveTo with additional 90 degrees rotation on y-axis
        tf2::Quaternion q2 = q1 * tf2::Quaternion(tf2::Vector3(0, 1, 0), M_PI_2);
        // First moveLinear with additional 90 degrees rotation on z-axis
        tf2::Quaternion q3 = q2 * tf2::Quaternion(tf2::Vector3(0, 0, 1), M_PI_2);
        double piece_height = 0.015;  // the height of the piece is 1.5cm
        double piece_width = 0.025;  // the width of the piece is 2.5cm
        double robot_base_height = 0.015; // 1,5 cm is thew hieght of the base of the real robot, that it is not configured in rviz 
        double dif_btw_tip_tcp = 0.24; // 22cm the distacne between the tip of the gripper and the TCP point of the wrist3_link (the planning target will get to that link point)
        
        bool add_constrain;
        bool precise_motion;
        // information about the set gripper function:
        // the 0.0 is open, 0.7 is close, and in between is not linear estimation
        // Value | gripper state in cm
        //  0.0  |    14   cm  (full open)
        //  0.1  |    13.5 cm
        //  0.2  |    11.5 cm
        //  0.3  |    9    cm
        //  0.4  |    6.5  cm
        //  0.5  |    4    cm
        //  0.6  |    2    cm
        //  0.7  |    0    cm   (full close)

        // Example target poses
        geometry_msgs::msg::PoseStamped objects_poses = createTargetPose(0.37, 0.32, 0.5, q1);
        geometry_msgs::msg::PoseStamped fisrt_object_pose = createTargetPose(0.37, 0.32, 0.45, q1);
      
        geometry_msgs::msg::PoseStamped targets_poses = createTargetPose(0.35, 0.14, 0.5, q1);
        geometry_msgs::msg::PoseStamped first_target_pose = createTargetPose(0.37, 0.32, 0.1 + dif_btw_tip_tcp, q1);
        
        geometry_msgs::msg::PoseStamped target_pose_4 = createTargetPose(0.37, 0.32, 0.01 + dif_btw_tip_tcp, q1);
        geometry_msgs::msg::PoseStamped target_pose_5 = createTargetPose(0.37, 0.32, 0.0 + dif_btw_tip_tcp + piece_height/2.0 - robot_base_height , q1);
        geometry_msgs::msg::PoseStamped target_pose_6 = createTargetPose(0.37, 0.10, 0.5, q1);

        RCLCPP_INFO(this->get_logger(), "setting gripper to 0.00");
        if (!tryAction([&]() { return setGripper(setGripperInCm(0)); }, "setGripper")) return;
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        RCLCPP_INFO(this->get_logger(), "setting gripper to 0.00");
        if (!tryAction([&]() { return setGripper(setGripperInCm(1)); }, "setGripper")) return;
        std::this_thread::sleep_for(std::chrono::seconds(5));

        RCLCPP_INFO(this->get_logger(), "setting gripper to 0.00");
        if (!tryAction([&]() { return setGripper(setGripperInCm(2)); }, "setGripper")) return;
        std::this_thread::sleep_for(std::chrono::seconds(5));
        RCLCPP_INFO(this->get_logger(), "setting gripper to 0.00");
        if (!tryAction([&]() { return setGripper(setGripperInCm(3)); }, "setGripper")) return;
        std::this_thread::sleep_for(std::chrono::seconds(5));RCLCPP_INFO(this->get_logger(), "setting gripper to 0.00");
        if (!tryAction([&]() { return setGripper(setGripperInCm(4)); }, "setGripper")) return;
        std::this_thread::sleep_for(std::chrono::seconds(5));
        RCLCPP_INFO(this->get_logger(), "setting gripper to 0.00");
        if (!tryAction([&]() { return setGripper(setGripperInCm(5)); }, "setGripper")) return;
        std::this_thread::sleep_for(std::chrono::seconds(5));
        RCLCPP_INFO(this->get_logger(), "setting gripper to 0.00");
        if (!tryAction([&]() { return setGripper(setGripperInCm(6.5)); }, "setGripper")) return;
        std::this_thread::sleep_for(std::chrono::seconds(5));
        RCLCPP_INFO(this->get_logger(), "setting gripper to 0.00");
        if (!tryAction([&]() { return setGripper(setGripperInCm(10)); }, "setGripper")) return;
        std::this_thread::sleep_for(std::chrono::seconds(5));
        RCLCPP_INFO(this->get_logger(), "setting gripper to 0.00");
        if (!tryAction([&]() { return setGripper(setGripperInCm(12)); }, "setGripper")) return;
        std::this_thread::sleep_for(std::chrono::seconds(5));
        RCLCPP_INFO(this->get_logger(), "setting gripper to 0.00");
        if (!tryAction([&]() { return setGripper(setGripperInCm(14)); }, "setGripper")) return;
        std::this_thread::sleep_for(std::chrono::seconds(5));

        RCLCPP_INFO(this->get_logger(), "setting gripper to 0.10");
        if (!tryAction([&]() { return setGripper(0.10); }, "setGripper")) return;
        std::this_thread::sleep_for(std::chrono::seconds(5));

        RCLCPP_INFO(this->get_logger(), "setting gripper to 0.20");
        if (!tryAction([&]() { return setGripper(0.20); }, "setGripper")) return;
        std::this_thread::sleep_for(std::chrono::seconds(5));

        RCLCPP_INFO(this->get_logger(), "setting gripper to 0.30");
        if (!tryAction([&]() { return setGripper(0.30); }, "setGripper")) return;
        std::this_thread::sleep_for(std::chrono::seconds(5));

        RCLCPP_INFO(this->get_logger(), "setting gripper to 0.40");
        if (!tryAction([&]() { return setGripper(0.40); }, "setGripper")) return;
        std::this_thread::sleep_for(std::chrono::seconds(5));

        RCLCPP_INFO(this->get_logger(), "setting gripper to 0.50");
        if (!tryAction([&]() { return setGripper(0.50); }, "setGripper")) return;
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        RCLCPP_INFO(this->get_logger(), "setting gripper to 0.60");
        if (!tryAction([&]() { return setGripper(0.60); }, "setGripper")) return;
        std::this_thread::sleep_for(std::chrono::seconds(5));

        RCLCPP_INFO(this->get_logger(), "setting gripper to 0.70");
        if (!tryAction([&]() { return setGripper(0.70); }, "setGripper")) return;
        std::this_thread::sleep_for(std::chrono::seconds(5));
        /* // start tasks
        RCLCPP_INFO(this->get_logger(), "adding the cylinder object");
        if (!tryAction([&]() { 
            return addCollisionObject("cylinder_object1", shape_msgs::msg::SolidPrimitive::CYLINDER, {0.015, 0.025/2.0}, {0.37, 0.32, 0.008}, createColor(0.8, 0.2, 0.2, 1.0));
        }, "addCollisionObject(cylinder_object1)")) return;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        RCLCPP_INFO(this->get_logger(), "setting gripper to 0.35");
        if (!tryAction([&]() { return setGripper(0.35); }, "setGripper")) return;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        RCLCPP_INFO(this->get_logger(), "moving robot to target position with shoulder constrain");
        if (!tryAction([&]() { return moveTo(target_pose_1, add_constrain = true, precise_motion = false); }, "moveTo")) return;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        RCLCPP_INFO(this->get_logger(), "moving to target linearly");
        if (!tryAction([&]() { return moveLinear(target_pose_2); }, "moveLinear")) return;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        RCLCPP_INFO(this->get_logger(), "allowing collisions with the cylinder object");
        if (!tryAction([&]() { return allowCollision("hand", "cylinder_object1"); }, "allowCollision(hand, cylinder_object1)")) return;
        if (!tryAction([&]() { return allowCollision("surface", "cylinder_object1"); }, "allowCollision(surface, cylinder_object1)")) return;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        RCLCPP_INFO(this->get_logger(), "moving to target linearly");
        if (!tryAction([&]() { return moveLinear(target_pose_3); }, "moveLinear")) return;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        if (!tryAction([&]() { return allowCollision("hand", "surface"); }, "allowCollision(hand, cylinder_object1)")) return;

        RCLCPP_INFO(this->get_logger(), "setting gripper to 0.40");
        if (!tryAction([&]() { return setGripper(0.40); }, "setGripper")) return;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        RCLCPP_INFO(this->get_logger(), "moving to target linearly");
        if (!tryAction([&]() { return moveLinear(target_pose_5); }, "moveLinear")) return;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        RCLCPP_INFO(this->get_logger(), "attaching the cylinder object to the gripper");
        if (!tryAction([&]() { return attachObject("cylinder_object1"); }, "attachObject(cylinder_object1)")) return;

        RCLCPP_INFO(this->get_logger(), "setting gripper to 0.70");
        if (!tryAction([&]() { return setGripper(0.70); }, "setGripper")) return;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        RCLCPP_INFO(this->get_logger(), "moving to target linearly");
        if (!tryAction([&]() { return moveLinear(target_pose_3); }, "moveLinear")) return;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        if (!tryAction([&]() { return reenableCollision("hand", "surface"); }, "allowCollision(hand, cylinder_object1)")) return;

        RCLCPP_INFO(this->get_logger(), "moving robot to target position with shoulder constrain");
        if (!tryAction([&]() { return moveTo(target_pose_6, add_constrain = true, precise_motion = false); }, "moveTo")) return;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        RCLCPP_INFO(this->get_logger(), "setting gripper to 0.00");
        if (!tryAction([&]() { return setGripper(0.00); }, "setGripper")) return;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        RCLCPP_INFO(this->get_logger(), "detaching the cylinder object to the gripper");
        if (!tryAction([&]() { return detachObject("cylinder_object1"); }, "attachObject(cylinder_object1)")) return; */

        //if (!tryAction([&]() { return checkRobotStatus(); }, "checkRobotStatus")) return;

        /* if (!moveTo(createTargetPose(0.37, 0.32, 0.008 + 0.20 + 0.1, q1))) return;
        if (!checkRobotStatus()) return;

        if (!moveLinear(createTargetPose(0.37, 0.32, 0.20 +0.008, q1))) return;
        if (!checkRobotStatus()) return;
        
        if (!allowCollision("hand", "cylinder_object1")) return;
        if (!allowCollision("surface", "cylinder_object1")) return;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        if (!setGripper(0.7)) return;
        if (!checkRobotStatus()) return;

        if (!attachObject("cylinder_object1")) return;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        if (!moveLinear(createTargetPose(0.37, 0.32, 0.22+0.008+0.1, q1))) return;
        if (!checkRobotStatus()) return; */

        //if (!moveTo(createTargetPose(0.5, 0.4, 0.3, q1))) return;
        //if (!checkRobotStatus()) return;
        //if (!checkRobotStatus()) return;
        //if (!moveLinear(createTargetPose(0.5, 0.2, 0.3, q1))) return;
        //if (!checkRobotStatus()) return;
        //if (!moveLinear(createTargetPose(0.5, 0.2, 0.6, q1))) return;
        //if (!checkRobotStatus()) return;
        //if (!allowCollision("hand", "target1")) return;
        //if (!reenableCollision("target1", "table1")) return;
        //if (!setGripper()) return;
        //if (!deleteObject("example_object2")) return;
        //if (!attachObject()) return;
        //if (!detachObject()) return;
        //if (!currentState()) return;

        RCLCPP_INFO(this->get_logger(), "All actions completed successfully.");
    }

    bool addCollisionObject(const std::string &name, uint8_t shape_type, const std::vector<double> &dimensions, const std::vector<double> &position, const std_msgs::msg::ColorRGBA &color) {
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_type;
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
    bool attachObject(const std::string &name) {
        return client_actions_->attachObject(name);
    }

    bool detachObject(const std::string &name) {
        return client_actions_->detachObject(name);
    }

    bool moveTo(const geometry_msgs::msg::PoseStamped& target_pose, bool constrain, bool precise_motion) {
        return client_actions_->moveTo(target_pose, constrain, precise_motion);
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

    bool setGripper(float finger_joint_position) {
        return client_actions_->setGripper(finger_joint_position);
    }

    float setGripperInCm(float cm) {
        float value = polynomialRegression(cm);
        RCLCPP_INFO(this->get_logger(), "Converted gripper width %f cm to value %f", cm, value);
        return value;
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