#include "full_drive/client.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <functional>
#include <Eigen/Dense>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/robotiq_gripper.h>
#include "master_project_msgs/msg/move_group_action_details.hpp" 
#include "master_project_msgs/msg/move_group_waypoint.hpp"
#include "master_project_msgs/msg/move_group_joint_state.hpp"

using namespace ur_rtde;

class ClientNode : public rclcpp::Node {
public:
    ClientNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("client_node", options),ip_("10.130.1.100"), client_actions_(std::make_shared<ClientActions>()) {
        rtde_control_ = std::make_unique<RTDEControlInterface>(ip_);
        rtde_receive_ = std::make_unique<RTDEReceiveInterface>(ip_);
        gripper_ = std::make_unique<RobotiqGripper>(ip_);

        task_details_subscriber_ = this->create_subscription<master_project_msgs::msg::MoveGroupActionDetails>(
            "task_details", 10, std::bind(&ClientNode::taskDetailsCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Connecting to gripper...");
        gripper_->connect();
        RCLCPP_INFO(this->get_logger(), "Gripper connected.");
        gripper_->activate();
        RCLCPP_INFO(this->get_logger(), "Gripper activated.");

        // Fit the polynomial coefficients once during initialization
        fitPolynomialCoefficients();
        executeActions();
    }

private:
    rclcpp::Subscription<master_project_msgs::msg::MoveGroupActionDetails>::SharedPtr task_details_subscriber_;
    std::shared_ptr<ClientActions> client_actions_;
    const int max_attempts = 50;
    Eigen::VectorXd coeffs;
    std::string ip_;
    std::unique_ptr<RTDEControlInterface> rtde_control_;
    std::unique_ptr<RTDEReceiveInterface> rtde_receive_;
    std::unique_ptr<RobotiqGripper> gripper_;

    void taskDetailsCallback(const master_project_msgs::msg::MoveGroupActionDetails::SharedPtr msg) {  // Change the message type
        RCLCPP_INFO(this->get_logger(), "Received task details: %s", msg->action_name.c_str());
        processTask(*msg);
    }

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
        double piece_diameter = 0.025;  // the diameter of the piece is 2.5cm
        double robot_base_height = 0.015; // 1,5 cm is the hieght of the base of the real robot, that it is not configured in rviz 
        double dif_btw_tip_tcp = 0.24; // 22cm the distacne between the tip of the gripper and the TCP point of the wrist3_link (the planning target will get to that link point)
        double dist_btw_objects = 0.005; //0.5 cm distance between the objects, that will be grasped
        double dist_btw_targets = 0.015; // distannce between vertical targets 
        double gripper_finger_width = 0.005; // the gripper finger width is 0.5 cm
        double num_objects = 12; // this will make like counter of the number of objects that have been grasped
        double pallet_width = 0.06;  // 6cm the width of tha pallet, that the objects are going to be placed on.
        double pallet_height = 0.09;
        double pallet_offset = 0.01;  // the pallet have material with 1cm above the surfce of the robot

        double x_targets1 = 0.342; // the distance from robot origin till the lower edge of the first pallet
        double y_targets1 = 0.1345;  // the distance from robot origin till the lower left edge of the first pallet
        double x_objects1 = 0.372; // the distance from robot origin til the mid of all objects x values in robot coordinates
        double y_objects1 = 0.273; // this is the distance from robot origin till the right edge of the first object

        int cols_counter = 0;
        int rows_counter =0;

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

        for (int i = 0; i < num_objects; ++i) {
            std::string object_name = "cylinder_object" + std::to_string(i);
            RCLCPP_INFO(this->get_logger(), "Adding the %s", object_name.c_str());
            
        }
        
        // Create object poses, the output will be like: object_poses = [0,1,2,3,4,5,6,7,8,9,10,11,12], where 0 is the objects poses, and the other ones are the actual poses for every object with its number respectively
        std::vector<geometry_msgs::msg::PoseStamped> object_poses;
        for (int i = 0; i <= num_objects; ++i) {
            double y_position = y_objects1 + (i + 1) * (piece_diameter / 2.0) + i * (piece_diameter / 2.0) + i * dist_btw_objects;
            geometry_msgs::msg::PoseStamped pose = createTargetPose(x_objects1, y_position, (2 * piece_height / 3) - robot_base_height + dif_btw_tip_tcp, q1);
            object_poses.push_back(pose);
        }
        
        std::vector<geometry_msgs::msg::PoseStamped> object_mid_poses;
        for (int i = 0; i <= num_objects; ++i) {
            double y_position = y_objects1 + (i + 1) * (piece_diameter / 2.0) + i * (piece_diameter / 2.0) + i * dist_btw_objects;
            geometry_msgs::msg::PoseStamped pose = createTargetPose(x_objects1, y_position, (2 * piece_height / 3) - robot_base_height + dif_btw_tip_tcp + 0.1, q1); // 10cm above the object
            object_mid_poses.push_back(pose);
        }

        // Create target poses, the output will be like: target_poses = [0,1,2,3,4,5,6], where 0 is the targets poses, and the other ones are the actual poses for every target with its number respectively
        // the first pallet positions is added now
        // Create target poses
        std::vector<geometry_msgs::msg::PoseStamped> target_poses;
        for (int i = 0; i <= 6; ++i) {
            double x_position = x_targets1 + pallet_height - (i / 2 + 1) * (piece_diameter / 2.0) - (i / 2) * (piece_diameter / 2.0) - (i / 2) * dist_btw_targets; // examples on i/2 > 1/2 = 0, 3/2 = 1
            double y_position = y_targets1 - (i % 2 + 1) * (piece_diameter / 2.0) - (i % 2) * (piece_diameter / 2.0) - (i % 2) * dist_btw_objects;  //examples on i%2 > 1%2 = 1, 2%2 = 0, 0%2 = 0
            geometry_msgs::msg::PoseStamped pose = createTargetPose(x_position, y_position, pallet_offset + dif_btw_tip_tcp, q1);
            target_poses.push_back(pose);
        }

        std::vector<geometry_msgs::msg::PoseStamped> target_mid_poses;
        for (int i = 0; i <= 6; ++i) {
            double x_position = x_targets1 + pallet_height - (i / 2 + 1) * (piece_diameter / 2.0) - (i / 2) * (piece_diameter / 2.0) - (i / 2) * dist_btw_objects;
            double y_position = y_targets1 - (i % 2 + 1) * (piece_diameter / 2.0) - (i % 2) * (piece_diameter / 2.0) - (i % 2) * dist_btw_objects;
            geometry_msgs::msg::PoseStamped pose = createTargetPose(x_position, y_position, pallet_offset + dif_btw_tip_tcp + 0.1, q1); // 10cm above the target
            target_mid_poses.push_back(pose);
        }

        // start tasks
        for (int i = 0; i < num_objects; ++i) {
            std::string object_name = "cylinder_object" + std::to_string(i+1);
            RCLCPP_INFO(this->get_logger(), "Adding the %s", object_name.c_str());
            if (!tryAction([&]() { 
                return addCollisionObject(
                    object_name, 
                    shape_msgs::msg::SolidPrimitive::CYLINDER, 
                    {0.015, 0.025 / 2.0}, 
                    {x_objects1, y_objects1 + (i + 1) * (piece_diameter / 2.0) + i * (piece_diameter / 2.0) + i * dist_btw_objects, piece_height / 2.0}, 
                    createColor(0.8, 0.2, 0.2, 1.0)
                );
            }, "addCollisionObject(" + object_name + ")")) return;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));

        RCLCPP_INFO(this->get_logger(), "moving robot to objects position with shoulder constrain");
        if (!tryAction([&]() { return moveTo(object_mid_poses[1], add_constrain = true, precise_motion = true); }, "moveTo")) return;
        //std::this_thread::sleep_for(std::chrono::seconds(1));

        RCLCPP_INFO(this->get_logger(), "moving robot to objects position with shoulder constrain");
        if (!tryAction([&]() { return moveTo(target_mid_poses[1], add_constrain = true, precise_motion = true); }, "moveTo")) return;
        //std::this_thread::sleep_for(std::chrono::seconds(1));

        
/*         for (int i = 0; i < 6; ++i) {
            RCLCPP_INFO(this->get_logger(), "setting gripper to 0.5");
            if (!tryAction([&]() { return setGripper(0.5); }, "setGripper")) return;
            //std::this_thread::sleep_for(std::chrono::seconds(1));

            RCLCPP_INFO(this->get_logger(), "moving robot to objects position with shoulder constrain");
            if (!tryAction([&]() { return moveTo(object_mid_poses[i], add_constrain = true, precise_motion = true); }, "moveTo")) return;
            //std::this_thread::sleep_for(std::chrono::seconds(1));

            RCLCPP_INFO(this->get_logger(), "moving to objects linearly");
            if (!tryAction([&]() { return moveLinear(object_mid_poses[i]); }, "moveLinear")) return;
            //std::this_thread::sleep_for(std::chrono::seconds(1));

            RCLCPP_INFO(this->get_logger(), "allowing collisions with the cylinder object");
            std::string object_name = "cylinder_object" + std::to_string(i+1);
            if (!tryAction([&]() { return allowCollision("hand", object_name); }, "allowCollision(hand, " + object_name + ")")) return;
            if (!tryAction([&]() { return allowCollision("surface", object_name); }, "allowCollision(surface, " + object_name + ")")) return;
            //std::this_thread::sleep_for(std::chrono::seconds(1));

            if (!tryAction([&]() { return allowCollision("hand", "surface"); }, "allowCollision(hand, surface)")) return;

            RCLCPP_INFO(this->get_logger(), "setting gripper to 0.55");
            if (!tryAction([&]() { return setGripper(0.55); }, "setGripper")) return;
            //std::this_thread::sleep_for(std::chrono::seconds(1));

            RCLCPP_INFO(this->get_logger(), "moving to object linearly");
            if (!tryAction([&]() { return moveLinear(object_poses[i]); }, "moveLinear")) return;
            //std::this_thread::sleep_for(std::chrono::seconds(1));

            RCLCPP_INFO(this->get_logger(), "attaching the cylinder object to the gripper");
            if (!tryAction([&]() { return attachObject(object_name); }, "attachObject(" + object_name + ")")) return;

            RCLCPP_INFO(this->get_logger(), "setting gripper to 0.70");
            if (!tryAction([&]() { return setGripper(0.70); }, "setGripper")) return;
            //std::this_thread::sleep_for(std::chrono::seconds(1));

            RCLCPP_INFO(this->get_logger(), "back to above object linearly");
            if (!tryAction([&]() { return moveLinear(object_mid_poses[i]); }, "moveLinear")) return;
            //std::this_thread::sleep_for(std::chrono::seconds(1));

            if (!tryAction([&]() { return reenableCollision("hand", "surface"); }, "reenableCollision(hand, surface)")) return;

            RCLCPP_INFO(this->get_logger(), "moving robot to targets position with shoulder constrain");
            if (!tryAction([&]() { return moveTo(target_mid_poses[i], add_constrain = true, precise_motion = false); }, "moveTo")) return;
            //std::this_thread::sleep_for(std::chrono::seconds(1));

            RCLCPP_INFO(this->get_logger(), "moving to targets linearly");
            if (!tryAction([&]() { return moveLinear(target_mid_poses[i]); }, "moveLinear")) return;
            //std::this_thread::sleep_for(std::chrono::seconds(1));

            if (!tryAction([&]() { return allowCollision("hand", "surface"); }, "allowCollision(hand, surface)")) return;

            RCLCPP_INFO(this->get_logger(), "moving to target linearly");
            if (!tryAction([&]() { return moveLinear(target_poses[i]); }, "moveLinear")) return;
            //std::this_thread::sleep_for(std::chrono::seconds(1));

            RCLCPP_INFO(this->get_logger(), "setting gripper to 0.55");
            if (!tryAction([&]() { return setGripper(0.55); }, "setGripper")) return;
            //std::this_thread::sleep_for(std::chrono::seconds(1));

            RCLCPP_INFO(this->get_logger(), "updating rviz gripper");
            if (!tryAction([&]() { return updateGripper(get_gripper_position()); }, "updateGripper")) return;
            //std::this_thread::sleep_for(std::chrono::seconds(1));

            RCLCPP_INFO(this->get_logger(), "detaching the cylinder object from the gripper");
            if (!tryAction([&]() { return detachObject(object_name); }, "detachObject(" + object_name + ")")) return;

            RCLCPP_INFO(this->get_logger(), "finished target");
            if (!tryAction([&]() { return moveLinear(target_mid_poses[i]); }, "moveLinear")) return;
            //std::this_thread::sleep_for(std::chrono::seconds(1));

            if (!tryAction([&]() { return reenableCollision("hand", "surface"); }, "reenableCollision(hand, surface)")) return;
        } */

        RCLCPP_INFO(this->get_logger(), "All actions completed successfully.");
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

    double get_gripper_position() {
        auto gripper_status = gripper_->getCurrentPosition();
        double position = gripper_status; // Normalize position between 0 and 1
        RCLCPP_INFO(this->get_logger(), "Real Gripper Position: %f", position);
        return position;
    }

    void processTask(const master_project_msgs::msg::MoveGroupActionDetails& task_details) {
        std::lock_guard<std::mutex> lock(task_mutex_);  // Lock the mutex at the beginning
        RCLCPP_INFO(this->get_logger(), "Processing task for group: %s, action: %s", task_details.group_name.c_str(), task_details.action_name.c_str());

        std::vector<std::vector<double>> robot_path;
        double initial_gripper_position = -1;
        double final_gripper_position = -1;
        double finger_change = 0.005;

        // Check if waypoints are empty
        if (task_details.waypoints.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No waypoints to process.");
            return;
        }

        for (const auto& waypoint : task_details.waypoints) {
            std::vector<double> joint_positions;

            // Log the number of joints in the waypoint
            RCLCPP_INFO(this->get_logger(), "Processing waypoint with %zu joints", waypoint.joints.size());

            for (const auto& joint : waypoint.joints) {
                RCLCPP_INFO(this->get_logger(), "Joint name: %s, position: %f", joint.name.c_str(), joint.position);
                joint_positions.push_back(joint.position);
            }

            joint_positions.push_back(waypoint.speed);
            joint_positions.push_back(waypoint.acceleration);
            if(task_details.precise_motion == true){
                joint_positions.push_back(0.001); // Tolerance or blend
            
            } else if(task_details.precise_motion == false){
                joint_positions.push_back(0.02); // Tolerance or blend
            
            }
            robot_path.push_back(joint_positions);
        }

        if (task_details.group_name == "ur5e_arm"){               
            rtde_control_->moveJ(robot_path);

        } else if (task_details.group_name == "gripper") {
            if (!task_details.waypoints.front().joints.empty()) {
                initial_gripper_position = task_details.waypoints.front().joints.back().position;
            }
            if (!task_details.waypoints.back().joints.empty()) {
                final_gripper_position = task_details.waypoints.back().joints.back().position;
            }

            RCLCPP_INFO(this->get_logger(), "Initial gripper position: %f, Final gripper position: %f", initial_gripper_position, final_gripper_position);

            if (abs(abs(initial_gripper_position) - abs(final_gripper_position)) > finger_change) {
                RCLCPP_INFO(this->get_logger(), "Moving gripper to position: %f", final_gripper_position);
                final_gripper_position = map_finger_joint_to_gripper(final_gripper_position);
                final_gripper_position = std::clamp(final_gripper_position, 0.0, 1.0);
                gripper_->move(final_gripper_position);
                RCLCPP_INFO(this->get_logger(), "Gripper moved to position: %f", final_gripper_position);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        // Keep checking if the robot is busy
        RCLCPP_INFO(this->get_logger(), "Checking if the robot is busy...");
        while (isRobotBusy()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            RCLCPP_INFO(this->get_logger(), "Robot is still busy, sleeping 1 second");
        }
        RCLCPP_INFO(this->get_logger(), "Robot is now idle. Task processing completed.");
    }

    double map_finger_joint_to_gripper(double finger_joint_position) {
        double close_finger_joint = 0.69991196175;
        double open_finger_joint = 0.07397215645645;
        return map_value(finger_joint_position, open_finger_joint, close_finger_joint, 0.988235, 0.105882);
    }

    double map_value(double value, double fromLow, double fromHigh, double toLow, double toHigh) {
        return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
    }

        float setGripperInCm(float cm) {
        float value = polynomialRegression(cm);
        RCLCPP_INFO(this->get_logger(), "Converted gripper width %f cm to value %f", cm, value);
        return value;
    }

    //////////////////////////////////////////////////////////////////////////////////
    ///////// Actions ////////////////////////////////////////////////////////////////
    ///////// vvvvvvv ////////////////////////////////////////////////////////////////

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

    bool updateGripper(float finger_joint_position) {
        return client_actions_->updateGripper(finger_joint_position);
    }

};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}