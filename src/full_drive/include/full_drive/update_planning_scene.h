#ifndef UPDATE_PLANNING_SCENE_H
#define UPDATE_PLANNING_SCENE_H

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <string>
#include <vector>
#include <moveit_msgs/msg/object_color.hpp>

class PlanningSceneUpdater {
public:
    PlanningSceneUpdater(const rclcpp::Node::SharedPtr& node)
        : node_(node), planning_scene_interface_() {}

    void addCollisionObject(const std::string& id, const shape_msgs::msg::SolidPrimitive& primitive, const geometry_msgs::msg::Pose& pose, const std_msgs::msg::ColorRGBA& color) {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = id;
        collision_object.header.frame_id = "world";
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(pose);
        collision_object.operation = collision_object.ADD;

        // Add the color to the marker
        moveit_msgs::msg::ObjectColor object_color = addColor(id, color);
        
        // Create a PlanningScene message and add the collision object and its color
        moveit_msgs::msg::PlanningScene planning_scene_msg;
        planning_scene_msg.is_diff = true;
        planning_scene_msg.world.collision_objects.push_back(collision_object);
        planning_scene_msg.object_colors.push_back(object_color);

        // Apply the planning scene
        planning_scene_interface_.applyPlanningScene(planning_scene_msg);
        
        RCLCPP_INFO(node_->get_logger(), "Added collision object: %s", id.c_str());
    }

    void removeCollisionObject(const std::string& id) {
        std::vector<std::string> object_ids;
        object_ids.push_back(id);
        planning_scene_interface_.removeCollisionObjects(object_ids);
        RCLCPP_INFO(node_->get_logger(), "Removed collision object: %s", id.c_str());
    }

private:
    moveit_msgs::msg::ObjectColor addColor(const std::string& id, const std_msgs::msg::ColorRGBA& color) {
        moveit_msgs::msg::ObjectColor object_color;
        object_color.id = id;
        object_color.color = color;
        

        RCLCPP_INFO(node_->get_logger(), "Added color for object: %s", id.c_str());
        return object_color;
    }

    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};

#endif // UPDATE_PLANNING_SCENE_H
