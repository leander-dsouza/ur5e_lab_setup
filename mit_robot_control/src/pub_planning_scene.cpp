// MIT License
// Copyright (c) 2022 Leander Stephen D'Souza

/**
 * Program to publish planning scene objects
 */

#include <tf2/LinearMath/Quaternion.h>
#include <moveit/planning_scene/planning_scene.h>

#include <algorithm>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <gazebo_msgs/msg/link_states.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


/**
 * PlanningSceneHandler Class.
 */
class PlanningSceneHandler : public rclcpp::Node {
 private:
  rclcpp::Subscription<
    gazebo_msgs::msg::LinkStates>::SharedPtr link_states_sub_;
  rclcpp::Publisher<
    moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Pose base_frame_pose_;
  geometry_msgs::msg::Pose object_pose_;
  geometry_msgs::msg::Pose table_pose_;

 public:
  /**
   * Initialize the PlanningSceneHandler Class.
   */
  PlanningSceneHandler()
  : Node("planning_scene_handler_node") {

    base_frame_pose_ = geometry_msgs::msg::Pose();
    object_pose_ = geometry_msgs::msg::Pose();
    table_pose_ = geometry_msgs::msg::Pose();

    // Subscribers
    link_states_sub_ = this->create_subscription<
      gazebo_msgs::msg::LinkStates>("/gazebo/link_states", 10,
        std::bind(&PlanningSceneHandler::link_states_callback,
          this, std::placeholders::_1));

    // Publishers
    planning_scene_pub_ = this->create_publisher<
      moveit_msgs::msg::PlanningScene>("/planning_scene", 10);

    // Timers
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
        std::bind(&PlanningSceneHandler::timer_callback, this));
  }

  /**
   * Listen for the /gazebo/link_states topic.
   */
  void link_states_callback(
    const gazebo_msgs::msg::LinkStates::SharedPtr msg) {

      for (size_t i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == "ur::base_link") {
            base_frame_pose_ = msg->pose[i];
        }
        if (msg->name[i] == "cube::link") {
          if (!(base_frame_pose_ == geometry_msgs::msg::Pose())) {
            get_relative_pose(msg->name[i], msg->pose[i],
              base_frame_pose_, {0.0, 0.0, 0.0});
          }
        }
        if (msg->name[i] == "table::link") {
          if (!(base_frame_pose_ == geometry_msgs::msg::Pose())) {
            get_relative_pose(msg->name[i], msg->pose[i],
              base_frame_pose_, {0.0, 0.0, 1.0 - 0.02});
          }
        }
      }
  }
  /**
   * Update the chatter topic.
   */
  void get_relative_pose(std::string model_name,
    geometry_msgs::msg::Pose &model_pose,
    geometry_msgs::msg::Pose &base_frame_pose,
    std::vector<double> origin_compensation) {

    // compensate for the origin of the model
    model_pose.position.x += origin_compensation[0];
    model_pose.position.y += origin_compensation[1];
    model_pose.position.z += origin_compensation[2];

    // shift origin to base frame pose
    model_pose.position.x -= base_frame_pose.position.x;
    model_pose.position.y -= base_frame_pose.position.y;
    model_pose.position.z -= base_frame_pose.position.z;

    // shift origin quaternion to base frame pose
    tf2::Quaternion model_quat, base_quat;
    tf2::fromMsg(model_pose.orientation, model_quat);
    tf2::fromMsg(base_frame_pose.orientation, base_quat);
    model_quat = base_quat.inverse() * model_quat;
    model_pose.orientation = tf2::toMsg(model_quat);

    if (model_name == "cube::link") {
      object_pose_ = model_pose;
    }
    if (model_name == "table::link") {
      table_pose_ = model_pose;
    }
  }

  void spawn_object(const moveit_msgs::msg::CollisionObject& object) {
    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(object);
    planning_scene.is_diff = true;
    planning_scene_pub_->publish(planning_scene);
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  /**
   * Publish the planning scene.
   */
  void pub_planning_scene() {
    // publish table
    moveit_msgs::msg::CollisionObject object;
    object.id = "table";
    object.header.frame_id = "base_link";
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    object.primitives[0].dimensions.resize(3);
    object.primitives[0].dimensions[0] = 1.5;
    object.primitives[0].dimensions[1] = 0.8;
    object.primitives[0].dimensions[2] = 0.03;
    object.primitive_poses.resize(1);
    object.primitive_poses[0] = table_pose_;
    spawn_object(object);

    // publish object
    object.id = "object";
    object.header.frame_id = "base_link";
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    object.primitives[0].dimensions.resize(3);
    object.primitives[0].dimensions[0] = 0.1;
    object.primitives[0].dimensions[1] = 0.1;
    object.primitives[0].dimensions[2] = 0.1;
    object.primitive_poses.resize(1);
    object.primitive_poses[0] = object_pose_;
    spawn_object(object);
  }

  /**
   * Timer callback function.
   */
  void timer_callback() {
    // wait for the base frame pose, object pose, and table pose
    if (base_frame_pose_ == geometry_msgs::msg::Pose() ||
      object_pose_ == geometry_msgs::msg::Pose() ||
      table_pose_ == geometry_msgs::msg::Pose()) {
      RCLCPP_INFO(this->get_logger(), "Waiting for gazebo/link_states topic");
      return;
    }
    pub_planning_scene();

    // exit the node
    rclcpp::shutdown();
    exit(0);
  }
};


/**
 * Mimic Main Function.
 */
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanningSceneHandler>());
  rclcpp::shutdown();

  return 0;
}
