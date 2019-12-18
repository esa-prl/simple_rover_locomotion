#ifndef LOCOMOTION_MODE_H
#define LOCOMOTION_MODE_H

#include "rclcpp/rclcpp.hpp"
#include <urdf/model.h>

#include <chrono>
#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <simple_rover_locomotion/msg/joint_command.hpp>
#include <simple_rover_locomotion/msg/joint_command_array.hpp>

#include <simple_rover_locomotion/srv/activate.hpp>
#include <simple_rover_locomotion/srv/change_locomotion_mode.hpp>

using namespace std::chrono_literals;


class LocomotionMode : public rclcpp::Node
{
  public:
    LocomotionMode(rclcpp::NodeOptions options);

    // TODO: Inside this namespace? 
    // TODO: What should be pointers, what not?
    struct Motor
    {
        std::shared_ptr<urdf::Joint> joint; 
        std::shared_ptr<urdf::Link> link; 
        urdf::Pose global_pose;

    };

    struct Leg
    {
        std::shared_ptr<Motor> driving_motor;
        std::shared_ptr<Motor> steering_motor;
        std::shared_ptr<Motor> deployment_motor;
        std::vector<std::shared_ptr<Motor>> motors;

        Leg() :
        driving_motor(std::make_shared<Motor>()),
        steering_motor(std::make_shared<Motor>()),
        deployment_motor(std::make_shared<Motor>())
        {
            motors.push_back(driving_motor);
            motors.push_back(steering_motor);
            motors.push_back(deployment_motor);
        } 
    };

  protected:
    // Joints Pulisher
    rclcpp::Publisher<simple_rover_locomotion::msg::JointCommandArray>::SharedPtr joints_publisher_;

    // Velocities Callback
    virtual void rover_velocities_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // Initialize Subscriber with callback function from derived class
    void initialize_subscribers();

    // Load parameters
    void load_params();

    // Load robot model (urdf)
    void load_robot_model();

    // Initialize Messages
    // simple_rover_locomotion::msg::JointCommand joint_command_;
    simple_rover_locomotion::msg::JointCommandArray joint_command_array_;

    sensor_msgs::msg::JointState current_joint_state_;

    // Model
    std::shared_ptr<urdf::Model> model_;
    std::vector<std::shared_ptr<LocomotionMode::Leg>> legs_;

  private:

    // Services Objects
    rclcpp::Service<simple_rover_locomotion::srv::Activate>::SharedPtr activate_service_;
    rclcpp::Service<simple_rover_locomotion::srv::ChangeLocomotionMode>::SharedPtr changelocomotionmode_service_;
 
    // Services Callbacks
    void activate(const simple_rover_locomotion::srv::Activate::Request::SharedPtr request,
            std::shared_ptr<simple_rover_locomotion::srv::Activate::Response>      response);
    void change_locomotion_mode(const simple_rover_locomotion::srv::ChangeLocomotionMode::Request::SharedPtr request,
                          std::shared_ptr<simple_rover_locomotion::srv::ChangeLocomotionMode::Response>      response);

    // Rover Velocities Subscription
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr rover_velocities_subscription_;    

    // Joint States Subscription
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;

    // Joint States Callback
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // Model
    std::string model_name_;
    std::string model_dir_;
    std::string model_path_;

    std::vector<std::shared_ptr<urdf::Joint>> joints_;
    std::vector<std::shared_ptr<urdf::Link>> links_;

    void init_motor(std::shared_ptr<LocomotionMode::Motor> &motor, std::shared_ptr<urdf::Link> link);

    std::shared_ptr<LocomotionMode::Motor> init_motor(std::shared_ptr<urdf::Link> link);

    // Find first joint in leg, which name contains the specified name
    std::shared_ptr<urdf::Link> get_link_in_leg(std::shared_ptr<urdf::Link> &start_link, std::string name);  

    urdf::Pose get_parent_joint_position(std::shared_ptr<urdf::Link> &link);

    urdf::Pose transpose_pose(urdf::Pose parent, urdf::Pose child);


    // Parameters
    // string mode_name_;

};

#endif