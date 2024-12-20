/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*      Title     : joystick_servo_example.cpp
 *      Project   : moveit_servo
 *      Created   : 08/07/2020
 *      Author    : Adam Pettinger
 */

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <thread>

// We'll just set up parameters here
const std::string JOY_TOPIC = "/joy";
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const std::string EEF_FRAME_ID = "end_effector_link";
const std::string BASE_FRAME_ID = "world";

// Enums for button names -> axis/button array index
// For XBOX 1 controller
enum Axis
{
  LEFT_STICK_X = 0,
  LEFT_STICK_Y = 1,
  LEFT_TRIGGER = 2,
  RIGHT_STICK_X = 3,
  RIGHT_STICK_Y = 4,
  RIGHT_TRIGGER = 5,
  D_PAD_X = 6,
  D_PAD_Y = 7
};
enum Button
{
  A = 0,
  B = 1,
  X = 2,
  Y = 3,
  LEFT_BUMPER = 4,
  RIGHT_BUMPER = 5,
  CHANGE_VIEW = 6,
  MENU = 7,
  HOME = 8,
  LEFT_STICK_CLICK = 9,
  RIGHT_STICK_CLICK = 10
};

// Some axes have offsets (e.g. the default trigger position is 1.0 not 0)
// This will map the default values for the axes
std::map<Axis, double> AXIS_DEFAULTS = { { LEFT_TRIGGER, 1.0 }, { RIGHT_TRIGGER, 1.0 } };
std::map<Button, double> BUTTON_DEFAULTS;

// To change controls or setup a new controller, all you should to do is change the above enums and the follow 2
// functions
/** \brief // This converts a joystick axes and buttons array to a TwistStamped or JointJog message
 * @param axes The vector of continuous controller joystick axes
 * @param buttons The vector of discrete controller button values
 * @param twist A TwistStamped message to update in prep for publishing
 * @param joint A JointJog message to update in prep for publishing
 * @return return true if you want to publish a Twist, false if you want to publish a JointJog
 */


/** \brief // This should update the frame_to_publish_ as needed for changing command frame via controller
 * @param frame_name Set the command frame to this
 * @param buttons The vector of discrete controller button values
 */


namespace open_manipulator_x
{
class JoyToServoPub : public rclcpp::Node
{
public:
  JoyToServoPub(const rclcpp::NodeOptions& options)
    : Node("joy_to_twist_publisher", options), frame_to_publish_(BASE_FRAME_ID)
  {
    // Setup pub/sub
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        JOY_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::Joy::ConstSharedPtr& msg) { return joyCB(msg); });

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, rclcpp::SystemDefaultsQoS());
    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, rclcpp::SystemDefaultsQoS());

    // Create a service client to start the ServoNode
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

    // connect_moveit_servo();
    // start_moveit_servo();
    servo_stop_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/stop_servo");
    client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(this, "gripper_controller/gripper_cmd");

  }




  ~JoyToServoPub() override
  {
    // stop_moveit_servo();
  }




private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_stop_client_;
  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr client_;

  std::string frame_to_publish_;
  control_msgs::msg::GripperCommand gripper_cmd_;

bool convertJoyToCmd(const std::vector<float>& axes, const std::vector<int>& buttons,
                     std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist,
                     std::unique_ptr<control_msgs::msg::JointJog>& joint)
{
  // Give joint jogging priority because it is only buttons
  // If any joint jog command is requested, we are only publishing joint commands
  if (buttons[A] || buttons[B] || buttons[X] || buttons[Y] || axes[D_PAD_X] || axes[D_PAD_Y])
  {
    // Map the D_PAD to the proximal joints
    joint->joint_names.push_back("joint1");
    joint->velocities.push_back(2*axes[D_PAD_X]);
    joint->joint_names.push_back("joint2");
    joint->velocities.push_back(2*axes[D_PAD_Y]);

    // Map the diamond to the distal joints
    joint->joint_names.push_back("joint4");
    joint->velocities.push_back(2*(buttons[B] - buttons[X]));
    joint->joint_names.push_back("joint3");
    joint->velocities.push_back(2*(buttons[Y] - buttons[A]));
    return false;
  }
  if (buttons[LEFT_STICK_CLICK])
  {
    auto goal_msg = control_msgs::action::GripperCommand::Goal();
    goal_msg.command.position = 0.016;
    goal_msg.command.max_effort = 100.0;

    auto send_goal_options = rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&JoyToServoPub::goal_result_callback, this, std::placeholders::_1);
    client_->async_send_goal(goal_msg, send_goal_options);
  }
  else if(buttons[RIGHT_STICK_CLICK])
  {
    auto goal_msg = control_msgs::action::GripperCommand::Goal();
    goal_msg.command.position = -0.008;
    goal_msg.command.max_effort = 100.0;

    auto send_goal_options = rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&JoyToServoPub::goal_result_callback, this, std::placeholders::_1);
    client_->async_send_goal(goal_msg, send_goal_options);
  }


  // The bread and butter: map buttons to twist commands
  twist->twist.linear.z = axes[RIGHT_STICK_Y];
  twist->twist.linear.y = axes[RIGHT_STICK_X];

  double lin_x_right = -0.5 * (axes[RIGHT_TRIGGER] - AXIS_DEFAULTS.at(RIGHT_TRIGGER));
  double lin_x_left = 0.5 * (axes[LEFT_TRIGGER] - AXIS_DEFAULTS.at(LEFT_TRIGGER));
  twist->twist.linear.x = lin_x_right + lin_x_left;

  twist->twist.angular.y = axes[LEFT_STICK_Y];
  twist->twist.angular.x = axes[LEFT_STICK_X];

  double roll_positive = buttons[RIGHT_BUMPER];
  double roll_negative = -1 * (buttons[LEFT_BUMPER]);
  twist->twist.angular.z = roll_positive + roll_negative;

  // RCLCPP_INFO_STREAM(this->get_logger(), to_string(buttons[MENU]));

  return true;
}

  void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
  {
    // Create the messages we might publish
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    // This call updates the frame for twist commands
    updateCmdFrame(frame_to_publish_, msg->buttons);

    // Convert the joystick message to Twist or JointJog and publish
    if (convertJoyToCmd(msg->axes, msg->buttons, twist_msg, joint_msg))
    {
      // publish the TwistStamped
      twist_msg->header.frame_id = frame_to_publish_;
      twist_msg->header.stamp = this->now();
      twist_pub_->publish(std::move(twist_msg));
    }
    else
    {
      // publish the JointJog
      joint_msg->header.stamp = this->now();
      joint_msg->header.frame_id = frame_to_publish_;
      joint_pub_->publish(std::move(joint_msg));
    }
  }
void connect_moveit_servo()
{
  for (int i = 0; i < 10; i++) {
    if (servo_start_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO_STREAM(this->get_logger(), "SUCCESS TO CONNECT SERVO START SERVER");
      break;
    }
    RCLCPP_WARN_STREAM(this->get_logger(), "WAIT TO CONNECT SERVO START SERVER");
    if (i == 9) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "fail to connect moveit_servo." <<
          "please launch 'servo.launch' at 'open_manipulator_x_moveit_configs' pkg.");
    }
  }
  for (int i = 0; i < 10; i++) {
    if (servo_stop_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO_STREAM(this->get_logger(), "SUCCESS TO CONNECT SERVO STOP SERVER");
      break;
    }
    RCLCPP_WARN_STREAM(this->get_logger(), "WAIT TO CONNECT SERVO STOP SERVER");
    if (i == 9) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "fail to connect moveit_servo." <<
          "please launch 'servo.launch' at 'open_manipulator_x_moveit_configs' pkg.");
    }
  }
}





void stop_moveit_servo()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "call 'moveit_servo' END srv.");
  auto future = servo_stop_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
  auto result = future.wait_for(std::chrono::seconds(1));
  RCLCPP_INFO_STREAM(this->get_logger(), "Result status: " << futureStatusToString(result));
  if (result == std::future_status::ready)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "SUCCESS to stop 'moveit_servo'");
    future.get();
  }
  else
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "FAIL to stop 'moveit_servo'");
  }
}


  void start_moveit_servo()
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "call 'moveit_servo' start srv.");
    auto future = servo_start_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
    auto result = future.wait_for(std::chrono::seconds(1));
    if (result == std::future_status::ready) {
      RCLCPP_INFO_STREAM(this->get_logger(), "SUCCESS to start 'moveit_servo'");
      future.get();
    } else {
      RCLCPP_ERROR_STREAM(
        this->get_logger(), "FAIL to start 'moveit_servo', execute without 'moveit_servo'");
    }
  }
  void updateCmdFrame(std::string& frame_name, const std::vector<int>& buttons)
  {
    if (buttons[CHANGE_VIEW] && frame_name == EEF_FRAME_ID)
      frame_name = BASE_FRAME_ID;
    else if (buttons[MENU] && frame_name == BASE_FRAME_ID)
      frame_name = EEF_FRAME_ID;
  }

  void goal_result_callback(const rclcpp_action::ClientGoalHandle<control_msgs::action::GripperCommand>::WrappedResult& result)
  {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        break;
      case rclcpp_action::ResultCode::CANCELED:
        break;
      default:
        break;
    }
  }

std::string futureStatusToString(const std::future_status& status)
{
  switch (status)
  {
    case std::future_status::ready:
      return "ready";
    case std::future_status::timeout:
      return "timeout";
    case std::future_status::deferred:
      return "deferred";
    default:
      return "unknown";
  }
}


};  // class JoyToServoPub

}  // namespace moveit_servo

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<open_manipulator_x::JoyToServoPub>(options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
