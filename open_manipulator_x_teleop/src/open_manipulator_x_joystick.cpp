
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/executors.hpp>
#include <memory>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <std_srvs/srv/trigger.hpp>

// Parameters
const std::string JOY_TOPIC = "/joy";
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const std::string BASE_FRAME_ID = "world";

// Enums for button names and axis (XBOX controller)
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

// Default values for axes
std::map<Axis, double> AXIS_DEFAULTS = { { LEFT_TRIGGER, 1.0 }, { RIGHT_TRIGGER, 1.0 } };

/**
 * @brief Convert joystick input into Twist or JointJog commands
 * @return true if TwistStamped command, false if JointJog command
 */


namespace open_manipulator_x
{
class JoyToServoPub : public rclcpp::Node
{
public:
  JoyToServoPub()
    : Node("joy_to_twist_publisher")
  {
    // Setup subscribers and publishers
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        JOY_TOPIC, 10, std::bind(&JoyToServoPub::joyCB, this, std::placeholders::_1));

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, 10);
    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, 10);
  
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

    client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(this, "gripper_controller/gripper_cmd");

  }

  void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
  {
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    if (convertJoyToCmd(msg->axes, msg->buttons, twist_msg, joint_msg))
    {
      twist_msg->header.frame_id = BASE_FRAME_ID;
      twist_msg->header.stamp = this->now();
      twist_pub_->publish(std::move(twist_msg));
    }
    else
    {
      joint_msg->header.frame_id = "link3";
      joint_msg->header.stamp = this->now();
      joint_pub_->publish(std::move(joint_msg));
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

  bool convertJoyToCmd(const std::vector<float>& axes, const std::vector<int>& buttons,
                      std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist,
                      std::unique_ptr<control_msgs::msg::JointJog>& joint)
  {
    if (buttons[A] || buttons[B] || buttons[X] || buttons[Y] || axes[D_PAD_X] || axes[D_PAD_Y])
    {
      joint->joint_names.push_back("joint1");
      joint->velocities.push_back(2.5*axes[D_PAD_X]);
      joint->joint_names.push_back("joint2");
      joint->velocities.push_back(2.5*axes[D_PAD_Y]);

      // Map the diamond to the distal joints
      joint->joint_names.push_back("joint4");
      joint->velocities.push_back(2.5*(buttons[B] - buttons[X]));
      joint->joint_names.push_back("joint3");
      joint->velocities.push_back(2.5*(buttons[Y] - buttons[A]));
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

    return true;
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


};  // class JoyToServoPub

}  // namespace open_manipulator_x

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<open_manipulator_x::JoyToServoPub>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
