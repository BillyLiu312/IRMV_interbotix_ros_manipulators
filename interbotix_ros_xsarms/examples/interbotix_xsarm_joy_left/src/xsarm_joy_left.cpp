// Copyright 2022 Trossen Robotics
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "interbotix_xs_msgs/msg/arm_joy.hpp"

typedef std::map<std::string, int> button_mappings;

// PS3 Controller button mappings
static const button_mappings ps3 = {
  {"GRIPPER_PWM_DEC", 0},  // buttons start here
  {"GRIPPER_RELEASE", 1},
  {"GRIPPER_PWM_INC", 2},
  {"GRIPPER_GRASP", 3},
  {"EE_Y_INC", 4},
  {"EE_Y_DEC", 5},
  {"WAIST_CCW", 6},
  {"WAIST_CW", 7},
  {"SLEEP_POSE", 8},
  {"HOME_POSE", 9},
  {"TORQUE_ENABLE", 10},
  {"FLIP_EE_X", 11},
  {"FLIP_EE_ROLL", 12},
  {"SPEED_INC", 13},
  {"SPEED_DEC", 14},
  {"SPEED_COARSE", 15},
  {"SPEED_FINE", 16},
  {"EE_X", 0},             // axes start here
  {"EE_Z", 1},
  {"EE_ROLL", 3},
  {"EE_PITCH", 4}
};

// PS4 Controller button mappings
static const button_mappings ps4 = {
  // {"GRIPPER_PWM_DEC", 0},  // buttons start here  // X
  // {"GRIPPER_RELEASE", 1},                         // O
  // {"GRIPPER_PWM_INC", 2},                         // square
  // {"GRIPPER_GRASP", 3},                           // triangle
  {"WAIST_CW", 4},                                // L1
  // {"EE_Y_DEC", 5},                                // R1
  // {"WAIST_CCW", 6},                               // L2
  // {"WAIST_CW", 7},                                // R2
  {"HOME_POSE", 8},                               // share
  // {"SLEEP_POSE", 9},                              // options
  {"SLEEP_POSE", 10},                             // PS
  {"GRIPPER_CHANGE", 11},                          // L3
  // {"EE_Z_INC", 12},                               // R3
  {"EE_Z", 0},             // axes start here     // left-x left1 right-1
  {"EE_X", 1},                                    // left-y up1 down-1
  {"WAIST_CCW", 2},                               // L2 axis full-1 none1
  // {"EE_ROLL", 3},                                 // right-x left1 right-1
  // {"EE_PITCH", 4},                                // right-y up1 down-1
  // {, 5},                                          // R2 axis full-1 none1
  {"EE_ROLL", 6},                                 // left1 right-1 none0
  {"EE_Y", 7}                                     // up1 down-1 none0
};

// Xbox 360 Controller button mappings
static const button_mappings xbox360 = {
  {"GRIPPER_PWM_DEC", 0},  // buttons start here
  {"GRIPPER_RELEASE", 1},
  {"GRIPPER_GRASP", 2},
  {"GRIPPER_PWM_INC", 3},
  {"WAIST_CCW", 4},
  {"WAIST_CW", 5},
  {"SLEEP_POSE", 6},
  {"HOME_POSE", 7},
  {"TORQUE_ENABLE", 8},
  {"FLIP_EE_X", 9},
  {"FLIP_EE_ROLL", 10},
  {"EE_X", 0},             // axes start here
  {"EE_Z", 1},
  {"EE_Y_INC", 2},
  {"EE_ROLL", 3},
  {"EE_PITCH", 4},
  {"EE_Y_DEC", 5},
  {"SPEED_TYPE", 6},
  {"SPEED", 7}
};

class InterbotixXSArmJoy : public rclcpp::Node
{
public:
  // ROS Subscription to receive raw Joy messages
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_raw;

  // ROS Publisher to publish ArmJoy messages
  rclcpp::Publisher<interbotix_xs_msgs::msg::ArmJoy>::SharedPtr pub_joy_cmd;

  // Keep track of the previously commanded ArmJoy message so that only unique messages are
  // published
  interbotix_xs_msgs::msg::ArmJoy prev_joy_cmd;

  // Holds the controller button mappings
  button_mappings cntlr = ps4;

  // Holds the name of the controller received from the ROS Parameter server
  std::string controller_type;

  // Joystick sensitivity threshold
  double threshold;

  explicit InterbotixXSArmJoy(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("xsarm_joy", options)
  {
    this->declare_parameter("threshold", 0.75);
    this->declare_parameter("controller", "ps4");

    this->get_parameter("threshold", threshold);
    this->get_parameter("controller", controller_type);

    if (controller_type == "xbox360") {
      cntlr = xbox360;
    } else if (controller_type == "ps3") {
      cntlr = ps3;
    } else if (controller_type == "ps4") {
      cntlr = ps4;
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Unsupported controller: '%s'. Defaulting to ps4 button mappings.",
        controller_type.c_str());
    }

    pub_joy_cmd = this->create_publisher<interbotix_xs_msgs::msg::ArmJoy>(
      "commands/joy_processed",
      10);
    sub_joy_raw = this->create_subscription<sensor_msgs::msg::Joy>(
      "commands/joy_raw",
      10,
      [this](sensor_msgs::msg::Joy msg) {joy_state_cb(msg);});
  }

private:
  /// @brief Joystick callback to create custom ArmJoy messages to control the Arm
  /// @param msg - raw sensor_msgs::msg::Joy data
  void joy_state_cb(const sensor_msgs::msg::Joy & msg)
  {
    static bool flip_ee_roll_cmd = false;
    static bool flip_ee_roll_cmd_last_state = false;
    static bool flip_ee_x_cmd = false;
    static bool flip_ee_x_cmd_last_state = false;
    static bool flip_torque_cmd = true;
    static bool flip_torque_cmd_last_state = true;
    static double time_start;
    static bool timer_started = false;
    static bool gripper_open = true;
    static int  last_gripper_bnt = 0;
    interbotix_xs_msgs::msg::ArmJoy joy_cmd;

    // Check the ee_x_cmd
    if (msg.axes.at(cntlr["EE_X"]) >= threshold) {
      joy_cmd.ee_x_cmd = interbotix_xs_msgs::msg::ArmJoy::EE_X_INC;
    } else if (msg.axes.at(cntlr["EE_X"]) <= -threshold) {
      joy_cmd.ee_x_cmd = interbotix_xs_msgs::msg::ArmJoy::EE_X_DEC;
    } 

    // Check the ee_y_cmd
    if (msg.axes.at(cntlr["EE_Y"]) == 1) {
      joy_cmd.ee_y_cmd = interbotix_xs_msgs::msg::ArmJoy::EE_Y_DEC;
    } else if (msg.axes.at(cntlr["EE_Y"]) == -1) {
      joy_cmd.ee_y_cmd = interbotix_xs_msgs::msg::ArmJoy::EE_Y_INC;
    }

    // Check the ee_z_cmd
    if (msg.axes.at(cntlr["EE_Z"]) <= -threshold) {
      joy_cmd.ee_z_cmd = interbotix_xs_msgs::msg::ArmJoy::EE_Z_DEC;
    } else if (msg.axes.at(cntlr["EE_Z"]) >= threshold) {
      joy_cmd.ee_z_cmd = interbotix_xs_msgs::msg::ArmJoy::EE_Z_INC;
    }

    // // Check the ee_pitch_cmd
    // if (msg.axes.at(cntlr["EE_PITCH"]) >= threshold) {
    //   joy_cmd.ee_pitch_cmd = interbotix_xs_msgs::msg::ArmJoy::EE_PITCH_UP;
    // } else if (msg.axes.at(cntlr["EE_PITCH"]) <= -threshold) {
    //   joy_cmd.ee_pitch_cmd = interbotix_xs_msgs::msg::ArmJoy::EE_PITCH_DOWN;
    // }
    
    // Check the ee_roll_cmd
    if (msg.axes.at(cntlr["EE_ROLL"]) == 1) {
      joy_cmd.ee_roll_cmd = interbotix_xs_msgs::msg::ArmJoy::EE_ROLL_CW;
    } else if (msg.axes.at(cntlr["EE_ROLL"]) == -1) {
      joy_cmd.ee_roll_cmd = interbotix_xs_msgs::msg::ArmJoy::EE_ROLL_CCW;
    }

    // Check the waist_cmd
    if (msg.axes.at(cntlr["WAIST_CCW"]) <= -threshold) {
      joy_cmd.waist_cmd = interbotix_xs_msgs::msg::ArmJoy::WAIST_CCW;
    } else if (msg.buttons.at(cntlr["WAIST_CW"]) == 1) {
      joy_cmd.waist_cmd = interbotix_xs_msgs::msg::ArmJoy::WAIST_CW;
    }

    // Check the gripper_cmd
    if (msg.buttons.at(cntlr["GRIPPER_CHANGE"]) == 1 && !last_gripper_bnt && gripper_open) {
      joy_cmd.gripper_cmd = interbotix_xs_msgs::msg::ArmJoy::GRIPPER_GRASP;
      gripper_open = false;
      last_gripper_bnt = 1;
    } else if (msg.buttons.at(cntlr["GRIPPER_CHANGE"]) == 1 && !last_gripper_bnt && !gripper_open) {
      joy_cmd.gripper_cmd = interbotix_xs_msgs::msg::ArmJoy::GRIPPER_RELEASE;
      gripper_open = true;
      last_gripper_bnt = 1;
    } else if (msg.buttons.at(cntlr["GRIPPER_CHANGE"]) == 0 && last_gripper_bnt)
    {
      last_gripper_bnt = 0;
    }
    

    // Check the pose_cmd
    if (msg.buttons.at(cntlr["HOME_POSE"]) == 1) {
      joy_cmd.pose_cmd = interbotix_xs_msgs::msg::ArmJoy::HOME_POSE;
    } else if (msg.buttons.at(cntlr["SLEEP_POSE"]) == 1) {
      joy_cmd.pose_cmd = interbotix_xs_msgs::msg::ArmJoy::SLEEP_POSE;
    }

    // Only publish a ArmJoy message if any of the following fields have changed.
    if (
      !((prev_joy_cmd.ee_x_cmd == joy_cmd.ee_x_cmd) &&
      (prev_joy_cmd.ee_y_cmd == joy_cmd.ee_y_cmd) &&
      (prev_joy_cmd.ee_z_cmd == joy_cmd.ee_z_cmd) &&
      (prev_joy_cmd.ee_roll_cmd == joy_cmd.ee_roll_cmd) &&
      (prev_joy_cmd.ee_pitch_cmd == joy_cmd.ee_pitch_cmd) &&
      (prev_joy_cmd.waist_cmd == joy_cmd.waist_cmd) &&
      (prev_joy_cmd.gripper_cmd == joy_cmd.gripper_cmd) &&
      (prev_joy_cmd.pose_cmd == joy_cmd.pose_cmd) &&
      (prev_joy_cmd.speed_cmd == joy_cmd.speed_cmd) &&
      (prev_joy_cmd.speed_toggle_cmd == joy_cmd.speed_toggle_cmd) &&
      (prev_joy_cmd.gripper_pwm_cmd == joy_cmd.gripper_pwm_cmd) &&
      (prev_joy_cmd.torque_cmd == joy_cmd.torque_cmd)))
    {
      pub_joy_cmd->publish(joy_cmd);
    }
    prev_joy_cmd = joy_cmd;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InterbotixXSArmJoy>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
