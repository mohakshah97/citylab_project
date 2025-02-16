
#include <functional>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/detail/pose2_d__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// #include "direction_custom_interface/action/go_to_pose.hpp"
#include "robot_patrol/action/go_to_pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <cmath>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class GoToPose : public rclcpp::Node {
public:
  using Move = robot_patrol::action::GoToPose; // change it to
                                                             // custom message
  using GoalHandleMove =
      rclcpp_action::ServerGoalHandle<Move>; // change to something instead of
                                             // MOVE

  explicit GoToPose(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("action_server", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Move>(
        this, "go_to_pose", std::bind(&GoToPose::handle_goal, this, _1, _2),
        std::bind(&GoToPose::handle_cancel, this, _1),
        std::bind(&GoToPose::handle_accepted, this, _1));

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // implementing subscriber to get the odometry data
    callback_group_1 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options1;
    options1.callback_group = callback_group_1;
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&GoToPose::OdomCallback, this, std::placeholders::_1),
        options1);

    RCLCPP_INFO(this->get_logger(), "Action Server Ready");
  }

private:
  rclcpp_action::Server<Move>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  geometry_msgs::msg::Pose2D current_pose;

  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    double theta = yaw;

    this->current_pose.x = x;
    this->current_pose.y = y;
    this->current_pose.theta = -(180 / M_PI) * theta;
    // current_

    // RCLCPP_INFO(this->get_logger(), "Position 2D: x=%.2f, y=%.2f,
    // theta=%.2f", x, y, yaw);
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Move::Goal> goal) {
    RCLCPP_INFO(this->get_logger(),
                "Received Action Call request with x=%.2f, y=%.2f, theta=%.2f",
                goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();

    auto feedback = std::make_shared<Move::Feedback>(); // change message type

    auto result = std::make_shared<Move::Result>(); // change message type
    auto move = geometry_msgs::msg::Twist();
    rclcpp::Rate loop_rate(1);
    float error_x = goal->goal_pos.x - current_pose.x;
    float error_y = goal->goal_pos.y - current_pose.y;
    float error_theta = goal->goal_pos.theta - current_pose.theta;
    float angle_to_goal = -(180/M_PI)*atan2(error_y, error_x);
    bool final_orientation = false;
    // implement a logic where we can get the odometry data and move based on
    // that
    while (!(abs(error_x) < 0.1 && abs(error_y) < 0.1 && abs(error_theta) < 1)) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->status = true;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      // float x_diff =

    //   while (!(abs(error_x) < 0.1 && abs(error_y) < 0.1)){



    //   if (error_x > 0) {
    //     if (error_y < 0) {
    //       move.linear.x = 0.1;
    //       move.angular.z = -0.1;
    //     }

    //     else {
    //       move.linear.x = 0.1;
    //       move.angular.z = 0.1;
    //     }
    //   }

    //   else {
    //     if (error_y > 0) {
    //       move.linear.x = -0.1;
    //       move.angular.z = 0.1;
    //     } else {
    //       move.linear.x = 0.1;
    //       move.angular.z = -0.1;
    //     }
    //   }

    //   publisher_->publish(move);
    //   feedback->current_pos = this->current_pose;
    //   goal_handle->publish_feedback(feedback);
    //   loop_rate.sleep();

    //   }


    //   if (std::abs(goal->goal_pos.x - current_pose.x) > 0.1)
    //   { if ((goal->goal_pos.x - current_pose.x) > 0){move.linear.x = 0.1;}
    //   else{move.linear.x = -0.1;}
    //   }
    //   else{move.linear.x = 0.0;}
      //move_x = false;}

    //   if (std::abs(goal->goal_pos.y - current_pose.y) > 0.1)
    //   { if ((goal->goal_pos.y - current_pose.y) > 0){move.linear.y = 0.1;}
    //   else{move.linear.y = -0.1;}
    //   }
    //   else{move.linear.y = 0.0;
    //   move_y = false;}

    
      if (std::abs(current_pose.theta - angle_to_goal) > 1) { 

        move.angular.z = -0.1;
        move.linear.x =0.0;
        if (std::abs(current_pose.theta - angle_to_goal) < 12)
        {
            move.angular.z = -0.01;
            move.linear.x =0.0;
        }
      }
      else 
      {
        if (abs(error_x ) < 0.1 && abs(error_y ) < 0.1 ){
         move.linear.x = 0.0;
         move.angular.z = 0.0;
         final_orientation = true;
        //  if(std::abs(current_pose.theta - angle_to_goal)< 1)
        //  {publisher_->publish(move);
        //  break;}
        }
      else{
       move.linear.x = 0.1;
       move.angular.z = 0.0;}
      //else{move.linear.x = -0.1;}
      }
      if(final_orientation){

      if (error_theta > 0){
      
        move.angular.z = -0.1;
        move.linear.x =0.0;
        if (std::abs(error_theta) < 12)
        {
            move.angular.z = -0.01;
            move.linear.x =0.0;
        }
      }
      else {
        move.angular.z = 0.1;
        move.linear.x =0.0;

        if (std::abs(error_theta) < 12)
        {
            move.angular.z = 0.01;
            move.linear.x =0.0;
        }
      }

      }


    //   while (!(abs(error_theta) < 3))
    //   {
      
    //     if(error_theta > 0){
    //         move.angular.z = 0.1;
    //     }
    //     else{
    //         move.angular.z = -0.1;
    //     }
      
    //   }



      publisher_->publish(move);
      // publishing feedback

      loop_rate.sleep();

      feedback->current_pos = this->current_pose;
      goal_handle->publish_feedback(feedback);
      error_x = goal->goal_pos.x - current_pose.x;
      error_y = goal->goal_pos.y - current_pose.y;
      error_theta = goal->goal_pos.theta - current_pose.theta;
      RCLCPP_INFO(this->get_logger(),
                  "Goal velocity 2D: x=%.2f, theta=%.2f , error_theta =%.2f, angle to goal=%.2f", move.linear.x, move.angular.z, error_theta, angle_to_goal);
      
    }

    // while(abs(error_theta) < 3)
    // {
    // if (error_theta > 0){
    //   move.angular.z = 0.1; }
    //   else{
    //   move.linear.z = -0.1;
      
    //   }
    // }



    // result->status = true;
    // goal_handle->succeed(result);

    // for (int i = 0; (i < goal->secs) && rclcpp::ok(); ++i) {
    //   // Check if there is a cancel request
    //   if (goal_handle->is_canceling()) {
    //     result->status = message;
    //     goal_handle->canceled(result);
    //     RCLCPP_INFO(this->get_logger(), "Goal canceled");
    //     return;
    //   }
    //   // Move robot forward and send feedback
    //   message = "Moving forward...";
    //   move.linear.x = 0.3;
    //   publisher_->publish(move);
    //   goal_handle->publish_feedback(feedback);
    //   RCLCPP_INFO(this->get_logger(), "Publish feedback");

    //   loop_rate.sleep();
    // }

    // Check if goal is done
    if (rclcpp::ok()) {
      // result->status = "Finished action server. Robot reached required
      // location";
      result->status = true;
      move.linear.x = 0.0;
      move.linear.y = 0.0;
      move.angular.z = 0.0;
      publisher_->publish(move);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Action Completed");
    }
  }
}; // class MyActionServer

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<GoToPose>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}