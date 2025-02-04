#include <rclcpp/rclcpp.hpp>
#include "rclcpp/logging.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <iostream>
#include <type_traits>
//using namespace std;
using namespace std::chrono_literals;

class Patrol : public rclcpp::Node
{
public:
  Patrol() : Node("largest_distance_node")
  {

    callback_group_1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = callback_group_1;

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&Patrol::scanCallback, this, std::placeholders::_1),options1);

    // Create a timer for the control loop (10 Hz)
    control_loop_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), // 100 milliseconds = 10 Hz
        std::bind(&Patrol::control_loop, this),callback_group_2);
  }

private:
   void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg_)
   {

    //obstacle = false;
    // for (size_t i = 0; i < msg_->ranges.size(); ++i) {
    //   float angle = msg_->angle_min + i * msg_->angle_increment;
    //   if (angle > -M_PI / 2 && angle < M_PI / 2 && msg_->ranges[i] < 0.35) {
    //     obstacle = true;
    //     break;
    //   }
    // }
    latest_scan_= msg_;
    RCLCPP_INFO(this->get_logger()," In scanCallback loop ");
    //RCLCPP_INFO(this->get_logger, ...)
   }

  void control_loop()
    {
        // check if distance in front is less than 35

    if (latest_scan_ == nullptr) {
      return; // No scan data yet
    }
    auto msg = latest_scan_;

    // checking if there is a object in the front

        // Approach 1 ................... have 60 degree front laser scan data 

    for (size_t i = 300; i < 420; ++i) {
      //angle = msg->angle_min + i * msg->angle_increment;
      if (msg->ranges[i] < 0.35) {
        //max_index_ = i;
        obstacle = true;
        break;
      }
    }

        // Approach 2 .................. only check with front laser scan ranges{360}

    // if(msg->ranges[360] < 0.35){
    //     obstacle = true;
    
    // }

    RCLCPP_INFO(this->get_logger(),"distance in front: %f",msg->ranges[360]);
    // range_ = (msg->ranges.size())/2;
    // float front_angle =  msg->angle_min + range_ * msg->angle_increment;
    // distance_ = msg->ranges[range_];

    //RCLCPP_INFO(this->get_logger(), "Distance in front: %f at index: %d and angle : %f", msg->ranges[max_index_], max_index_, angle);


    if(obstacle)
    { // else find max distance(not inf) at an angle and move in that direction
        // get max distance

        RCLCPP_INFO(this->get_logger(),"obstacle Detected");

        float max_range = 0.0;
        int max_index = -1;
        

        for (size_t i = 180; i < 540; ++i)
        {
            //float angle_ = msg->angle_min + i * msg->angle_increment;
            // if (angle_ > -M_PI/2 && angle_ < M_PI/2)
            // {

            //     if (msg->ranges[i] > max_range && (!std::isinf(msg->ranges[i])))
            //     {
            //         max_range = msg->ranges[i];
            //         max_index = i;
            //     }
            // }
            if (msg->ranges[i] > max_range && (!std::isinf(msg->ranges[i])))
            {
                max_range = msg->ranges[i];
                max_index = i;
            }


        }

        //RCLCPP_INFO(this->get_logger(), "max range: %f at index: %d", max_range, max_index);

        
        if (max_index != -1){
        // get that direction
        _direction = (msg->angle_min + max_index * msg->angle_increment);
        //move_msg.linear.x = 0.1;
        move_msg.angular.z = _direction/2.0;
        //publisher_->publish(move_msg);
        }
        obstacle  = false;
        //RCLCPP_INFO(this->get_logger(), "Robot is turning towards: %f ",_direction);
    }

    else{
        _direction = 0.0;
       // move_msg.linear.x = 0.1;
        move_msg.angular.z = _direction/2.0;
        //publisher_->publish(move_msg);
        RCLCPP_INFO(this->get_logger(), " Robot Moving straight ");
    }
     move_msg.linear.x = 0.1;
    // move_msg.angular.z = _direction/2.0;
     publisher_->publish(move_msg);
     sleep(0.01);

     //RCLCPP_INFO(this->get_logger(), "Robot is moving towards: %f ",_direction);
   }
    geometry_msgs::msg::Twist move_msg;
    bool obstacle = false;
    int range_;
    float distance_;
    float _direction = 0.0;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    rclcpp::CallbackGroup::SharedPtr callback_group_1;
    rclcpp::CallbackGroup::SharedPtr callback_group_2;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<Patrol> robot_patrol_node = std::make_shared<Patrol>();
  //rclcpp::spin(std::make_shared<Patrol>());

  // trying with multi threaded approach
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(robot_patrol_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}