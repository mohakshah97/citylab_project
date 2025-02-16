#include <rclcpp/rclcpp.hpp>
#include "rclcpp/logging.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <iostream>
#include <type_traits>
#include <string>
// #include "direction_custom_interface/srv/get_direction.hpp"
#include "robot_patrol/srv/get_direction.hpp"

//using namespace std;
using namespace std::chrono_literals;
using MyCustomService = robot_patrol::srv::GetDirection;

class Patrol : public rclcpp::Node
{
public:
  Patrol() : Node("largest_distance_node")
  {

    callback_group_1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_3 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = callback_group_1;

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&Patrol::scanCallback, this, std::placeholders::_1),options1);

    // Create a timer for the control loop (10 Hz)
    control_loop_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), // 100 milliseconds = 10 Hz
        std::bind(&Patrol::control_loop, this),callback_group_2);

    // client service direction_service
    client = this->create_client<MyCustomService>("direction_service");
  }

private:
   void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg_)
   {

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


    //RCLCPP_INFO(this->get_logger(),"distance in front: %f",msg->ranges[360]);
    // range_ = (msg->ranges.size())/2;
    // float front_angle =  msg->angle_min + range_ * msg->angle_increment;
    // distance_ = msg->ranges[range_];

    //RCLCPP_INFO(this->get_logger(), "Distance in front: %f at index: %d and angle : %f", msg->ranges[max_index_], max_index_, angle);


    //if(msg->ranges[360] < 0.35)
    if(obstacle)
    { 
        RCLCPP_INFO(this->get_logger(),"obstacle Detected");

    //     while (!client->wait_for_service(1s)) {
    //     RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    //   }

    // call the direction service and get the responsee

        auto request = std::make_shared<MyCustomService::Request>();
        request->laser_data = *msg;
        auto result_future = client->async_send_request(
        request, std::bind(&Patrol::response_callback, this,
                           std::placeholders::_1));
        //auto result_future = client->async_send_request(request);

        

        // Now check for the response after a timeout of 1 second
        // auto status = result_future.wait_for(1s);

        // if (status != std::future_status::ready) {

        // RCLCPP_WARN(this->get_logger(), "Response not ready yet.");
        // }

          // Wait for the result.

        /* publishing inside the timer callback
        ..................................

        if (result_future.wait_for(1s) == std::future_status::ready){
        auto response = result_future.get();

        RCLCPP_INFO(this->get_logger(), "Direction from service: %s", response->direction.c_str());

        if (response->direction == "Right")
        {
        move_msg.linear.x = 0.1;
        move_msg.angular.z = -0.5;
        }
        else if(response->direction == "Forward")
        {
        move_msg.linear.x = 0.1;
        move_msg.angular.z = 0.0;
        
        }
        else if(response->direction == "Left"){
        move_msg.linear.x = 0.1;
        move_msg.angular.z = 0.5;
        
        }
        //publisher_->publish(move_msg);
        obstacle = false;
        }
        else {

        RCLCPP_INFO(this->get_logger(), "Service call timed out");
        move_msg.linear.x = 0.1;
        move_msg.angular.z = 0.0;
        }
    
        */
  

        // float max_range = 0.0;
        // int max_index = -1;
        

        // for (size_t i = 180; i < 540; ++i)
        // {

        //     if (msg->ranges[i] > max_range && (!std::isinf(msg->ranges[i])))
        //     {
        //         max_range = msg->ranges[i];
        //         max_index = i;
        //     }


        // }

        
        // if (max_index != -1){
        // // get that direction
        // _direction = (msg->angle_min + max_index * msg->angle_increment);
        // //move_msg.linear.x = 0.1;
        // move_msg.angular.z = _direction/2.0;
        // //publisher_->publish(move_msg);
        // }
        //obstacle  = false; // closely observe


        //RCLCPP_INFO(this->get_logger(), "Robot is turning towards: %f ",_direction);
    }

    else{
        
        move_msg.linear.x = 0.1;
        move_msg.angular.z = 0.0;
        //publisher_->publish(move_msg);
        RCLCPP_INFO(this->get_logger(), " Robot Moving straight ");
    }


    // move_msg.linear.x = 0.1;
    // move_msg.angular.z = _direction/2.0;
     publisher_->publish(move_msg);
     //sleep(0.01);

     //RCLCPP_INFO(this->get_logger(), "Robot is moving towards: %f ",_direction);
   }

    void response_callback(rclcpp::Client<MyCustomService>::SharedFuture future) {
    // Get response value
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Response: %s",response->direction.c_str());
    //service_done_ = true;
    if (response->direction == "Right")
    {
    move_msg.linear.x = 0.1;
    move_msg.angular.z = -0.5;
    }
    else if(response->direction == "Front")
    {
    move_msg.linear.x = 0.1;
    move_msg.angular.z = 0.0;
    
    }
    else{
    move_msg.linear.x = 0.1;
    move_msg.angular.z = 0.5;
    
    }
    publisher_->publish(move_msg);
    obstacle = false;

  }
    geometry_msgs::msg::Twist move_msg;
    bool obstacle = false;
    int range_;
    float distance_;
    //float _direction = 0.0;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Client<MyCustomService>::SharedPtr client;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    rclcpp::CallbackGroup::SharedPtr callback_group_1;
    rclcpp::CallbackGroup::SharedPtr callback_group_2;
    rclcpp::CallbackGroup::SharedPtr callback_group_3;
    
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