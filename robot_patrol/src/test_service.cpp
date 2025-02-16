// test_service.cpp
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "direction_custom_interface/srv/get_direction.hpp"
#include <cmath>
#include <iostream>
#include <type_traits>

using namespace std;
using MyCustomService = direction_custom_interface::srv::GetDirection;

class TestService : public rclcpp::Node
{
public:
  TestService() : Node("test_service")
  {
    client_ = this->create_client<MyCustomService>("direction_service");
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&TestService::scan_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Service Client Ready");
    
  }
  bool service_complete = false;
  bool service_called = false;

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // while (!client_->wait_for_service(1s)) {
    //   RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    // }

    auto request = std::make_shared<MyCustomService::Request>();
    request->laser_data = *msg; // Copy the laser scan data
    if (!service_called){
    auto result_future = client_->async_send_request(request, std::bind(&TestService::response_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Client Requested Service");
    service_called = true;
    }
    // if (result_future.wait_for(1s) == std::future_status::ready) {
    //   auto response = result_future.get();
    //   RCLCPP_INFO(this->get_logger(), "Direction: %s", response->direction.c_str());
    // } else {
    //   RCLCPP_INFO(this->get_logger(), "Service call timed out");
    // }
  }

    void response_callback(rclcpp::Client<MyCustomService>::SharedFuture future)
  {
    auto result = future.get();
    RCLCPP_INFO(this->get_logger(), "Service Response receieved :- direction: %s", result->direction.c_str());
    this->service_complete = true;
  }

  rclcpp::Client<MyCustomService>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  //bool service_complete = false;

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestService>();
  while (!node->service_complete){
  rclcpp::spin_some(node);
  }
  rclcpp::shutdown();
  return 0;
}
