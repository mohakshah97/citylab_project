
#include "rclcpp/rclcpp.hpp"
#include "direction_custom_interface/srv/get_direction.hpp"
#include "rclcpp/service.hpp"
#include <cstddef>
#include <functional>
#include <memory>
#include <iostream>
#include <string>


using std::placeholders::_1;
using std::placeholders::_2;

using MyCustomService = direction_custom_interface::srv::GetDirection;

class DirectionService : public rclcpp::Node
{
public: 
DirectionService():Node("service_moving")
{
srv_ = create_service<MyCustomService>("direction_service", std::bind(&DirectionService::direction_callback,this,_1,_2));
RCLCPP_INFO(this->get_logger(), "Service Server Ready");
}

private :
rclcpp::Service<MyCustomService>::SharedPtr srv_;
float total_dist_sec_right = 0.0;
float total_dist_sec_front = 0.0;
float total_dist_sec_left = 0.0;
float max_distance_direction;
std::string direction_;

void direction_callback(const std::shared_ptr<MyCustomService::Request> request,
const std::shared_ptr<MyCustomService::Response> response)
{
RCLCPP_INFO(this->get_logger(), "Service Requested");
if (request == nullptr){
    return;// laser data is empty
}
// Now calcualte the best direction by adding distance in each 60 degree angular piece
total_dist_sec_right = 0.0;
total_dist_sec_front = 0.0;
total_dist_sec_left = 0.0;
max_distance_direction = 0;

for (size_t i = 180; i < 300; i++){
if (!std::isinf(request->laser_data.ranges[i])){
// check if safest direction is right 
total_dist_sec_right += request->laser_data.ranges[i];
}
}


for (size_t i = 300; i < 420; i++){
// check if safest direction is front 
if (!std::isinf(request->laser_data.ranges[i])){
total_dist_sec_front += request->laser_data.ranges[i];
}   
}

if ( total_dist_sec_right > total_dist_sec_front){
max_distance_direction = total_dist_sec_right;
direction_ = "Right";}
else{
direction_ = "Forward";
max_distance_direction = total_dist_sec_front;
}
for (size_t i = 420; i < 540; i++){
// check if safest direction is left 
if (!std::isinf(request->laser_data.ranges[i])){
total_dist_sec_left += request->laser_data.ranges[i];
}
}

if (total_dist_sec_left > max_distance_direction)
{
max_distance_direction = total_dist_sec_left;
direction_ = "Left";
}

response->direction = direction_;

RCLCPP_INFO(this->get_logger(), "Service Completed");
// RCLCPP_INFO(this->get_logger(), "Distance_Right: %f, Distance_Front: %f, Distance_Left: %f", total_dist_sec_right, total_dist_sec_front, total_dist_sec_left);
 RCLCPP_INFO(this->get_logger(), "Direction: %s because max_distance: %f", response->direction.c_str(), max_distance_direction);

}


};

int main(int argc, char*argv[])
{
rclcpp::init(argc,argv);
rclcpp::spin(std::make_shared<DirectionService>());
rclcpp::shutdown();
return 0;


}