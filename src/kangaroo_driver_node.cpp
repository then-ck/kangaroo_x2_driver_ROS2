#include "kangaroo_ros2/kangaroo_driver.hpp"


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<kangaroo>());
    rclcpp::shutdown();
    return 0;
}
