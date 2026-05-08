    #include "g1_control/standing_controller.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StandingController>());
    rclcpp::shutdown();
    return 0;
}