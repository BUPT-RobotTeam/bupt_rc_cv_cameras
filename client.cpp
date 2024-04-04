#include "rclcpp/rclcpp.hpp"
#include "bupt_rc_cv_interfaces/srv/cv_depth.hpp"
#include <memory>
#include <chrono>

using namespace std::chrono_literals;
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    if (argc != 3) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: get_depth x y");
        return 1;
    }
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("get_depth");
    rclcpp::Client<bupt_rc_cv_interfaces::srv::CVDepth>::SharedPtr client = node->create_client<bupt_rc_cv_interfaces::srv::CVDepth>("bupt_rc_cv/cameras/depth");

    auto request = std::make_shared<bupt_rc_cv_interfaces::srv::CVDepth::Request>();
    request->x = atoll(argv[1]);
    request->y = atoll(argv[2]);

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Depth: %lf", result.get()->depth);
    }
    else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get_depth");
    }
    
    rclcpp::shutdown();
}


