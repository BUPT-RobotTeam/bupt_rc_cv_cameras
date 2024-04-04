#include "rclcpp/rclcpp.hpp"
#include "bupt_rc_cv_interfaces/msg/cv_cameras.hpp"
#include "bupt_rc_cv_interfaces/srv/cv_depth.hpp"
#include <opencv2/opencv.hpp>
#include <termios.h>
#include <unistd.h>


class CamerasSubscriber : public rclcpp::Node {
     
public:
    CamerasSubscriber() : Node("cameras_subscriber") {
        //------------------------------订阅图像数据------------------------------
        cv::namedWindow("Cam", cv::WINDOW_NORMAL);
        auto topic_callback = [] (const bupt_rc_cv_interfaces::msg::CVCameras::SharedPtr msg){
            // 加载图像数据
            cv::Mat frame(msg->frame_height, msg->frame_width, CV_8UC3, msg->frame_data.data());
            // 显示帧数
            cv::putText(frame, "FPS: " + std::to_string(msg->cam_fps), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
            cv::imshow("Cam", frame);
            cv::waitKey(1);
        };
        subscription_ = this->create_subscription<bupt_rc_cv_interfaces::msg::CVCameras>("bupt_rc_cv/cameras", 10, topic_callback);
    }


    ~CamerasSubscriber() {
        std::cout << "All windows has been destroyed" << std::endl;
        cv::destroyAllWindows();
    }

    void spin(){
        int key;
        struct termios old_settings, new_settings;

        tcgetattr(STDIN_FILENO, &old_settings);
        new_settings = old_settings;
        new_settings.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);

        while (rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());
            if (kbhit()) {
                key = getchar();
                if (key == 'q') {
                    break;  // 按下 'q' 键退出循环
                }
            }
        }
        tcsetattr(STDIN_FILENO, TCSANOW, &old_settings);
    }
private:
    int kbhit(){
        struct timeval tv;
        fd_set read_fd;

        tv.tv_sec = 0;
        tv.tv_usec = 0;
        FD_ZERO(&read_fd);
        FD_SET(STDIN_FILENO, &read_fd);

        if (select(STDIN_FILENO + 1, &read_fd, nullptr, nullptr, &tv) == -1) {
          return 0;
        }

        return FD_ISSET(STDIN_FILENO, &read_fd);
    }

    void send_depth_request() {
        auto request = std::make_shared<bupt_rc_cv_interfaces::srv::CVDepth::Request>();
        request->x = 300;
        request->y = 300;

        // 发送Service请求
        auto future = client_->async_send_request(request);

        // 等待响应
        if (rclcpp::spin_until_future_complete(shared_from_this(), future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            RCLCPP_INFO(get_logger(), "Received respose: %lf", response->depth);
        }
        else {
            RCLCPP_ERROR(get_logger(), "Failed to receive response.");
        }
    }
private:
    rclcpp::Subscription<bupt_rc_cv_interfaces::msg::CVCameras>::SharedPtr subscription_;
    rclcpp::Client<bupt_rc_cv_interfaces::srv::CVDepth>::SharedPtr client_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    CamerasSubscriber node_cam_subscriber;
    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "|                    press [q] to exit                    |" << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    node_cam_subscriber.spin();
    rclcpp::shutdown();
    return 0;
}
