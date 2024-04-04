#include "include/cameras.hpp"
#include <opencv2/opencv.hpp>
#include <memory>
#include <termios.h>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "bupt_rc_cv_interfaces/msg/cv_cameras.hpp"


using namespace std::chrono_literals;

class CameraPublisher : public rclcpp::Node {
public:
    CameraPublisher(cameras& cam) : rclcpp::Node("camera_publisher"), cam_(cam) {
        publisher_ = this->create_publisher<bupt_rc_cv_interfaces::msg::CVCameras>("topic", 10);

        auto timer_callback = [this, &cam]() {
            auto message = bupt_rc_cv_interfaces::msg::CVCameras();        
            message.cam_type = cam.get_cam_type();
            cv::Mat frame = cam.get_frame();
            message.frame_width = frame.cols;
            message.frame_height = frame.rows;
            message.frame_data.assign(frame.data, frame.data + frame.total() * frame.elemSize());
            publisher_->publish(message);
        };

        timer_ = this->create_wall_timer(10ms, timer_callback);
    }

    ~CameraPublisher() {
        std::cout << "camera has been stopped" << std::endl;
        this->cam_.stop();
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
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<bupt_rc_cv_interfaces::msg::CVCameras>::SharedPtr publisher_;
    cameras& cam_;

};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    cameras cam;
    CAMERAS_CHECK(cam.open(), "camera open error");
    CAMERAS_CHECK(cam.start(), "camera start error");
    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "|                    press [q] to exit                    |" << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    CameraPublisher node_cam_publisher(cam);
    node_cam_publisher.spin();

    rclcpp::shutdown();
    return 0;
}
