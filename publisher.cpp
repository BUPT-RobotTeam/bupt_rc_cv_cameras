#include <opencv2/opencv.hpp>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <termio.h>
#include "cameras.h"
#include "rclcpp/rclcpp.hpp"
#include "bupt_rc_cv_interfaces/srv/cv_depth.hpp"
#include "bupt_rc_cv_interfaces/msg/cv_camera_array.hpp"


//------------------------------default configuration------------------------------

std::string config_path = "/home/bupt-rc/ros2_ws/bupt_rc_cv_ws/src/bupt_rc_cv_cameras/config.yaml";


using namespace std::chrono_literals;

class CamerasPublisher : public rclcpp::Node {
public:
    CamerasPublisher(cameras* cam_01, cameras* cam_02) : rclcpp::Node("camera_publisher") {                                                                                    
        this->publisher_ = this->create_publisher<bupt_rc_cv_interfaces::msg::CVCameraArray>("bupt_rc_cv/cameras", 1);
        this->cam_01_ = cam_01;
        this->cam_02_ = cam_02;
        this->timer_ = this->create_wall_timer(10ms, std::bind(&CamerasPublisher::timer_callback, this));
    }

    ~CamerasPublisher() {
        this->cam_01_->stop();
        this->cam_02_->stop();
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
    void timer_callback() {
        auto message = bupt_rc_cv_interfaces::msg::CVCameraArray();

        auto message_cam_01 = bupt_rc_cv_interfaces::msg::CVCamera();
        auto message_cam_02 = bupt_rc_cv_interfaces::msg::CVCamera();

        cv::Mat frame_01 = this->cam_01_->get_frame();
        cv::Mat frame_02 = this->cam_02_->get_frame();
        
        message_cam_01.img.frame_data.assign(frame_01.data, frame_01.data + frame_01.total() * frame_01.elemSize());
        message_cam_01.img.frame_width = frame_01.cols;
        message_cam_01.img.frame_height = frame_01.rows;
        message_cam_01.cam_fps = 30.0;
        message_cam_01.cam_name = this->cam_01_->get_cam_name();
        message_cam_01.cam_type = this->cam_01_->get_cam_type();

        message_cam_02.img.frame_data.assign(frame_02.data, frame_02.data + frame_02.total() * frame_02.elemSize());
        message_cam_02.img.frame_width = frame_02.cols;
        message_cam_02.img.frame_height = frame_02.rows;
        message_cam_02.cam_fps = 30.0;
        message_cam_02.cam_name = this->cam_02_->get_cam_name();
        message_cam_02.cam_type = this->cam_02_->get_cam_type();
        
        message.cameras.push_back(message_cam_01);
        message.cameras.push_back(message_cam_02);

        this->publisher_->publish(message);
    }

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
    rclcpp::Publisher<bupt_rc_cv_interfaces::msg::CVCameraArray>::SharedPtr publisher_;
    cameras* cam_01_;
    cameras* cam_02_;
};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "|                    press [q] to exit                    |" << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;


    // 打开和开启相机
    try {
        cameras cam_01(config_path);
        CAMERAS_CHECK(cam_01.open(), cam_01.get_cam_name() + " open fail");
        CAMERAS_CHECK(cam_01.start(), cam_01.get_cam_name() + "start fail");

        cameras cam_02(config_path);
        CAMERAS_CHECK(cam_02.open(), cam_02.get_cam_name() + " open fail");
        CAMERAS_CHECK(cam_02.start(), cam_02.get_cam_name() + "start fail");
        CamerasPublisher node_cams_publisher(&cam_01, &cam_02);

        node_cams_publisher.spin();
        rclcpp::shutdown();
    }
    catch (const std::exception& e) {
        std::cout << "[BUPT_RC]Exception caught: " << e.what() << std::endl;
    }
    return 0;
}
