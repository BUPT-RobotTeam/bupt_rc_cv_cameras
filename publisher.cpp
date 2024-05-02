#include <opencv2/opencv.hpp>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <termio.h>
#include <chrono>
#include "cameras.h"
#include "rclcpp/rclcpp.hpp"
#include "bupt_rc_cv_interfaces/srv/cv_depth.hpp"
#include "bupt_rc_cv_interfaces/msg/cv_camera_array.hpp"
#include <thread>


//------------------------------default configuration------------------------------

std::string config_path = "/home/bupt-rc/ros2_ws/bupt_rc_cv_ws/src/bupt_rc_cv_cameras/config.yaml";
bool capture_frame_stop = false;

using namespace std::chrono_literals;

class Publisher : public rclcpp :: Node {
public:
    Publisher(bupt_rc_cv_interfaces::msg::CVCamera* cameras[]) : rclcpp::Node("camera_publisher"){                                                      
        for (int i = 0; i < 2; ++i) {
            this->cameras_[i] = cameras[i];
        }

        this->publisher_ = this->create_publisher<bupt_rc_cv_interfaces::msg::CVCameraArray>("bupt_rc_cv/cameras", 1);
        this->timer_ = this->create_wall_timer(10ms, std::bind(&Publisher::timer_callback, this));
    }

    ~Publisher() {
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
                    capture_frame_stop = true;
                    break;  // 按下 'q' 键退出循环
                }
            }
        }
        tcsetattr(STDIN_FILENO, TCSANOW, &old_settings);
    }
private:
    void timer_callback() {
        auto cameras_array = bupt_rc_cv_interfaces::msg::CVCameraArray();
        for (int i = 0; i < 2; ++i) {
            cameras_array.cameras.push_back(*this->cameras_[i]);
        }
        this->publisher_->publish(cameras_array);
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
    bupt_rc_cv_interfaces::msg::CVCamera* cameras_[2];

};

void publisher_thread_func(bupt_rc_cv_interfaces::msg::CVCamera* cameras[]) {
    Publisher camera_publish_node(cameras);
    camera_publish_node.spin();
    rclcpp::shutdown();
}

void camera_thread_func(bupt_rc_cv_interfaces::msg::CVCamera* message) {
    cameras cam(config_path);
    CAMERAS_CHECK(cam.open(), cam.get_cam_name() + " open fail");
    CAMERAS_CHECK(cam.start(), cam.get_cam_name() + " start fail");

    while (true) {
        if (capture_frame_stop)
            break;
        cv::Mat frame = cam.get_frame();
        if (frame.empty())
            continue;

        // 装填数据
        message->img.frame_data.assign(frame.data, frame.data + frame.total() * frame.elemSize());
        message->img.frame_width = frame.cols;
        message->img.frame_height = frame.rows;
        message->cam_fps = cam.get_fps();
        message->cam_name = cam.get_cam_name();
        message->cam_type = cam.get_cam_type();
    }
    cam.stop();
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    const int camera_num = 2;
    std::thread threads[camera_num + 1];
    cv::Mat frame[camera_num];
    bupt_rc_cv_interfaces::msg::CVCamera message[2];
    bupt_rc_cv_interfaces::msg::CVCamera* message_ptr[2];
    

    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "|                    press [q] to exit                    |" << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;


    try {
        for (int i = 0; i < camera_num; ++i) {
            threads[i] = std::thread(camera_thread_func, &message[i]);
            message_ptr[i] = &message[i];
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
        threads[camera_num] = std::thread(publisher_thread_func, message_ptr);

        for (int i = 0; i < camera_num + 1; ++i) {
            threads[i].join();
        }

    }
    catch (const std::exception& e) {
        std::cout << "[BUPT_RC]Exception caught: " << e.what() << std::endl;
    }
    return 0;
}
