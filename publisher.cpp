#include "include/cameras.hpp"
#include <opencv2/opencv.hpp>
#include <memory>
#include <termios.h>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "bupt_rc_cv_interfaces/msg/cv_cameras.hpp"
#include "bupt_rc_cv_interfaces/srv/cv_depth.hpp"


using namespace std::chrono_literals;

class CameraPublisher : public rclcpp::Node {
public:
    CameraPublisher(cameras& cam) : rclcpp::Node("camera_publisher"), cam_(cam) {
        publisher_ = this->create_publisher<bupt_rc_cv_interfaces::msg::CVCameras>("bupt_rc_cv/cameras", 1);



        // 时间回调函数，用于定时发送公共信息
        auto timer_callback = [this, &cam]() {
            auto message = bupt_rc_cv_interfaces::msg::CVCameras();        

            message.cam_type = cam.get_cam_type();                          // 获取相机的类型
            cv::Mat frame = cam.get_frame();                                // 获取该帧数据
            message.img.frame_width = frame.cols;                               // 获取数据帧的宽
            message.img.frame_height = frame.rows;                              // 获取数据帧的高
            message.cam_fps = cam.get_fps();                                // 获取相机的帧数
            // 装载图片数据
            message.img.frame_data.assign(frame.data, frame.data + frame.total() * frame.elemSize()); 
            // 发布
            publisher_->publish(message);
        };


        // depth回调函数，用于返回depth信息
        auto depth_callback = [this, &cam](const std::shared_ptr<bupt_rc_cv_interfaces::srv::CVDepth::Request> request, 
                std::shared_ptr<bupt_rc_cv_interfaces::srv::CVDepth::Response> response) {
            response->depth = cam.get_depth(request->x, request->y);
        };

        // 每10s发送一次
        timer_ = this->create_wall_timer(20ms, timer_callback);
        service_ = this->create_service<bupt_rc_cv_interfaces::srv::CVDepth>("bupt_rc_cv/cameras/depth", depth_callback);
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
    rclcpp::Service<bupt_rc_cv_interfaces::srv::CVDepth>::SharedPtr service_;
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
