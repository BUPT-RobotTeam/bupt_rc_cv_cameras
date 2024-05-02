#include "rclcpp/rclcpp.hpp"
#include "bupt_rc_cv_interfaces/srv/cv_depth.hpp"
#include "bupt_rc_cv_interfaces/msg/cv_camera_array.hpp"
#include "bupt_rc_cv_interfaces/msg/cv_camera.hpp"
#include <opencv2/opencv.hpp>
#include <termios.h>
#include <unistd.h>


class CamerasSubscriber : public rclcpp::Node {
     
public:
    CamerasSubscriber() : Node("cameras_subscriber") {
        //------------------------------订阅图像数据------------------------------
        auto topic_callback = [] (const bupt_rc_cv_interfaces::msg::CVCameraArray::SharedPtr msg){
            for (auto camera : msg->cameras) {
                cv::Mat frame(camera.img.frame_height, camera.img.frame_width, CV_8UC3, camera.img.frame_data.data());
                cv::putText(frame, "FPS: " + std::to_string(camera.cam_fps), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
                cv::imshow(camera.cam_name, frame);
                cv::waitKey(1);
            }
        };

        subscription_ = this->create_subscription<bupt_rc_cv_interfaces::msg::CVCameraArray>("bupt_rc_cv/cameras", 1, topic_callback);
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

private:
    rclcpp::Subscription<bupt_rc_cv_interfaces::msg::CVCameraArray>::SharedPtr subscription_;
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
