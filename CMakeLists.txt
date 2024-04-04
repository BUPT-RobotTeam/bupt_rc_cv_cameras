cmake_minimum_required(VERSION 3.8)
project(bupt_rc_cv_cameras)

# 基本设置
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(CMAKE_CXX_STANDARD 20)
set(MVCAM_COMMON_RUNENV /opt/MVS/lib)
set(MVCAM_COMMON_PATH /opt/MVS)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(bupt_rc_cv_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${MVCAM_COMMON_PATH}/include
)


# 发包的程序
add_executable(cameras_publisher
    publisher.cpp 
    ${PROJECT_SOURCE_DIR}/include/hikcam.hpp 
    ${PROJECT_SOURCE_DIR}/include/cameras.hpp
)

add_executable(cameras_subscriber
    subscriber.cpp
)


# publisher 库链接
target_link_libraries(cameras_publisher
    MvCameraControl 
    ${OpenCV_LIBS} 
    ${realsense2_LIBRARY}
    # ${MVCAM_COMMON_RUNENV}/aarch64
)

ament_target_dependencies(cameras_publisher rclcpp bupt_rc_cv_interfaces)

target_link_libraries(cameras_subscriber
    ${OpenCV_LIBS}
)
ament_target_dependencies(cameras_subscriber rclcpp bupt_rc_cv_interfaces)


install(TARGETS cameras_publisher
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS cameras_subscriber
    DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
