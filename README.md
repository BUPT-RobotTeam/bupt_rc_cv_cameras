# cameras 队库的 ROS包

> 你可以随意更改代码然后push到仓库中
>
> 文章写于: 2024年4月5日

## 效果演示
https://github.com/BUPT-RobotTeam/bupt_rc_cv_cameras/assets/129849375/a7749d84-3244-43c6-bd38-c2ad17854f86

## 环境要求
> 1. OpenCV
> 2. realsense2 SDK
> 3. MvCameraControl SDK
> 4. bupt_rc_cv_interfaces
> 5. ROS2 (作者使用的平台是humble)

## 你可能需要修改
```cmake
# 你可能需要手动修改以下两个路径
set(MVCAM_COMMON_RUNENV /opt/MVS/lib)
set(MVCAM_COMMON_PATH /opt/MVS)
```

## 效果说明
> 目的: 为了将cameras类与项目使用的ROS相结合
>
> 效果：主动发布图像数据(包括：相机类型、数据帧高度、数据帧长度、数据帧、相机帧数)
>
>       外部请求时候发送深度数据（非深度摄像机返回的数据为0.0）

## 话题说明
1. 图像话题发布于：bupt_rc_cv/cameras
2. 深度信息的Service为: bupt_rc_cv/cameras/depth
