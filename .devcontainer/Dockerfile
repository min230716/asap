FROM ros:humble

# Gazebo & ROS 2 관련 패키지 설치
RUN apt update && apt install -y \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    && rm -rf /var/lib/apt/lists/*

# 작업 디렉토리 설정
WORKDIR /ros2_ws
