FROM ros:humble-ros-core

# 基本ツール
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-desktop \
    usbutils \
    udev \
    libudev-dev \
    python3-pip \
    vim \
    xterm \
    git \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# GPIO, I2C, PWMなど対応
RUN apt-get update && apt-get install -y \
    python3-rpi.gpio \
    python3-pigpio

# OpenCV（USBカメラ用）
RUN apt-get install -y ros-humble-cv-bridge \
    python3-opencv

# エントリポイントスクリプト
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

