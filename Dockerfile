# =============================================================================
# @file Dockerfile
# @brief Taiphoon_Agent 多階段 Docker 建構檔案
# @author Taiphoon Agent Team
# @date 2026-07-23
#
# Stage 階層架構:
#   taiphoon-base        共用基礎 (locale)
#     └── taiphoon-ros2   共用 ROS2 repo 設定
#           ├── taiphoon-builder   完整編譯環境 (Desktop + dev-tools)
#           │     └── taiphoon-dev       開發映像 (+ entrypoint)
#           └── taiphoon-runtime  精簡執行環境 (ros-base)
#
# 參考來源:
#   https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
#
# 使用方式:
#   docker build --target taiphoon-dev -t taiphoon-agent:dev .
#   docker build --target taiphoon-runtime -t taiphoon-agent:runtime .
# =============================================================================

# =============================================================================
# Stage 0: 共用基礎層 - locale
# =============================================================================
FROM ubuntu:22.04 AS taiphoon-base

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Taipei

RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    tzdata \
    && locale-gen en_US en_US.UTF-8 \
    && locale-gen zh_TW zh_TW.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# =============================================================================
# Stage 1: 共用 ROS2 repo 層 - 只做一次，builder 和 runtime 都從這裡繼承
# =============================================================================
FROM taiphoon-base AS taiphoon-ros2

# ---- 啟用 Ubuntu Universe 倉庫 ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    curl \
    gnupg \
    lsb-release \
    && add-apt-repository universe \
    && rm -rf /var/lib/apt/lists/*

# ---- 加入 ROS2 GPG key ----
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# ---- 加入 ROS2 倉庫 ----
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# =============================================================================
# Stage 2: 完整建構環境 - ROS2 Desktop + dev-tools + 編譯 px4_msgs & 專案
# =============================================================================
FROM taiphoon-ros2 AS taiphoon-builder

# ---- 安裝 ROS2 Humble Desktop + 開發工具 ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# ---- 初始化 rosdep ----
RUN rosdep init && rosdep update

# ---- 系統工具補充 ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    vim \
    nano \
    net-tools \
    iputils-ping \
    htop \
    tmux \
    python3-pip \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# ---- 建立 taiphoon 使用者 ----
RUN useradd -m -s /bin/bash taiphoon \
    && echo "taiphoon ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && usermod -a -G dialout taiphoon \
    && usermod -a -G video taiphoon

# ---- bashrc 環境設定 ----
RUN echo "source /opt/ros/humble/setup.bash" >> /home/taiphoon/.bashrc \
    && echo "export RMW_IMPLEMENTATION=\${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" >> /home/taiphoon/.bashrc \
    && echo "export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}" >> /home/taiphoon/.bashrc \
    && echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /home/taiphoon/.bashrc

USER taiphoon
WORKDIR /home/taiphoon

# ---- 建構 px4_msgs 工作空間 ----
ENV PX4_UROS_WS=/home/taiphoon/px4_ros_uxrce_dds_ws

RUN mkdir -p ${PX4_UROS_WS}/src \
    && cd ${PX4_UROS_WS}/src \
    && git clone --depth 1 https://github.com/PX4/px4_msgs.git \
    && cd ${PX4_UROS_WS} \
    && /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select px4_msgs" \
    && echo "source ${PX4_UROS_WS}/install/setup.bash" >> /home/taiphoon/.bashrc \
    && echo "px4_msgs workspace built successfully"

# ---- 建構 Taiphoon_Agent 工作空間 ----
ENV TAIPHOON_WS=/home/taiphoon/Taiphoon_Agent

COPY --chown=taiphoon:taiphoon . ${TAIPHOON_WS}/

RUN cd ${TAIPHOON_WS} \
    && /bin/bash -c "source /opt/ros/humble/setup.bash && source ${PX4_UROS_WS}/install/setup.bash && colcon build --symlink-install" \
    && echo "source ${TAIPHOON_WS}/install/setup.bash" >> /home/taiphoon/.bashrc \
    && echo "Taiphoon_Agent built successfully"

# =============================================================================
# Stage 3: 開發映像 - 繼承 builder，加上 entrypoint
# =============================================================================
FROM taiphoon-builder AS taiphoon-dev

USER taiphoon
WORKDIR /home/taiphoon

COPY --chown=taiphoon:taiphoon docker/entrypoint.sh /home/taiphoon/entrypoint.sh
RUN chmod +x /home/taiphoon/entrypoint.sh

ENTRYPOINT ["/home/taiphoon/entrypoint.sh"]
CMD ["bash"]

# =============================================================================
# Stage 4: 精簡執行映像 - 從 ros2 共用層獨立建構，僅含 ros-base
# =============================================================================
FROM taiphoon-ros2 AS taiphoon-runtime

# ---- 僅安裝 ROS2 Humble 基礎執行環境 (不裝 Desktop/GUI) ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    && rm -rf /var/lib/apt/lists/*

# ---- 建立 taiphoon 使用者 ----
RUN useradd -m -s /bin/bash taiphoon \
    && echo "taiphoon ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# ---- bashrc 環境設定 ----
RUN echo "source /opt/ros/humble/setup.bash" >> /home/taiphoon/.bashrc \
    && echo "export RMW_IMPLEMENTATION=\${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" >> /home/taiphoon/.bashrc \
    && echo "export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}" >> /home/taiphoon/.bashrc

ENV PX4_UROS_WS=/home/taiphoon/px4_ros_uxrce_dds_ws
ENV TAIPHOON_WS=/home/taiphoon/Taiphoon_Agent

# ---- 從 builder 複製已編譯產出 ----
COPY --from=taiphoon-builder --chown=taiphoon:taiphoon \
    /home/taiphoon/px4_ros_uxrce_dds_ws/install \
    /home/taiphoon/px4_ros_uxrce_dds_ws/install

COPY --from=taiphoon-builder --chown=taiphoon:taiphoon \
    /home/taiphoon/Taiphoon_Agent/install \
    /home/taiphoon/Taiphoon_Agent/install

COPY --from=taiphoon-builder --chown=taiphoon:taiphoon \
    /home/taiphoon/Taiphoon_Agent/config \
    /home/taiphoon/Taiphoon_Agent/config

COPY --from=taiphoon-builder --chown=taiphoon:taiphoon \
    /home/taiphoon/Taiphoon_Agent/launch \
    /home/taiphoon/Taiphoon_Agent/launch

# ---- bashrc 補上 workspace source ----
RUN echo "source ${PX4_UROS_WS}/install/setup.bash" >> /home/taiphoon/.bashrc \
    && echo "source ${TAIPHOON_WS}/install/setup.bash" >> /home/taiphoon/.bashrc

USER taiphoon
WORKDIR /home/taiphoon

COPY --chown=taiphoon:taiphoon docker/entrypoint.sh /home/taiphoon/entrypoint.sh
RUN chmod +x /home/taiphoon/entrypoint.sh

ENTRYPOINT ["/home/taiphoon/entrypoint.sh"]
CMD ["bash"]
