# =============================================================================
# @file Dockerfile
# @brief Taiphoon_Agent 多階段 Docker 建構檔案（含 uXRCE-DDS 橋接）
# @author Taiphoon Agent Team
# @date 2026-07-23
#
# Stage 階層架構:
#   taiphoon-base        共用基礎 (locale)
#     └── taiphoon-ros2   共用 ROS2 repo 設定
#           ├── taiphoon-builder   完整編譯環境 (Desktop + dev-tools + uXRCE-DDS 橋接)
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
# Stage 1: 共用 ROS2 repo 層
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
# Stage 2: 完整建構環境 - ROS2 Desktop + dev-tools + uXRCE-DDS 橋接
# =============================================================================
FROM taiphoon-ros2 AS taiphoon-builder

# Release 關閉 symlink-install，避免 runtime 指向未複製的 source／build。
ARG COLCON_SYMLINK_INSTALL=true
ARG PX4_MSGS_REF=main
ARG BUILD_JOBS=2
ARG ROS_DEVELOPMENT_PACKAGE=ros-humble-desktop
ARG COLCON_BUILD_TYPE=""
ENV CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_JOBS}
ENV MAKEFLAGS=-j${BUILD_JOBS}

# ---- 安裝 ROS2 Humble Desktop + 開發工具 ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    ${ROS_DEVELOPMENT_PACKAGE} ros-humble-rmw-cyclonedds-cpp \
    ros-humble-ament-cmake-gtest python3-yaml \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# ---- 初始化 rosdep ----
RUN rosdep init && rosdep update

# ---- 系統工具補充 (與原始 Dockerfile 一致，保留 cache) ----
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
    procps \
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
    && git clone https://github.com/PX4/px4_msgs.git \
    && git -C px4_msgs checkout ${PX4_MSGS_REF} \
    && cd ${PX4_UROS_WS} \
    && /bin/bash -ec 'source /opt/ros/humble/setup.bash; \
        install_args=(); \
        if [ "${COLCON_SYMLINK_INSTALL}" = true ]; then install_args+=(--symlink-install); fi; \
        colcon build "${install_args[@]}" --packages-select px4_msgs \
        --cmake-args "-DCMAKE_BUILD_TYPE=${COLCON_BUILD_TYPE}"' \
    && echo "source ${PX4_UROS_WS}/install/setup.bash" >> /home/taiphoon/.bashrc \
    && echo "px4_msgs workspace built successfully"

# ---- 安裝 MicroXRCEAgent 編譯依賴 (放在 px4_msgs 之後，保留上游 cache) ----
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    libssl-dev \
    ros-humble-foonathan-memory-vendor \
    && rm -rf /var/lib/apt/lists/*

# ---- 建構 MicroXRCEAgent (uXRCE-DDS 橋接) ----
# 放在 COPY . . 之前，確保 Taiphoon 程式碼變動不會觸發重編
USER taiphoon
ENV UXRCE_AGENT_VERSION=v2.4.3
ENV UXRCE_AGENT_SRC=/tmp/Micro-XRCE-DDS-Agent

RUN git clone --depth 1 --branch ${UXRCE_AGENT_VERSION} \
    https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
    ${UXRCE_AGENT_SRC} \
    && mkdir -p ${UXRCE_AGENT_SRC}/build \
    && cd ${UXRCE_AGENT_SRC}/build \
    && /bin/bash -c "source /opt/ros/humble/setup.bash && cmake .. \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DCMAKE_PREFIX_PATH=/opt/ros/humble \
        -Dfoonathan_memory_DIR=/opt/ros/humble/lib/foonathan_memory/cmake" \
    && make -j${BUILD_JOBS} \
    && echo "MicroXRCEAgent build complete"

# ---- 安裝 MicroXRCEAgent (需 root) ----
USER root
RUN cd ${UXRCE_AGENT_SRC}/build \
    && make install \
    && find temp_install -name 'libfast*.so*' -exec cp -a {} /usr/local/lib/ \; 2>/dev/null || true \
    && ldconfig \
    && echo "MicroXRCEAgent installed to /usr/local/bin"

# ---- 驗證 MicroXRCEAgent binary 存在 ----
USER taiphoon
RUN which MicroXRCEAgent && echo "MicroXRCEAgent found at $(which MicroXRCEAgent)"

# ---- 建構 Taiphoon_Agent 工作空間 ----
ENV TAIPHOON_WS=/home/taiphoon/Taiphoon_Agent

COPY --chown=taiphoon:taiphoon . ${TAIPHOON_WS}/

RUN cd ${TAIPHOON_WS} \
    && /bin/bash -ec 'source /opt/ros/humble/setup.bash; \
        source ${PX4_UROS_WS}/install/setup.bash; \
        install_args=(); \
        if [ "${COLCON_SYMLINK_INSTALL}" = true ]; then install_args+=(--symlink-install); fi; \
        colcon build "${install_args[@]}" --executor sequential \
        --cmake-args "-DCMAKE_BUILD_TYPE=${COLCON_BUILD_TYPE}"' \
    && echo "source ${TAIPHOON_WS}/install/setup.bash" >> /home/taiphoon/.bashrc \
    && echo "Taiphoon_Agent built successfully"

# ---- 系統級 shell 環境 (支援 docker compose exec 非互動式 shell) ----
USER root
RUN echo "source /opt/ros/humble/setup.bash" > /etc/profile.d/ros2.sh \
    && echo "source ${PX4_UROS_WS}/install/setup.bash" >> /etc/profile.d/ros2.sh \
    && echo "source ${TAIPHOON_WS}/install/setup.bash" >> /etc/profile.d/ros2.sh \
    && chmod +x /etc/profile.d/ros2.sh

# 讓 non-interactive bash shell 也能自動 source ROS2 環境
ENV BASH_ENV=/etc/profile.d/ros2.sh

USER taiphoon

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
# Stage 4: 精簡執行環境 - 從 ros2 共用層獨立建構，僅含 ros-base
# =============================================================================
FROM taiphoon-ros2 AS taiphoon-runtime

# ---- 僅安裝 ROS2 Humble 基礎執行環境 ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rmw-cyclonedds-cpp \
    procps \
    && rm -rf /var/lib/apt/lists/*

# ---- 建立 taiphoon 使用者 ----
RUN useradd -m -s /bin/bash taiphoon \
    && echo "taiphoon ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && usermod -a -G dialout taiphoon

# ---- bashrc 環境設定 ----
RUN echo "source /opt/ros/humble/setup.bash" >> /home/taiphoon/.bashrc \
    && echo "export RMW_IMPLEMENTATION=\${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" >> /home/taiphoon/.bashrc \
    && echo "export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}" >> /home/taiphoon/.bashrc

ENV PX4_UROS_WS=/home/taiphoon/px4_ros_uxrce_dds_ws
ENV TAIPHOON_WS=/home/taiphoon/Taiphoon_Agent

# ---- 從 builder 複製已編譯產出 ----
COPY --from=taiphoon-builder --chown=taiphoon:taiphoon \
    /usr/local/bin/MicroXRCEAgent \
    /usr/local/bin/MicroXRCEAgent

COPY --from=taiphoon-builder --chown=taiphoon:taiphoon \
    /usr/local/lib/ \
    /usr/local/lib/

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

RUN ldconfig

# ---- bashrc 補上 workspace source ----
RUN echo "source ${PX4_UROS_WS}/install/setup.bash" >> /home/taiphoon/.bashrc \
    && echo "source ${TAIPHOON_WS}/install/setup.bash" >> /home/taiphoon/.bashrc

# ---- 系統級 shell 環境 ----
RUN echo "source /opt/ros/humble/setup.bash" > /etc/profile.d/ros2.sh \
    && echo "source ${PX4_UROS_WS}/install/setup.bash" >> /etc/profile.d/ros2.sh \
    && echo "source ${TAIPHOON_WS}/install/setup.bash" >> /etc/profile.d/ros2.sh \
    && chmod +x /etc/profile.d/ros2.sh

ENV BASH_ENV=/etc/profile.d/ros2.sh

USER taiphoon
WORKDIR /home/taiphoon

COPY --chown=taiphoon:taiphoon docker/runtime-entrypoint.sh /home/taiphoon/runtime-entrypoint.sh
COPY --chown=taiphoon:taiphoon docker/runtime-healthcheck.sh /home/taiphoon/runtime-healthcheck.sh
RUN chmod +x /home/taiphoon/runtime-entrypoint.sh /home/taiphoon/runtime-healthcheck.sh
HEALTHCHECK --interval=5s --timeout=3s --start-period=40s --retries=3 \
    CMD /home/taiphoon/runtime-healthcheck.sh
ENTRYPOINT ["/home/taiphoon/runtime-entrypoint.sh"]
CMD []

# Release 的開發映像沿用工具鏈，但使用不自動編譯的受監控入口。
FROM taiphoon-dev AS taiphoon-release-dev
COPY --chown=taiphoon:taiphoon docker/runtime-entrypoint.sh /home/taiphoon/runtime-entrypoint.sh
COPY --chown=taiphoon:taiphoon docker/runtime-healthcheck.sh /home/taiphoon/runtime-healthcheck.sh
RUN chmod +x /home/taiphoon/runtime-entrypoint.sh /home/taiphoon/runtime-healthcheck.sh
HEALTHCHECK --interval=5s --timeout=3s --start-period=40s --retries=3 \
    CMD /home/taiphoon/runtime-healthcheck.sh
ENTRYPOINT ["/home/taiphoon/runtime-entrypoint.sh"]
CMD []
