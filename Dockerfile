# NVIDIA's multi-architecture manifest; Docker selects the native architecture.
ARG ISAAC_SIM_IMAGE=nvcr.io/nvidia/isaac-sim:6.0.0@sha256:68735a60b6c15c85e0dd0098570c6d2cc79e928f2d068ce2790aa43284ac165d
FROM ${ISAAC_SIM_IMAGE}
USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN . /etc/os-release && test "$VERSION_ID" = 24.04
COPY docker/ros-archive-keyring.gpg /usr/share/keyrings/ros-archive-keyring.gpg
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg locales software-properties-common \
    && add-apt-repository universe \
    && locale-gen en_US.UTF-8
RUN echo 'deb [arch=amd64,arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main' \
       > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git python3-dev python3-venv python3-pip python3-yaml \
    python3-colcon-common-extensions python3-rosdep \
    ros-jazzy-desktop ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-moveit ros-jazzy-moveit-planners-ompl ros-jazzy-moveit-kinematics \
    ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-cv-bridge ros-jazzy-image-geometry \
    ros-jazzy-tf2-geometry-msgs ros-jazzy-tf-transformations \
    ros-jazzy-pointcloud-to-laserscan ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 ROS_DISTRO=jazzy \
    ROS_DOMAIN_ID=44 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    COURSE_ROOT=/workspace/course COURSE_WS=/workspace/bobac_ws \
    VISION_PYTHON=/opt/course-vision/bin/python
COPY docker/vision-requirements.txt /opt/course/vision-requirements.txt
RUN /usr/bin/python3 -m venv --system-site-packages /opt/course-vision \
    && /opt/course-vision/bin/python -m pip install \
       torch==2.9.0 torchvision==0.24.0 --index-url https://download.pytorch.org/whl/cu130 \
    && /opt/course-vision/bin/python -m pip install -r /opt/course/vision-requirements.txt \
    && /opt/course-vision/bin/python -m pip check \
    && /opt/course-vision/bin/python -m pip freeze --local > /opt/course/vision-resolved.txt \
    && dpkg-query -W > /opt/course/apt-resolved.txt
COPY bobac_robotac_src/src /workspace/bobac_ws/src
COPY docker/entrypoint.sh docker/usd-python.sh /opt/course/
COPY tools /opt/course/tools
RUN source /opt/ros/jazzy/setup.bash \
    && cd /workspace/bobac_ws \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && cp -a install /opt/course/prebuilt-install \
    && chmod +x /opt/course/*.sh \
    && chown -R 1234:1234 /workspace /opt/course-vision /opt/course
USER 1234:1234
WORKDIR /workspace/course
ENTRYPOINT ["/opt/course/entrypoint.sh"]
CMD ["bash"]
