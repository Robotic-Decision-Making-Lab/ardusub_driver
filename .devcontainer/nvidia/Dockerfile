FROM ghcr.io/robotic-decision-making-lab/ardusub_driver:iron-desktop-nvidia

# Install ROS dependencies
# This is done in a previous stage, but we include it again here in case anyone wants to
# add new dependencies during development
ENV USERNAME=ros
ENV USER_WORKSPACE=/home/$USERNAME/ws_ros
WORKDIR $USER_WORKSPACE

COPY --chown=$USER_UID:$USER_GID . src/$PROJECT_NAME

RUN sudo apt-get -q update \
    && sudo apt-get -q -y upgrade \
    && rosdep update \
    && rosdep install -y --from-paths . --ignore-src -r --rosdistro ${ROS_DISTRO} \
    && sudo apt-get autoremove -y \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install \
    pre-commit

# Disable the setuputils installation warning
# This prevents us from needing to pin the setuputils version (which doesn't always work)
ENV PYTHONWARNINGS="ignore"
