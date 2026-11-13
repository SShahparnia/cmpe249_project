# Mac sim machine (Module 3): Ubuntu 22.04 + ROS 2 Humble + TB4 sim (Ignition Fortress)
# + slam_toolbox + Nav2, with a browser desktop (noVNC) for Gazebo / RViz.
# Build and run through scripts/mac/sim.sh; see docs/mac_setup.md.
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

RUN apt-get update \
 && apt-get install -y --no-install-recommends sudo tzdata locales ca-certificates \
 && rm -rf /var/lib/apt/lists/*

# Reuse the laptop setup script so there is one package list for every sim machine.
# It is copied to the same path the repo is mounted at, so the ~/.bashrc line it
# writes (source <repo>/scripts/ros_env.sh) points at the live repo at runtime.
COPY scripts/laptop/setup_laptop.sh /workspace/cmpe249_project/scripts/laptop/setup_laptop.sh
RUN bash /workspace/cmpe249_project/scripts/laptop/setup_laptop.sh \
 && rm -rf /var/lib/apt/lists/*

# Browser desktop: virtual X display + VNC + noVNC, software OpenGL (no GPU in the container).
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      xvfb x11vnc novnc websockify fluxbox xterm mesa-utils libgl1-mesa-dri \
 && rm -rf /var/lib/apt/lists/*

ENV DISPLAY=:1 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    GALLIUM_DRIVER=llvmpipe \
    QT_X11_NO_MITSHM=1

COPY docker/mac/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

WORKDIR /workspace/cmpe249_project
EXPOSE 6080
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["sleep", "infinity"]
