# Build:
#  docker build --rm -f Dockerfile -t gramaziokohler/novnc .
#
# Usage:
#  docker pull gramaziokohler/novnc

FROM ros:melodic

ENV TZ=Asia/Shanghai
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install git, supervisor, VNC, & X11 packages
RUN apt-get update && apt-get install -y \
    websockify \
    ca-certificates \
    fluxbox \
    git \
    net-tools \
    supervisor \
    x11vnc \
    xterm \
    xvfb

# Setup demo environment variables
ENV HOME=/root \
    DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    LC_ALL=C.UTF-8 \
    DISPLAY=:0.0 \
    DISPLAY_WIDTH=1024 \
    DISPLAY_HEIGHT=768 \
    RUN_FLUXBOX=yes

RUN apt-get update && apt-get install -y novnc

RUN apt-get update && apt-get install -y socat
RUN apt-get update && apt-get install -y ros-melodic-xacro ros-melodic-tf ros-melodic-robot-state-publisher
RUN apt-get update && apt-get install -y ros-melodic-rviz
COPY ./workspace /workspace

WORKDIR /workspace

COPY ./docker/novnc/.supervisord /root/.supervisord
EXPOSE 8080

RUN ["chmod", "775", "devel/setup.bash"]
CMD ["supervisord", "-c", "/root/.supervisord/supervisord.conf"]
