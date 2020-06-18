FROM ros:melodic

RUN apt-get update && apt-get install -y \
                    python-catkin-tools \
                    libopencv-dev \
                    socat \
                    ros-melodic-xacro \
                    ros-melodic-tf \
                    ros-melodic-robot-state-publisher \
                    ros-melodic-rviz \
                    openssh-server

# Taken from - https://docs.docker.com/engine/examples/running_ssh_service/#environment-variables
RUN mkdir /var/run/sshd
RUN echo 'root:root' | chpasswd
RUN sed -i 's/PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config

# SSH login fix. Otherwise user is kicked off after login
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd

ENV NOTVISIBLE "in users profile"
RUN echo "export VISIBLE=now" >> /etc/profile

# 22 for ssh server. 7777 for gdb server.
EXPOSE 22 7777

# Create dev user with password 'dev'
RUN useradd -ms /bin/bash dev
RUN echo 'dev:dev' | chpasswd

RUN apt-get update && apt-get install -y gdb

# Upon start, run ssh daemon
CMD ["/usr/sbin/sshd", "-D"]
