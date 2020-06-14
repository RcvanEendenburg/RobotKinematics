FROM ros:melodic

RUN apt-get update && apt-get install -y python-catkin-tools libopencv-dev socat
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash'

COPY ./workspace /workspace

WORKDIR /workspace
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make'

COPY ./docker/image/scripts/rk_entrypoint.sh /
COPY ./docker/image/scripts/run_rk.sh /
RUN ["chmod", "775", "devel/setup.bash"]
RUN ["chmod", "775", "/rk_entrypoint.sh"]
RUN ["chmod", "+x", "/run_rk.sh"]
ENTRYPOINT ["/rk_entrypoint.sh"]