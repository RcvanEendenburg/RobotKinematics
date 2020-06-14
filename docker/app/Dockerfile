FROM ros:melodic AS build-stage

RUN apt-get update && apt-get install -y python-catkin-tools libopencv-dev socat
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash'

COPY ./workspace /workspace

WORKDIR /workspace
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make'

FROM scratch AS export-stage
COPY --from=build-stage /workspace .