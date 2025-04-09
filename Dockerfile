FROM moveit/moveit2:jazzy-release

RUN apt-get update && apt-get install -y --no-install-recommends \
  ros-jazzy-yasmin \
  ros-jazzy-yasmin-* \
  git \
  && rm -rf /var/lib/apt/lists/*

USER ubuntu

WORKDIR /home/ubuntu

RUN mkdir ros2_ws

WORKDIR /home/ubuntu/ros2_ws

RUN mkdir src

COPY ./src ./src

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"


RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/ubuntu/.bashrc
RUN echo "source /home/ubuntu/ros2_ws/install/setup.bash" >> /home/ubuntu/.bashrc

CMD ["bash"]
