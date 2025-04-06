FROM moveit/moveit2:jazzy-release

USER ubuntu

RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/ubuntu/.bashrc
WORKDIR /home/ubuntu

RUN mkdir ros2_ws

WORKDIR /home/ubuntu/ros2_ws

RUN mkdir src

COPY ./lion_robot_description ./src/lion_robot_description
COPY ./lion_moveit_config ./src/lion_moveit_config

RUN colcon build

RUN echo "source /home/ubuntu/ros2_ws/install/setup.bash" >> /home/ubuntu/.bashrc

CMD ["bash"]
