# Robotic Unscrewing

This package provides a robotic system for automated unscrewing tasks using ROS 2. Follow the steps below to build, launch, and run the system.


## Getting Started

### 1. Build and Start the Docker Container

In your terminal, run the following command to build and start the Docker container

```bash
 docker compose up --build
```

### 2. Run the system

This will run moveit, screwdriver, and camera APIs. This will run all action servers, services and let the system ready to take commands

In a new terminal, attach to the running container

```bash
docker exec -it devcontainer /bin/bash
```

Then, inside the container, launch the system launch file

```bash
ros2 launch system_state_machine system_without_sm.launch.py
```

Wait until you see the following message in the terminal

```bash
[move_group-3] You can start planning now!
```

### 3. Run the state machine 

Now start unscrewing!!

Open another new terminal, and attach to the container

Open another new terminal, and again enter the development container:

```bash
docker exec -it devcontainer /bin/bash
```

Then, inside the container, run the state machine

```bash
ros2 run system_state_machine system_sm
```

## Visualization

What will you see?

Rviz: to visualize the robot motions

To visualize the state machine you can simply open a new terminal and attach to the container and then run

```bash
ros2 ros2 run yasmin_viewer yasmin_viewer_node
```

Then on your web browser open the following link:

```bash
http://localhost:5000
```
