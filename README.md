# UR3e TSP Motion Planning and Visualization

This project aims to control a **Universal Robots UR3e** robotic arm using **ROS 2** (Humble) and Python. The arm performs a motion planning task where it traces a path in a circular trajectory. Additionally, this project integrates a variation of the **Traveling Salesman Problem (TSP)**, specifically an **Asymmetric TSP (ATSP)** solver, to optimize the sequence in which the arm visits a set of waypoints. The motion of the arm is visualized in **RViz** for better understanding and demonstration.

## Features

- **ROS 2 Control**: The robot is controlled using **ROS 2 (Humble)**, with joint trajectories published to the arm's **joint_trajectory_controller**.
- **Circle Path Planning**: The arm follows a circular path with customizable radius and step count.
- **TSP Integration**: The project implements an **Asymmetric TSP (ATSP)** solver to optimize the robot's movement between waypoints in the shortest possible order.
- **Waypoint Visualization**: The waypoints along the robot's path are highlighted in **RViz** using ROS markers, providing real-time visualization.
- **Flexible Path Control**: The robot can move between waypoints generated from the circular path or any other user-defined path, with optimization based on TSP solutions.

## Prerequisites

- **ROS 2 Humble** installed on an Ubuntu system.
- **Python 3** and required Python packages.
- A working setup with the **UR3e robot** and relevant controllers.
- **RViz** for visualization.

## Setup

### 1. Install ROS 2 Humble

Follow the [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) for **ROS 2 Humble**.

### 2. Install Python Dependencies

Install required Python packages for TSP solving and ROS 2 integration.

```bash
sudo apt install python3-colcon-common-extensions python3-argcomplete python3-rclpy
pip install python-tsp
```

### 3. Create and Build a Workspace

Create a ROS 2 workspace and build it.

```bash
mkdir -p ~/ur3e_ws/src
cd ~/ur3e_ws
colcon build
source install/setup.bash
```

### 4. Clone the Repository

Clone the repository into your workspace.

```bash
cd ~/ur3e_ws/src
git clone https://github.com/your-username/ur3e-tsp-motion.git
cd ~/ur3e_ws
colcon build
source install/setup.bash
```

### 5. Run the Node

After building the workspace, run the node that performs the TSP motion planning.

```bash
ros2 run ur3e_tsp_motion ur3e_circle_motion
```

### 6. Visualize in RViz

Once the node is running, open **RViz** and set the following fixed frame: `base_link`.

You will see the **waypoints** visualized as red spheres, and the **robot's path** will be updated according to the TSP optimization.

```bash
ros2 run rviz2 rviz2
```

### 7. (Optional) Modify Waypoints and Path

You can modify the number of waypoints and their path by adjusting the `radius` and `num_steps` parameters in the `UR3eCircleMotion` class.

## TSP Solver

This project uses the **python-tsp** library to solve the TSP. The solver optimizes the order of the waypoints to minimize the total travel distance. The algorithm uses a heuristic to provide a solution in a reasonable amount of time, although this can be further improved by integrating more advanced solvers like **LKH** (Lin-Kernighan-Helsgaun) or others.

## Contributing

Feel free to fork the repository and create a pull request with improvements or bug fixes. Contributions are always welcome!

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
