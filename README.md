# Butler Bot - Restaurant Automation (ROS 2)

Butler Bot is a ROS 2 based restaurant automation simulation that manages customer orders and executes turtle bot navigation in a Gazebo environment.  
It demonstrates order handling, task execution, and robot movement services

---

## Features
- **Order Management**: Handle new and canceled orders using services and actions.  
- **Order Dispatcher**: Publishes active orders to the robot executor.  
- **Order Executor**: Executes tasks, monitors tray status, and controls robot navigation.  
- **Simulation**: Restaurant environment with map and robot movement using SLAM & Nav2
---

## Package Structure
- `butler_bot` → Robot description, controllers, navigation setup
- `butler_order_manager` → Order stack handling, services, and action servers  
- `butler_bot_interfaces` → Custom `.srv` and `.action` definitions (planned / pending setup)  
- `my_robot_description` → URDF model for the robot  
- `my_robot_bringup` → Launch files for simulation and testing     

---

## How to Build
```bash
# Build packages
colcon build --symlink-install
source install/setup.bash

# Install Turtlebot3 binaries

## 7. How to Run
# Launch Restaurant + TurtleBot
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Launch Navigation2 + Map in Rviz2
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:={your_ws}src/restaurant/config/maps/restaurant.yaml

# Start Order Manager
ros2 launch butler_order_manager butler_order_manager.launch.py

# Order/Cancel:
ros2 run butler_order_manager order_client

# Load Machine Sensor Simulation
ros2 run butler_bot load_machine_sim
