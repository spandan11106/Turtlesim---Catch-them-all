# Turtlesim - Catch Them All

A ROS 2 project where you control a turtle in the classic `turtlesim` simulator to catch all spawned turtles! This project demonstrates ROS 2 concepts such as custom messages, services, parameters, and multi-node orchestration.

![Project Screenshot](image.png)

[▶️ Watch Demo Video](project_demo.webm)

## Quick Start
1. **Install ROS 2 (Humble or later)**
2. **Clone this repository:**
   ```sh
   git clone https://github.com/spandan11106/Turtlesim---Catch-them-all.git
   cd Turtlesim---Catch-them-all
   ```
3. **Build the workspace:**
   ```sh
   colcon build
   source install/setup.bash
   ```
4. **Launch the project:**
   ```sh
   ros2 launch bringup turtlesim_launch.xml
   ```

## Features
- **Automatic Turtle Catching:** The main controller node automatically finds and catches all turtles in the simulation.
- **Custom Interfaces:** Defines custom messages and services for turtle management.
- **Bringup Launch:** Easily launch the full system with one command.
- **Parameterization:** Choose to catch the closest turtle first or in spawn order.
- **Testing:** Includes basic Python tests and ROS 2 linting.

## Project Structure
```
Turtlesim---Catch-them-all/
├── src/
│   ├── bringup/         # Launch and config files
│   ├── catch_them_all/  # Main logic and nodes
│   └── interfaces/      # Custom messages and services
├── build/               # Ignored: build artifacts
├── install/             # Ignored: install artifacts
├── log/                 # Ignored: logs
├── README.md
└── .gitignore
```

## Parameters and Configuration
- The project uses a YAML file (`src/bringup/config/parameters.yaml`) to set parameters such as `catch_closest_turtle_first`.
- You can easily change these parameters to modify the turtle-catching behavior without changing code.
- Example parameter:
  ```yaml
  catch_closest_turtle_first: true
  ```

## Packages and Nodes
- **catch_them_all**
  - Contains the main logic and nodes:
    - `turtle_controller.py`: Controls the main turtle, subscribes to alive turtles, and sends velocity commands to catch them.
    - `turtle_spawner.py`: Spawns new turtles and manages their lifecycle.
- **interfaces**
  - Defines custom messages (`Turtle.msg`, `TurtleArray.msg`) and services (`CatchTurtle.srv`) for communication between nodes.
- **bringup**
  - Contains launch files and configuration (YAML) for starting the whole system easily.

## Dependencies
- ROS 2 (Humble or later)
- `turtlesim` package
- Python 3

## Usage Examples
### Change Parameters in YAML
Edit `src/bringup/config/parameters.yaml` to change project behavior. For example:
```yaml
/turtle_controller:
  ros__parameters:
    catch_closest_turtle_first: False  # Catch turtles in spawn order
/turtle_spawner:
  ros__parameters:
    turtle_name_prefix: "demo"
    spawn_frequency: 2.0
```
- `catch_closest_turtle_first`: If `True`, the controller will always target the closest turtle. If `False`, turtles are caught in the order they spawn.
- `turtle_name_prefix`: Prefix for spawned turtle names.
- `spawn_frequency`: How often (in seconds) new turtles are spawned.

### ROS 2 Introspection Commands
List topics:
```sh
ros2 topic list
```
Echo turtle pose:
```sh
ros2 topic echo /turtle1/pose
```
List services:
```sh
ros2 service list
```
Call the catch turtle service:
```sh
ros2 service call /catch_turtle interfaces/srv/CatchTurtle "{name: 'turtle2'}"
```

## Troubleshooting
- **Build errors:** Make sure you have sourced the correct ROS 2 setup script and installed all dependencies.
- **No turtles are being caught:** Check your parameters in the YAML file and ensure the nodes are running.
- **Nodes not found:** Use `ros2 node list` to see which nodes are active.
- **Still stuck?** Open an issue on GitHub with your error message and setup details.

## Contributing
Pull requests and issues are welcome!

---
Created by Spandan Mhapsekar