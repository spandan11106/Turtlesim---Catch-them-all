# Turtlesim - Catch Them All

A ROS 2 project where you control a turtle in the classic `turtlesim` simulator to catch all spawned turtles! This project demonstrates ROS 2 concepts such as custom messages, services, parameters, and multi-node orchestration.

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

## How to Build and Run
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

## Dependencies
- ROS 2 (Humble or later)
- `turtlesim` package
- Python 3

## License
Specify your license in the `package.xml` files.

## Contributing
Pull requests and issues are welcome!

---
Created by Spandan Mhapsekar