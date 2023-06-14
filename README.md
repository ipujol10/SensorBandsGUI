# SensorBandsGUI
## Requirements

* Have ROS2 Humble installed ([Install ROS2 Humble](Requerimentsrequirements))
* Have [PyQt5](https://pypi.org/project/PyQt5/)
* Have [Matplotlib](https://matplotlib.org/stable/users/getting_started/index.html#installation-quick-start)
* Have [git](https://git-scm.com/downloads) installed

## Instalation
Create a workspace in your computer ([Create a workspace](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html))

Clone the repository inside the *src* folder
```sh
git clone https://github.com/ipujol10/SensorBandsGUI.git [<dest_folder>]
```

## Compilation
In the root folder of the workspace, load it
```sh
source install/local_setup.bash
```

Compile the workspace
```sh
colcon build
```

## Run the GUI
Run the ROS executable
```sh
ros2 run [<package_name>] launch_gui
```

### The GUI
