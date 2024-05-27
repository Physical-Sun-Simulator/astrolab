# astrolab
ROS2 package which provides the user interface, arm, table and digital twin nodes and connections for the Physical Sunlight Simulator. Note that the following guides assumes the user is using Linux.

## Table of Contents
- [astrolab](#astrolab)
  - [Table of Contents](#table-of-contents)
  - [How to install](#how-to-install)
  - [How to run the package for the first time](#how-to-run-the-package-for-the-first-time)
  - [How to set up rqt for execution diagnostics](#how-to-set-up-rqt-for-execution-diagnostics)
  - [How to set up rqt for connection diagnostics](#how-to-set-up-rqt-for-connection-diagnostics)

## How to install
1. Make a ROS workspace as described [here](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
2. Clone the astrolab repository in the src directory (e.g. using `git clone https://github.com/Physical-Sun-Simulator/astrolab`)
3. Clone the simulation_interfaces repository in the src directory (e.g. using `git clone https://github.com/Physical-Sun-Simulator/simulation_interfaces`)
4. Run `source install/setup.bash` in the workspace directory
5. Build the package using `colcon build` in the workspace directory and check there are no errors!

## How to run the package for the first time
1. Run `source install/setup.bash` in the workspace directory
2. Make and activate a Python virtual environment in the workspace directory as described [here](https://docs.python.org/3/library/venv.html)
3. Run `pip install Flask https://github.com/Physical-Sun-Simulator/sun_position`
4. Build the package using `colcon build` in the workspace directory
5. Execute `ros2 launch astrolab demo_launch.py`
6. Open `127.0.0.1` in a webbrowser

## How to set up rqt for execution diagnostics
1. Run `source install/setup.bash` in the workspace directory
2. Execute `rqt` in the workspace directory
3. In the Perspectives tab click on important and select `Diagnostics - 1.perspective` in the rqt_perspectives folder.
4. In the top-left most component (Console) click on Fit Columns.
5. In the component on its right (Rotational) fill in `/arm_angle_topic/data` in the Topic textfield, click on Subscribe and fill in 90 in the Max Value textfield.
6. In the component on its right (Bar) fill in `/arm_speed_service/data` in the Topic textfield, click on Subscribe, fill in 100 in the Max Value textfield and fill in 0 in the Min Value textfield.
7. In the component on its right (Rotational) fill in `/table_angle_topic/data` in the Topic textfield, click on Subscrbie and fill in 260 in the Max Value textfield.
8. In the component on its right (Bar) fill in `/table_speed_service/data` in the Topic textfield, click on Subscribe, fill in 100 in the Max Value textfield and fill in 0 in the Min Value textfield.
9. In the down-left most component (Service Caller) set the Service dropdown to `/arm_speed_service`.
10. In the down-right most component (Service Caller (2)) set the Service dropdown to `/table_speed_service`.

## How to set up rqt for connection diagnostics
1. Run `source install/setup.bash` in the workspace directory
2. Execute `rqt` in the workspace directory
3. In the Perspectives tab click on important and select `Diagnostics - 1.perspective` in the rqt_perspectives folder.