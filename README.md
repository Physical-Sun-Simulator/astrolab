# astrolab
ROS2 package which provides the user interface, arm, table and digital twin nodes and connections for the Physical Sunlight Simulator

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

## How to utilize rqt for diagnostics
1. Run `source install/setup.bash` in the workspace directory
2. Execute `rqt` in the workspace directory
3. Open the "Basic" profile in the Profile tab
4. Fill in `/arm_angle_topic/data` for the left most component
5. Set the maximum to 90 for the left most component
6. Fill in `/arm_speed_service/data` for the next component
7. Set the maximum to 100
8. Set the minimum to 0
9. Fill in `/table_angle_topic/data` for the next component
10. Set the maximum to 360
11. Fill in `/table_speed_service/data` for th next component
12. Set the maximum to 100
13. Set the minimum to 0