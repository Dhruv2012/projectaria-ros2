# EgoMimic Hardware
Hardware repo for EgoMimic [Project Page](https://egomimic.github.io/) | [Hardware Doc](https://docs.google.com/document/d/1ac5yN-IIRzRgKeJleBk5XEHwk4tcsTsFBDvItDau1Nk/edit?tab=t.0#heading=h.axwpytr6n3bk)

This repo interfaces with the hardware components of EgoMimic, including the ViperX arms and supports using Aria Glasses as part of ROS.  We fork off of the Aloha ROS2 implementation from [Trossen](https://github.com/Interbotix/aloha.git).

To train imitation learning policies for this hardware see EgoMimic Github

## Structure
- [``aloha``](./aloha/): Python package providing useful classes and constants for teleoperation and dataset collection.
- [``config``](./config/): a config for each robot, designating the port they should bind to, more details in quick start guide.
- [``launch``](./launch): a ROS 2 launch file for all cameras and manipulators.
- [``scripts``](./scripts/): Python scripts for teleop and data collection


## Install instructions
Build all of this without a conda environment in `~/interbotix_ws`
1. Set up ROS2, interbotix packages, and realsense cameras via the [(Trossen Instructions)](https://docs.trossenrobotics.com/aloha_docs/getting_started/stationary/software_setup.html).  But critically under the step for [Aloha Software Installation](https://docs.trossenrobotics.com/aloha_docs/getting_started/stationary/software_setup.html#aloha-software-installation), make sure to clone this repo instead

```
IGNORE: git clone https://github.com/Interbotix/aloha.git
git clone git@github.com:SimarKareer/EgoMimic-Hardware.git
```

2. Setup Aria Glasses [Aria Instructions](https://facebookresearch.github.io/projectaria_tools/docs/ARK/sdk/setup).  In brief you just need to run
```
python3 -m pip install projectaria_client_sdk --no-cache-dir
aria-doctor
aria auth pair (then open mobile app and approve)
```
3. Install extra packages
```
pip install opencv-python fastplotlib==0.1.0a13 glfw==2.6.2 pyg==0.1.14 dm_env h5py ipython
```
4. Add permissions on video cams
```
sudo usermod -a <user> -G video
```
5. Add aliases to `~/.bash_aliases`
```
alias setup_aloha="source /opt/ros/humble/setup.bash && source ~/interbotix_ws/install/setup.bash"
alias ros_aloha="usbreset "Aria" && sleep 3 && ros2 launch aloha aloha_bringup.launch.py"
```
6. Restart machine

7. If facing issues, build realsense ros package from scratch, I used this [commit](https://github.com/IntelRealSense/realsense-ros/commit/7c163180e56172f38700d9f3ac9a4205de03765e)

### Tips
- Use a standard terminal instead of vscode.
- Any time changes are made to the ROS package, you must run `colcon build` in `~/interbotix_ws`
- ros2 run rqt_reconfigure rqt_reconfigure


## Usage
To start up ROS run
```
setup_aloha
ros_aloha
```

To ensure all camera sensors are functional launch Rviz
```
rviz2
```

Occasionally Aria fails to connect, simply restart the computer and it should connect fine.

## Data Collection / Teleop Scripts
For data collection and teleop we'll continue to use the base python installation (no conda env)

### Teleoperation
```
python scripts/dual_side_teleop.py --arm <left, right, both>
```

### Data Collection
First, define a task in `aloha/constants.py`.  After adding this you'll need to rebuild via `colcon build`.  Then run
```
python scripts/dual_side_teleop.py --arm <left, right, both> --task_name <task name>
```