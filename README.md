# Pascals SLAM BCBF work

# Installation
## Requirements
These packages were built and supported with the following requirements:

- Ubuntu 22.04
- ROS 2 Humble Hawksbill

Our research bench over the 2024 Summer using `ROS2 Humble` on `Ubuntu Linux - Jammy Jellyfish (22.04)` can be installed using a guide can be found [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

## Dependencies
After following the tutorial and ensuring ROS2 is correctly installed by following examples at the end of the tutorial, we can install base libraries that may be used for simulation and testing:

- [Rivz2](https://github.com/ros2/rviz/tree/humble)
- [Turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble-devel)
- [Turtlebot3_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_msgs/tree/humble-devel)
- [Turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/humble-devel)
- [Gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2)

After installing these packages, it's best to develop and change the future source code of our required packages within a ROS2 workspace. An install and setup guide of a ROS2 workspace can be found [here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) after ensuring proper install of base ROS2 Humble.

Over the summer, development used `slam_toolbox` as the primary SLAM algorithm to provide a 2D occupancy grid for future BCBF. This isn't ideal, and recall that `slam_toolbox` could not output probabilistic cells within the occupancy grid, and instead created binary output: `0` representing `free`, or `100` representing `occupied`, not a probabilistic value between. However, this could still act as a good baseline for installation and early development, proving a BCBF framework could work on binary states before tackling probabilistic states. If using `slam_toolbox`, it should be downloaded and built within our ROS2 workspace as a source, the GitHub page of `slam_toolbox` for installation and development can be found here:

- [Slam_toolbox](https://github.com/SteveMacenski/slam_toolbox/tree/humble)

At this point, we should have the required packages to run a version of SLAM on a Turtlebot3 virtual model for navigation within Gazebo. We should now look to install both solutions to our CBFs/BCBFs made over the Summer. The most finished solution, replicated code of "Control-Barrier-Aided Teleoperation with Visual-Inertial SLAM for Safe MAV Navigation in Complex Environments" by Siqi Zhou, or better known as the `ICRA Paper`, defines safety based on the boundaries of the map of a SLAM algorithm, and should be a finished, or nearly finished solution in ROS2, store in this repo as package `occupancy_grid_processor` (not the best name). Secondly, we have a partially made solution to "Belief Control Barrier Functions for Risk-Aware Control", made by Matti Vahs. This is the the solution we wish to replicate most, and is a true BCBF solution for SLAM output, and is shown as package `belief_cbf` (a better name). Notable differences include attempts at conceptually easier optimizers, better estimations of trajectory, and accurate safety estimations. These packages should be downloaded as individual folders, placed in the used `ROS2 workspace` /src folder; built code/examples of which will be mentioned soon. 

Before building each package, we should deal with all possible dependencies now. Our CBF solution requires packages `qpsolvers` for QP solving, and `pynput` for initial keyboard velocity control, though using something like teleop_twist_keyboard should be used in the future, however managing topic control to ensure that safe velocities are properly communicated. Installing these packages using pip in a fresh Ubuntu install may look like this, including installing pip as:

```bash
$ sudo apt install python3-dev
$ sudo apt install python3-pip
$ pip install pynput
$ pip install qpsolvers[open_source_solvers]
```

Then, building our individual CBF/BCBF packages after considering dependencies within the /ros2_ws folder looks similar to:

```bash
$ colcon build --packages-select occupancy_grid_processor --symlink-install
$ colcon build --packages-select belief_cbf --symlink-install
```
stderr warnings without fatal failure are expected in building these packages. Running each package may look similar to:

```
$ ros2 run belief_cbf belief_cbf
```
For the BCBF, or
```
$ ros2 run belief_cbf belief_cbf
```

## Launch simulation nodes

To test and confirm that we installed correctly, we can make new terminals for each node and test our BCBF/CBF solutions with `slam_toolbox` and examine its output by executing these commands:

**Terminal 1:**

```bash
$ source /opt/ros/humble/setup.bash
$ export TURTLEBOT3_MODEL=burger
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2:**

```bash
$ source /opt/ros/humble/setup.bash
$ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```

**Terminal 3:**

```bash
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```

**Terminal 4:**

```bash
$ source /opt/ros/humble/setup.bash
$ ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

**Terminal 5:**

```bash
$ source /opt/ros/humble/setup.bash
$ export TURTLEBOT3_MODEL=burger
$ ros2 run turtlebot3_teleop teleop_keyboard
```

**Terminal 6:**
```
$ ros2 run belief_cbf belief_cbf
```
or
```
$ ros2 run belief_cbf belief_cbf
```

# Step-by-step installation and execution
To exactly replicate our environment from a new install of Ubuntu Linux - Jammy Jellyfish (22.04), follow these steps:

### Prerequisites

1. **Install Ubuntu 22.04 (Jammy Jellyfish).**
2. **Run initial update:**

```bash
$ sudo apt update
```

### Install ROS2 Humble

Follow the [official ROS2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). Here's our step by step:

1. **Set up locales:**
```bash
$ sudo apt update && sudo apt install locales
```
Here, we confirmed that our locale was already set as `en_US.UTF-8`, though the guide can be followed to update.

2. **Install required packages:**
```bash
$ sudo apt install software-properties-common
$ sudo add-apt-repository universe
$ sudo apt update && sudo apt install curl -y
```

3. **Add ROS2 apt repository:**
```bash
$ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

4. **Install ROS2 with dev tools**
```bash
$ sudo apt update
$ sudo apt upgrade
$ sudo apt install ros-humble-desktop
$ sudo apt install ros-humble-ros-base
$ sudo apt install ros-dev-tools
```

For simplicity, after a successful install, at this point we can close our current terminal and test in new ones.

5. **Test ROS2 install**

Open a new terminal, and enter this for our talker:
```bash
$ source /opt/ros/humble/setup.bash
$ ros2 run demo_nodes_cpp talker
```

Open another new terminal, with the one we just created still running, and create a listener:

```bash
$ source /opt/ros/humble/setup.bash
$ ros2 run demo_nodes_py listener
```

We should now see messages communicated back and forth with one another. Now, we `CTRL-C` kill both current running ROS2 programs, and close one terminal to still have one, sourced terminal running.

If we ever need to source a terminal to its original ROS2 install, we can use 
```bash
$ source /opt/ros/humble/setup.bash
```

6. **Install packages for slam_toolbox testing**

In our single, sourced, and ready terminal, install our necessary packages:

```bash
$ sudo apt update

$ sudo apt install ros-humble-rviz2

$ sudo apt install ros-humble-turtlebot3
$ sudo apt install ros-humble-turtlebot3-msgs
$ sudo apt install ros-humble-turtlebot3-simulations

$ sudo apt install ros-humble-gazebo-ros-pkgs
```

After a successful install, let's consider dependencies:

```bash
$ rosdep update
```

However, on the first run, we should be prompted that rosdep has not been initiated. If so, run:

```bash
$ sudo rosdep init
```

Then finally, run:

```bash
$ rosdep update
```

7. **Create ROS2 Workspace and Install slam_toolbox**

Similar to earlier, we are now following the [official ROS2 Humble Workspace installation guide](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html). Here's our step by step:

If we are still in the same terminal from our earlier step, continue using it. Otherwise, create a new terminal and source it using 

```bash
$ source /opt/ros/humble/setup.bash
```

Then we begin workspace creation:

```bash
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
```

We can test cloning as followed in the guide, and there is nothing wrong in doing so, though I will assume in following this guide that your workspace is properly configured and we will move to the `slam_toolbox` step. Assuming we are still in the still in the `src` directory, run:

```bash
$ git clone https://github.com/SteveMacenski/slam_toolbox -b humble
$ cd ..
```

Now we should be in the `ros2_ws` directory, outside of `src`

```bash
$ rosdep install -i --from-path src --rosdistro humble -y
$ colcon build --packages-select slam_toolbox
```

8. **Test run slam_toolbox with required packages**

As stated above, we now should be able to test our installation using the following commands in new terminals:

**Terminal 1:**

```bash
$ source /opt/ros/humble/setup.bash
$ export TURTLEBOT3_MODEL=burger
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

(If having an issue with the robot not appearing, try to `CTRL-C` `gazebo` and try again)

**Terminal 2:**

```bash
$ source /opt/ros/humble/setup.bash
$ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```

**Terminal 3:**

```bash
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```

**Terminal 4:**

```bash
$ source /opt/ros/humble/setup.bash
$ ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

**Terminal 5:**

```bash
$ source /opt/ros/humble/setup.bash
$ export TURTLEBOT3_MODEL=burger
$ ros2 run turtlebot3_teleop teleop_keyboard
```

Following the prompt in `teleop_keyboard`, we should now notice the `burger` turtlebot3 model move around the simulation in `gazebo`, and in `rviz2`. The `rviz2` example should be presenting the SLAM map, as well as robot localization, as we are getting updates from the map.

9. **Install dependencies for BCBF and CBF packages**

After running and validifying the builds of Gazebo, slam_toolbox, and turtlebot3, we can now build the dependencies of our CBF and BCBF packages. Close existing terminals, and in a new terminal, execute:

```bash
$ sudo apt install python3-dev
$ sudo apt install python3-pip
$ pip install pynput
$ pip install qpsolvers[open_source_solvers]
```

These Python installs will be needed for our BCBF and CBF, more specifically our current CBF implementation, but it will be good to have.

10. **Download and Install BCBF and CBF implementation**

From any point in our Ubuntu install, execute

```bash
$ 
```


# Updating packages and Future work

We are already aware of the main issue of `slam_toolbox` being unable to provide meaningful output in long-term development. From this issue, we need to either develop changes within `slam_toolbox` to properly display the probabilistic cells missing (already attempted for weeks) or choose to use a new SLAM algorithm entirely. Promising algorithms to explore could RTAB-Map, or ORB-SLAM2/3.

Once a new SLAM algorithm that sends probabilistic 2D occupancy grids is chosen, we need to properly implement the BCBF, reflecting most of the Matti Vahs paper, into `belief_cbf`, or create a new package entirely. Current `belief_cbf` should correctly calculate safety, and should present a partial QP solver + other ideas, but here is when time became the biggest constraint and needs further development for proper implementation.

Topics need to be reconnected to receive current/desired twist input (velocities) and incorporate the CBF filter that then sends the filtered twist command to the robot itself. This process isn't entirely incorporated in the current solutions, where we see the CBF replication incorporating its own keyboard to bypass this development requirement, breaking lots of expectations about keyboard input after the program has ended (which you might see in testing/development). Otherwise, belief_cbf should not incorporate keyboard input yet and should be properly designed to incorporate external topic input and handle the proper output.

QP Solver/optimization problems I truthfully cannot confirm as "the best possible" framework and setup. These should be reexamined and worked on as correctly incorporating gradient and other necessary information. Speaking of gradient, the `belief_cbf` was last being worked on as improving the gradient calculation, where my own attempts were not sufficient enough for what was needed in the QP solver. This resulted in solutions that took too long to compute, giving incorrect results, or in most cases, never solving.

Changes to any packages should be made within `ros2_ws/src/packageName/packageName/packageName.py`, and remade through the `colcon build` process once more after saving.

All development was made outside of docker. I held the belief that working with docker could lead to build errors and remove the modularity of each package within each terminal. I believed that each node dependent in its own terminal would be better in testing/development, though now I realize this belief might be false, and we should explore adding docker for development and running as a proper container.
