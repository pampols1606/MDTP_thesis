# Multiple drone Tracking and Pursuit
## Introduction
This repository contains the ROS 2 packages for running an algorithm to track multiple [Parrot BEBOP 2](https://www.parrot.com/en/support/documentation/bebop-range) drones using a [Parrot ANAFI](https://www.parrot.com/en/support/documentation/anafi) drone and pursue one of the detected drones. This is part of a Master's thesis written by Ferran Pampols Termens and supervised by Dr. Martin Barczyk, titled [*Multiple Drone Tracking and Pursuit*](https://gitlab.com/barczyk-mechatronic-systems-lab/anafi_ros2/-/raw/main/Docs/Pampols_Termens_Ferran_202406_MSc.pdf).

## Setup
These instructions are tested on Ubuntu 20.04 (Focal) and [ROS 2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation.html), which has to be installed at first. They should also work on other LTS versions of Ubuntu and ROS 2. Any other OS is not recommended or tested.

An NVIDIA GeForce RTX 3090 GPU has been used with CUDA compiler (nvcc) 11.7. To install CUDA see its [official webpage](https://developer.nvidia.com/cuda-11-7-0-download-archive?target_os=Linux). Then to be sure the right compiler version is selected paste these lines at the end of your .bashrc file.

```bash
export PATH="/usr/local/cuda-11.7/bin:$PATH"
export LD_LIBRARY_PATH="/usr/local/cuda--11.7/lib64:$LD_LIBRARY_PATH"
```

To run the files contained in this repository, it is required to install some packages contained in the requirements.txt file. To automatically install the packages, you can run the next command:

```bash
cd ~/anafi_ros2
python -m pip install -r requirements.txt
```

Additionally, Parrot Sphinx has been used for testing some parts of the code in a simulated environment. See the official documentation of Sphinx and follow the [installation procedure](https://developer.parrot.com/docs/sphinx/installation.html).

## ROS 2 applications
Note: For all `ros2 launch <package> <launch file>` command, you can add the option `--show-args` to get an explanation of what launch arguments are expected and how to use them. If no arguments are changed, the default values will be used.

### Simulated environment operation

To test the funcionality of the manual and pursuing control and  code in Sphinx first, run the next command to start the systemd service firmwared:

```bash
sudo systemctl start firmwared.service
```

Next, launch a first simulation of the ANAFI drone by entering the following command:

```bash
sphinx "/opt/parrot-sphinx/usr/share/sphinx/drones/anafi.drone"::firmware="https://firmware.parrot.com/Versions/anafi/pc/%23latest/images/anafi-pc.ext2.zip"
```

Once launched, the core application is waiting for the Unreal Engine application to connect In a second shell, run for example:
```bash
parrot-ue4-empty
```

With the simulated world started, then in a different shell, run the following commands to build the ROS 2 packages, changing *<path_to_project_workspace>* to the path of your workspace containing this repository:

```bash
source /opt/ros/foxy/setup.bash
cd <path_to_project_workspace>
colcon build
source install/setup.bash
```

For the simulation, the position variables of the world's frame origin are used as the position of a BEBOP 2 drone. To get that data, run the following command:

```bash
. /opt/parrot-sphinx/usr/bin/parrot-sphinx-setenv.sh
```

Then paste the following command to run the launch file to test both the manual and automatic control:

```bash
ros2 launch sphinx_ros2 pursuer_sim_launch.py
```

This launch file containes the nodes to control the ANAFI drone through the Parrot Olympe library. A part of the manual control, the automatic control allows to test the PID controller and its parameters by sending the pose of the center of the world as if it was one BEBOP 2 drone to pursue. The keyboard controls for the manual control are the following:

- Take off --> *up*
- Land --> *down*
- Borward --> *w*
- Backward --> *s*
- Right --> *d*
- Left --> *a*
- Up --> *r*
- Down --> *f*
- Roate clockwise --> *c*
- Roate counter-clockwise --> *x*

![Manual control](https://gitlab.com/barczyk-mechatronic-systems-lab/anafi_ros2/-/raw/main/Docs/keyboard_control.jpg?ref_type=heads&inline=false)

To start/end the pursuit of the BEBOP 2 drone (the drone tries to reach the origin of the world's coordinate drame) press the *space* key. 

To close the control algorithm run *Ctrl+C* to the shell in which the program was started. After finishing the run of the launch file, the program will close and if the drone is flying it will land.

Also there are other nodes that have been used for testing and saving data:

- af_step_sim: This node connects to the ANAFI drone and allows to control the drone by sending a step signal in each one of the PCMD function parameters of the Olympe library. The behaviour of the step signals can be changed in a GUI.

For more information about this node, please see the files on the *sphinx_ros2* ROS 2 package.

### Real environment operation
To run the algorithm in a real environment, first make sure that you are connected through Wi-Fi to the ANAFI's IP adress. Then open a shell and build the files as in the previous section (you can skip the *colcon build* step if you have already done it):

```bash
source /opt/ros/foxy/setup.bash
cd <path_to_project_workspace>
colcon build
source install/setup.bash
```

Then paste the following command to run the launch file to test both the manual and automatic control:

```bash
ros2 launch anafi_ros2 pursuer_launch.py
```

The keys used to control the behaiviour of the system are the same as in the simulation environment. Additionally, it has been added the functinality of the *Esc* key to land and disconnect from the ANAFI drone as an improvement for a safety test of the algorith. If tou want to close the complete functionality of the launch file, press *Ctrl+C* to the shell.

To start the pursuit of the BEBOP 2 drone in a real environment, select one of the Detected drones using the GUI of the *Select target* popup window. Then press the *space* key to start/end the pursuit.

Other nodes have been used for testing and saving data:

- af_save_data: It saves the data obtained by the pose estimation and the ground truth position of both the pursuer and pursued drones using a Vicon System.

- af_frame_pub: This node connects to the ANAFI drone and publishes the frames obtained by its camera to a ROS 2 topic named *anafi/frames*.

- af_vicon_step: This node connects to the ANAFI drone and allows to control the drone by sending a step signal in each one of the PCMD function parameters of the Olympe library. The behaviour of the step signals can be changed in a GUI.

For more information about these nodes, please see the files on the *anafi_ros2* ROS 2 package.


# Contact
Contact Ferran Pampols Termens for help (pampolst@ualberta.ca)



