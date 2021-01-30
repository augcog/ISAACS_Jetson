# drone-jetson
This ROS code runs on the ISAACS on-board Linux computer. Our on-board computer is currently the Nvidia Jetson NX and we're using the ZED 2 camera. This repo contains the **ROS Melodic** packages for ZED 2 Camera and Voxblox, for example. This code is to be cloned in the `src` directory of your catkin workspace, and compiled.

Note that the ZED 2 camera requires an Nvidia GPU, so while these instructions may work for any linux computer, only computers with Nvidia GPUs will be able to connect to a ZED 2 camera. All other computers will be able to work only with ROS Bags (recordings) of the ZED 2.

## Installation

### Prerequisites

- Ubuntu 18.04
- ROS Melodic
- [ZED SDK (tested with 3.4.0, CUDA 10.2)](https://www.stereolabs.com/developers/release/)

### Setting up ROS workspace, downloading, and building

Create a catkin workspace as follows. If you've made one before on your personal computer for a different project/tutorial, still make a new one please.

- `mkdir -p ~/ros_catkin_ws/src`
- `cd ~/ros_catkin_ws`
- `catkin init` Creates a catkin workspace (by creating a hidden directory *.catkin_tools*)
- `catkin config --extend /opt/ros/melodic`
- We'll set the compilation flag to be in **Debug** mode so it won't take as long to compile as the **Release** mode. Debug will compile faster with few/no compiler optimizations. Run: `catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug` 
- `catkin config --merge-devel`
- `cd src`
- `git clone https://github.com/immersive-command-system/drone-jetson.git`
- `git submodule update --init --recursive` Clones all the submodules, which are git repos within this git repo. Each submodule is a ROS package.
- Good job!
- The next step is to compile, but do note it takes up quite a bit of RAM. Please watch the RAM usage as you compile so your computer doesn't freeze on you. To limit the RAM usage during compilation, you have two options. The smooth-brain way is to kill `ctrl+c` the compilation process whenever the RAM usage is increasing past 90-95% of your total RAM, and re-run the compilation process. It will pick up where it left off, quickly checking off the stuff it has aleady compiled, and resuming! In addition, you can also limit your usage by closing all other programs, or if using a VM, increase its RAM allowance. I followed the smooth brain way to stay below my 8gb of RAM. And then the wrinkly-brain way of limiting RAM usage is learning to use these [commands/flags](https://github.com/catkin/catkin_tools/issues/167).
- `cd ..` you should be back in the directory *ros_catkin_ws*
- `catkin build` Compiles all the packages
- `source devel/setup.bash` this tells ROS to use this workspace. Run this command on every new terminal you open for running ROS stuff. Alternatively, add `cd $HOME/row_catkin_ws && source devel/setup.bash` to your `.bashrc` file so it will run this command for you every time you open a new terminal.


### Installing other ROS Packages
- ROS Bridge, for sending data to other computers on the network, namely VR interface (or ISAACS server in the future). `sudo apt-get install ros-melodic-rosbridge-server`


