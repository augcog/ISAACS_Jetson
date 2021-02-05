# drone-jetson
This ROS code runs on the ISAACS on-board Linux computer and interfaces with the ZED camera as well as the DJI drone. Our on-board computer is currently the Nvidia Jetson NX (instead of the Manifold 2) and we're using the ZED 2 camera. This repo contains the **ROS Melodic** packages for DJI SDK, ZED 2 Camera, and Voxblox, to name a few, as well as their dependencies. This code is to be cloned in the `src` directory of your catkin workspace, and compiled.

Note that the ZED 2 camera requires an Nvidia GPU, so while these instructions may work for any linux computer, only computers with Nvidia GPUs will be able to connect to a ZED 2 camera. All other computers will be able to work only with ROS Bags (recordings) of the ZED 2.

## Installation

### Prerequisites

- Ubuntu 18.04
- [ZED SDK](https://www.stereolabs.com/developers/release/) (tested with 3.4.0, CUDA 10.2). Please upgrade if your version is earlier than 3.4.0
- ROS Melodic (see below for details)
- DJI Onboard SDK ("DJI OSDK") See below for instructions

##### Installing ZED SDK (Prerequisite)
Use the link above for downloading ZED SDK
- If you're on Jetson, download the release for Jetson

##### Installing ROS Melodic on Ubuntu 18.04 (Prerequisite)
Follow these [installation instructions](http://wiki.ros.org/melodic/Installation/Ubuntu) but keep in mind these notes:
- Install the "ros-melodic-desktop-full" version of ROS
- After running the command that installs that version of ROS, also run `sudo apt-get install python-catkin-tools python-catkin-pkg`
- We're using python 2.x Do not do follow instructions for python3
- Any instruction that refers to “kinetic” or “noetic”, replace it with “melodic”. The names kinetic and noetic are other versions of ROS
- Any place that makes a distinction between *catkin* and *rosbuild*, use "catkin" as its the newer version


##### Installing DJI Onboard-SDK (Prerequisite)
- Just like we installed the ZED SDK already (which has nothing to do with ROS), we do the same thing and install DJI's SDK, which has no relation to ROS. The ROS packages/wrappers come later on top of the SDKs. So now, before installing the DJI SDK ROS wrapper, we need to first install the *DJI SDK*. It's like first building the engine before we put it in the complete, ready-to-use car (The car being the wrapper for the engine). We have done the same thing with ZED SDK--first we installed the ZED SDK and soon we will install the ZED ROS Wrapper (aka package).
- `cd ~`
- You may navigate to and install this in any directory. I like to install stuff in my code directory I made in $HOME . So I'll do `cd ~/code/`
- `git clone https://github.com/dji-sdk/Onboard-SDK`
- `cd Onboard-SDK`
- `mkdir build`
- `cd build`
- `cmake ..`
- `make djiosdk-core`
- `sudo make install`

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
- `cd drone-jetson`
- `git submodule update --init --recursive` Clones all the submodules, which are git repos within this git repo. Each submodule is a ROS package.
- Good job!
- The next step is to compile, but do note it takes up quite a bit of RAM. Please watch the RAM usage as you compile so your computer doesn't freeze on you. To limit the RAM usage during compilation, you have two options. The smooth-brain way is to kill `ctrl+c` the compilation process whenever the RAM usage is increasing past 90-95% of your total RAM. Then, **re-run** the compilation process (using the same command). It will pick up where it left off, quickly checking off the stuff it has aleady compiled, and resuming! In addition, you can also limit your RAM usage by closing all other programs, or if using a VM, increase its RAM allowance -- 3-4GB of RAM has worked for some people. I followed the smooth-brain way to stay below my 8gb of RAM. So that's one way to do this. The other way, the wrinkly-brain way, of limiting RAM usage is learning to use these [commands/flags](https://github.com/catkin/catkin_tools/issues/167).
- `cd ..` Before compiling, you should be back in the directory *ros_catkin_ws*
- `catkin build` Compiles all the packages
- `source devel/setup.bash` this tells ROS to use this workspace. Run this command on every new terminal you open for running ROS stuff. Alternatively, add `cd $HOME/ros_catkin_ws && source devel/setup.bash` to your `.bashrc` file so it will run this command for you every time you open a new terminal.



### Installing other ROS Packages
- ROS Bridge, for sending data to other computers on the network, namely VR interface (or ISAACS server in the future). `sudo apt-get install ros-melodic-rosbridge-server`

## Tests

### Voxblox (and ROS) Test
First, let's make sure ROS is working AND test voxblox:
- open a terminal, (let's call it T1), source the workspace as shown above, and run: `roscore`
- It will print out info about what's running and then quiet down. Now ROS core is fully up and running. Remember, ROS Core is the thing that helps all the nodes pass messages between each other. Great, ROS works!
- **Leave T1 with roscore running!!**

Now, to make sure Voxblox is working, we will follow the Voxblox running instructions they have [here](https://voxblox.readthedocs.io/en/latest/pages/Running-Voxblox.html). You will see how to visualize Voxblox Meshes in the commonly-used visualizer program called "rviz"! The code you run here will (under the hood) take in a point-cloud dataset (ROS bag), use voxblox to convert it to meshes, and publish the meshes on the voxblox_mesh topic. Finally, you'll use rviz to visualize those meshes!
<br> **Don't forget to source your workspace for every terminal you open!**
1. They say to download one of the datasets -- and they are big! (note some contain textured data, some do not -- textured data is just cooler!)
- Notice the downloaded file is a `.bag` file -- a ROS bag. It's simply a recording of data. ROS Bags are used for storing data, and replaying data at a later time, as if it was live-captured.
2. They say to "edit the path to the bagfile in cow_and_lady_dataset.launch"
- Do so by opening another terminal (let's call it T2) and running `roscd voxblox_ros/launch` 
- Edit the launch file so the path points to your dataset. **The path should be an absolute path to the bag rather rather than a relative path or a path using the $HOME symbol `~`** Make sure you're editing the right launch file for the dataset you downloaded
3. Run the dataset launch file. 
- If using the the "cow & lady dataset", run: `roslaunch voxblox_ros cow_and_lady_dataset.launch`
- The ROS tutorials should have covered what this type of command does hopefully, but basically 1) `roslaunch` runs a launch file AND starts ROS core (if it hasn't already been started), 2) specifies which package the launch file is in, and 3) specifies which launch file to launch.
- Now, stuff should play in T2 for a while. If it stops before we get to the end of this test, simply re-launch the launch file. It's playing back data so you can re-replay it as much as you'd like. 
4. They say to 1) **open rviz** and 2) see the the mesh visualized on the **/voxblox_node/mesh topic** in the **world static frame**
- Do so by opening a 3rd terminal (T3), source the workspace of course, and run: `rviz` -- a beloved visualizer program!
- Now, we'll add the ROS Topic we want to visualize. Near the bottom-left, click **"Add"**, then **"By Topic"**, then scroll down and select **"Voxblox Mesh"**. Click **"OK"**. If "voxblox" is not there, re-launch roslaunch command in T2 again, and try again -- click "Add" again. If it's still not there, kill rviz and source the workspace in T3.
- Now in the left menu/region (called "Displays") click on **"Fixed Frame"** and select (or type in) **"world"**
5) If T2 is still printing out stuff, then you should see voxblox meshes get visualized! If no meshes are showing, re-launch the command in T2.

### ROS-Bridge + Unity Test
This test will make sure you're able to connect Unity to the on-board computer over a ROS Bridge Server connection. If successful, you'll be able to see the dataset you've visualized in rviz show up in Unity!
To be continued...

## Help

### Troubleshooting
- Just try this again `source ~/ros_catkin_ws/devel/setup.bash`
- Restart your computer
- **Compilation failing? "Could not find a package..."** Looks like you're still missing a dependency! Google the name of it + "ros" and find how to install it. Also try searching for it in the [ROS Index](https://index.ros.org/). It may be a ROS package or a system dependency (choose accordingly during the search)
- **Compilation Failing after git pull? Did someone add a new package?** Maybe the new package you pulled was a git submodule. So run this again from the instructions above: `git submodule update --init --recursive`
- **Installing a new ROS package in this repo?** Have its dependencies automatically downloaded using `rosdep install --from-paths src --ignore-src -r -y` found [here](http://wiki.ros.org/rosdep#Install_dependency_of_all_packages_in_the_workspace)
- Possibly need [system depedency](https://index.ros.org/search/?term=sdl2&section=deps) like SDL2 for compilation: `sudo apt-get install libsdl2-dev`
- If all else fails, restart your computer

### Tips & tricks
- **Want to see info about *catkin build* beforehand?** Use `catkin build --dry-run`
- **If you want to install new ros packages:** that don't have installation instructions, they would either be a git repo you need to clone, or the BETTER way is through an apt-get installation, like we installed **ros bridge** above. Sometimes you gotta guess the name so usually it follows this format `sudo apt-get install ros-<ROS_version>-<package_name>`. Google around!





