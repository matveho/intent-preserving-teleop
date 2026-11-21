## 1 Install FireWire drivers & OpenHaptics
Clone the [PHANToM OMNI installation repository](https://github.com/jhu-cisst-external/phantom-omni-1394-drivers) to install the PHANToM OMNI (FireWire version, i.e. 1394) driver and OpenHaptics SDK. Run the install script with sudo privileges.

### 1.1 Install
Install script (you will need sudo privileges):
```bash
mkdir -p ~/phantom-omni-1394-drivers     # get the files
cd ~/phantom-omni-1394-drivers

git clone https://github.com/jhu-cisst-external/phantom-omni-1394-drivers
cd ~/phantom-omni-1394-drivers/phantom-omni-1394-drivers

sudo ./install-omni-drivers.sh # finally run the install script
sudo rm -rf Linux_JUJU_PDD_64-bit OpenHapticsAE_Linux_v3_0  # this is just to clean temporary files
```

### 1.2 Check Connection & Device Permissions
In another terminal run
```shell
sudo dmesg -w
```

You should see a device registration resembling:
```shell
firewire_core: created device fw1:
GUID: 000b990082b6980c, S400
```

Check the device permissions:
```shell
ls -l /dev/fw*
```

If your device belongs to the `plugdev` group, add your user:
```shell
sudo usermod -aG plugdev "$USER"
```

Log out and back in.

### 1.3 Configure the Omni
Use the program `PHANTOMConfiguration` *with sudo privileges*.   Click "Add", enter a name for the device, select the arm type (i.e. Omni) and then click "Apply" and "OK" to quit. 

```shell
sudo PHANToMConfiguration
```

## 2 Setup ROS 2
Follow [the directions here](https://docs.ros.org/en/crystal/Installation/Linux-Install-Binary.html) to install ROS 2 on linux..

### 2.1 Add the ROS 2 apt repository
```shell
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

And then add the repository to your sources list:
```shell
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
```

### 2.2 Downloading ROS 2
- Go [the releases page](https://github.com/ros2/ros2/releases)
- Download the latest package for Linux; let’s assume that it ends up at ~/Downloads/ros2-crystal-linux-x86_64.tar.bz2.
  - Note: there may be more than one binary download option which might cause the file name to differ.
- Unpack it:
```shell
mkdir -p ~/ros2_crystal
cd ~/ros2_crystal
tar xf ~/Downloads/ros2-crystal-linux-x86_64.tar.bz2
```

### 2.3 Installing and initializing rosdep
```shell
sudo apt update
sudo apt install -y python-rosdep
sudo rosdep init # if already initialized you may continue
rosdep update
```

### 2.4 Installing the missing dependencies
Set your rosdistro according to the release you downloaded.
```shell
CHOOSE_ROS_DISTRO=crystal # or bouncy
rosdep install --from-paths ros2-linux/share --ignore-src --rosdistro $CHOOSE_ROS_DISTRO -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 osrf_testing_tools_cpp poco_vendor rmw_connext_cpp rosidl_typesupport_connext_c rosidl_typesupport_connext_cpp rti-connext-dds-5.3.1 tinyxml_vendor tinyxml2_vendor urdfdom urdfdom_headers"
```

#### 2.4.1 Installing the python3 libraries
```shell
sudo apt install -y libpython3-dev
```

### 2.5 Sourcing the setup script
Set up your environment by sourcing the following file. After installation, verify:
```shell
source ~/ros2_crystal/ros2-linux/setup.bash
ros2 --version
```

## 3 Download the SAW dependencies
The `JHU` team maintains a [repository](https://github.com/jhu-saw/vcs) called `vcs` containing version-controlled dependency manifests.

### 3.1 Requirements
This has to be done once per computer. The following packages might not be installed by default, Install the workspace tools:
```shell
sudo apt install python3-vcstool python3-colcon-common-extensions python3-pykdl
```

For cisst/SAW, you will also need the following Ubuntu packages:
```shell
sudo apt install libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion gfortran libcppunit-dev libqt5xmlpatterns5-dev libbluetooth-dev libhidapi-dev python3-pyudev gfortran-9 ros-humble-joint-state-publisher* ros-humble-xacro
```

### 3.2 Initialization
Initialize rosdep if you haven't already:
```shell
sudo rosdep init
rosdep update
```

If rosdep init says it is already initialized, that's fine.

### 3.3 Create Workspace
Create your ROS 2 workspace
```shell
source ~/ros2_crystal/ros2-linux/setup.bash
mkdir -p ~/ros2_ws/src
```

### 3.4 Download Repositories
Clone all repositories using `vcs`:
```shell
cd ~/ros2_ws/src
vcs import --recursive --input https://raw.githubusercontent.com/jhu-saw/vcs/main/ros2-sensable-main.vcs
```
This should download `cisst`, `cisstNetlib`, `cisst-ros`, `sawSensablePhantom`, `sawControllers`, `crtk_msgs` and `crtk_python_client`.

### 3.5 Install Dependencies & Compile
```shell
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 3.6 After Compilation
Switch source
```shell
source ~/ros2_ws/install/setup.bash
```

Verify that the packages were discovered:
```shell
ros2 pkg list | grep -E 'sensable|cisst|crtk'
```

### 4 Run the PHANToM example
The repository provides a standalone graphical application:
```shell
sawSensablePhantomQtExample -D
```

You can also configure a named device using a JSON configuration:
```json
{
    "devices": [
        {
            "name": "omni"
        }
    ]
}
```

Then:
```shell
sawSensablePhantomQtExample -D -j sawSensablePhantom.json
```

These are the example's [documented launch options](https://github.com/jhu-saw/sawSensablePhantom).

### 5 Access the Omni through ROS 2
The ROS node is `sensable_phantom` and can be found in the package `sensable_phantom`. If you only have one Omni and don't need a configuration file:

### 5.1 Inspecting Packages
Once the standalone example works, we can use the ROS interface.

For ROS 2, first inspect the available executables:
```shell
ros2 pkg list | grep sensable
```

You may inspect the package:
```shell
ros2 pkg executables sensable_phantom
```

### 5.2 Running The Phantom
Run the sensable phantom node
```shell
rosrun sensable_phantom sensable_phantom
```

The ROS node has one more command line option:
```shell
 -j <value>, --json-config <value> : json configuration file (optional)
 -p <value>, --ros-period <value> : period in seconds to read all tool positions (default 0.002, 2 ms, 500Hz).  There is no point to have a period higher than the device (optional)
 -m, --component-manager : JSON files to configure component manager (optional)
 -D, --dark-mode : replaces the default Qt palette with darker colors (optional)
```

### 5.2 Output
Once the node is started AND connected, the following ROS topics should appear:
```shell
ros2 topic list

/left/button1
/left/button2
/left/measured_cp
/left/measured_cv
/left/measured_js
/left/operating_state
/left/servo_cf
/left/servo_cp
/left/state_command
/stats/events/period_statistics
/stats/left/period_statistics
/stats/publishers_left/period_statistics
/stats/subscribers/period_statistics
```

The documented default publishing period is 2 ms (500 Hz).

You can inspect a topic with:
```shell
ros2 topic echo /left/measured_cp
```

### 6 Python client
After building the code using `colcon build`, make sure the source path is set
```shell
source ~/ros2_ws/install/setup.bash
```

Then, you can use Python to do something like:
```shell
import sensable
arm = sensable.arm('/arm') # the parameter is the ROS namespace for your device
arm.measured_cp() # returns a PyKDL frame with the current cartesian position
```

Examples [can be found here](https://github.com/jhu-saw/sawSensablePhantom/tree/main/ros/scripts).