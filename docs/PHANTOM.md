# PHANToM Omni Setup (Linux, ROS 2, and SAW)
> [!WARNING]
> Install jazzy, not humble. Install libgraphviz-dev and graphviz.


This guide describes how to connect the original **SensAble PHANToM Omni (FireWire / IEEE 1394)** to Linux, install the vendor driver and OpenHaptics SDK, build the JHU cisst/SAW integration, and read the device through ROS 2.

The intended end result is:

1. Linux detects the Omni over FireWire.
2. `PHANToMConfiguration` detects and configures the device.
3. The SAW example can initialize the device.
4. The `sensable_phantom` ROS 2 node publishes the device state.
5. A Python application can subscribe to the state and, when appropriate, send commands.

## Contents

- [1. Hardware & Software Prerequisites](#1-hardware-and-software-prerequisites)
- [2. Install the FireWire Driver & OpenHaptics](#2-install-the-firewire-driver-and-openhaptics)
- [3. Verify FireWire Detection & Permissions](#3-verify-firewire-detection-and-permissions)
- [4. Configure the OMNI](#4-configure-the-omni)
- [5. Install ROS 2](#5-install-ros-2)
- [6. Install Workspace Tools & System Dependencies](#6-install-workspace-tools-and-system-dependencies)
- [7. Create ROS 2 Workspace & Import SAW Dependencies](#7-create-the-ros-2-workspace-and-import-saw-dependencies)
- [8. Install ROS Dependencies & Build](#8-install-ros-dependencies-and-build)
- [9. Test the Standalone Qt Application](#9-test-the-standalone-qt-application)
- [10. Run the ROS 2 Node](#10-run-the-ros-2-node)
- [11. Inspect the Published Topics](#11-inspect-the-published-topics)
- [12. Access the Device from Python](#12-access-the-device-from-python)
- [13. Quick Setup Guide](#13-quick-setup-guide)
- [14. Troubleshooting](#13-troubleshooting)
- [15. References](#14-references)

---

## 1. Hardware & Software Prerequisites

### Hardware

- Original SensAble PHANToM Omni with its power supply.
- Working FireWire (IEEE 1394) connection between the Omni and the Linux computer.
- A FireWire controller supported by the Linux kernel.

The original Omni uses FireWire. It is not the same connection as the USB- or Ethernet-based Geomagic/3D Systems Touch devices, which use different driver instructions.

FireWire adapters can be difficult to source for laptops. Confirm that the computer has a compatible FireWire controller before troubleshooting software.

### Software

For the documented baseline, use:

| Component | Recommended version |
|---|---|
| Operating system | Ubuntu 22.04 LTS (64-bit) |
| ROS | ROS 2 Humble |
| Build system | `colcon` |
| Multi-repository tool | `vcstool` |
| Device integration | JHU cisst/SAW `sawSensablePhantom` |
| Low-level device software | SensAble driver and OpenHaptics SDK |

Check the operating system and architecture:

```bash
lsb_release -a
uname -r
uname -m
```

For the recommended setup, `uname -m` should report `x86_64`.

## 2. Install the FireWire Driver & OpenHaptics

The JHU driver repository provides an installation script for the original FireWire PHANToM Omni. It installs the device driver and OpenHaptics software required by the SAW component.

Repository: [jhu-cisst-external/phantom-omni-1394-drivers](https://github.com/jhu-cisst-external/phantom-omni-1394-drivers)

Clone the repository:
```bash
mkdir -p ~/phantom-omni-1394-drivers     # get the files
cd ~/phantom-omni-1394-drivers

git clone https://github.com/jhu-cisst-external/phantom-omni-1394-drivers.git
cd phantom-omni-1394-drivers
```

Run the installation script
```bash
sudo ./install-omni-drivers.sh # finally run the install script
sudo rm -rf Linux_JUJU_PDD_64-bit OpenHapticsAE_Linux_v3_0  # this is just to clean temporary files
```

## 3. Verify FireWire Detection & Permissions

Power on the Omni and connect its FireWire cable.

In a terminal, monitor kernel messages:

```bash
sudo dmesg -w
```

You can also run this before reconnecting the device so that new messages are easy to identify.

A successful connection should produce a message similar to:

```text
firewire_core: created device fw1:
GUID: 000b990082b6980c, S400
```

The device node may be `/dev/fw0`, `/dev/fw1`, or another number. Do not assume it will always be `fw0`.

List the FireWire device nodes:

```bash
ls -l /dev/fw*
```

The JHU driver instructions configure the FireWire device node with the `plugdev` group. An example permission line is:

```text
crw-rw---- 1 root plugdev ... /dev/fw1
```

If the device node belongs to `plugdev`, add your Linux user to that group:

```bash
sudo usermod -aG plugdev "$USER"
```

Log out and log back in for the new group membership to take effect. Then verify:

```bash
groups
ls -l /dev/fw*
```

You should be able to access the relevant `/dev/fwN` device without running ordinary user applications as root.

## 4. Configure the OMNI

After Linux detects the FireWire device, run the vendor configuration utility:

```bash
sudo PHANToMConfiguration
```

In the configuration window:

1. Select **Add**.
2. Select the PHANToM model.
3. Choose the **Omni** arm type, if prompted.
4. Give the device a useful name, such as `left`.
5. Select **Apply**.
6. Select **OK** to close the application.

The device serial number should appear in the configuration utility when the hardware is detected correctly.

The device name matters because it is used as the ROS namespace for the published topics. For example, a device named `left` will normally publish topics under `/left`.

If the configuration utility reports a missing `libglw` library, the upstream driver notes suggest installing:

```bash
sudo apt install libglw1-mesa
```

The package name may differ on other Ubuntu releases.

At this point, Linux and the vendor configuration utility should recognize the Omni.

## 5. Install ROS 2

For the recommended Ubuntu 22.04 setup, install **ROS 2 Humble**.

Follow the official ROS 2 instructions for Ubuntu 22.04: [ROS 2 Humble — Ubuntu (Debian packages)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

The official instructions cover locale configuration, adding the ROS package repository and key, and installing the ROS packages. Prefer those maintained instructions over older guides that use `apt-key` or obsolete ROS distributions.

### 5.1 Set locale
Make sure you have a locale which supports `UTF-8`. If you are in a minimal environment (such as a docker container), the locale may be something minimal like `POSIX`. We test with the following settings. However, it should be fine if you’re using a different UTF-8 supported locale.
```shell
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

### 5.2 Setup Sources
You will need to add the ROS 2 apt repository to your system.

First ensure that the [Ubuntu Universe repository](https://wiki.ubuntu.com/Repositories/Ubuntu) is enabled.
```shell
sudo apt install software-properties-common
sudo add-apt-repository universe
```

The [ros-apt-source](https://github.com/ros-infrastructure/ros-apt-source/) packages provide keys and apt source configuration for the various ROS repositories.

Installing the ros2-apt-source package will configure ROS 2 repositories for your system. Updates to repository configuration will occur automatically when new versions of this package are released to the ROS repositories.

```shell
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -fsSL -o /dev/null -w '%{url_effective}' https://github.com/ros-infrastructure/ros-apt-source/releases/latest | awk -F/ '{print $NF}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

### 5.3 Install ROS 2 packages
Update your apt repository caches after setting up the repositories.
```shell
sudo apt update
```

ROS 2 packages are built on frequently updated Ubuntu systems. It is always recommended that you ensure your system is up to date before installing new packages.
```shell
sudo apt upgrade
```

> [!WARNING]
> Due to early updates in Ubuntu 22.04 it is important that `systemd` and `udev-related` packages are updated before installing ROS 2. The installation of ROS 2’s dependencies on a freshly installed system without upgrading can trigger the **removal of critical system packages.**

Desktop Install (Recommended): ROS, RViz, demos, tutorials.
```shell
sudo apt install ros-humble-desktop
```

### 5.4 Environment setup
After configuring the ROS apt repository and installing the desktop distribution and development tools as described in the official guide, source ROS 2 in the current terminal:

```bash
source /opt/ros/humble/setup.bash
```

Verify the environment:

```bash
printenv ROS_DISTRO
ros2 --help
```

`ROS_DISTRO` should report `humble`.

## 6. Install Workspace Tools & System Dependencies

The JHU `vcs` repository provides manifests for downloading the cisst/SAW repositories and their matching revisions.

Install the workspace tools:

```bash
sudo apt update
sudo apt install -y git python3-vcstool python3-colcon-common-extensions python3-rosdep python3-pykdl
```

Install the additional Ubuntu dependencies listed by the JHU cisst/SAW build instructions for Ubuntu

```bash
sudo apt install libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion gfortran libcppunit-dev libqt5xmlpatterns5-dev libbluetooth-dev libhidapi-dev python3-pyudev gfortran-9 ros-humble-joint-state-publisher* ros-humble-xacro
```

Some packages may already be installed. If `apt` cannot find a package, check the exact package name and your enabled Ubuntu repositories rather than silently skipping a required dependency.

Initialize `rosdep` if it has not already been initialized:

```bash
sudo rosdep init
rosdep update
```

If `rosdep init` reports that the configuration already exists, do not initialize it again; run `rosdep update` instead.

### Python environment note

Build ROS and cisst/SAW using the system Python environment. If you use Conda or a Python virtual environment, deactivate it before building:

```bash
deactivate  # only if a virtual environment is active
```

Check which Python interpreter is active:

```bash
which python3
python3 --version
```

For a normal Ubuntu installation, `python3` should resolve to the system interpreter under `/usr/bin`.

## 7. Create ROS 2 Workspace & Import SAW Dependencies

Create a ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Import the SensAble manifest from the JHU `vcs` repository:

```bash
vcs import --recursive --input https://raw.githubusercontent.com/jhu-saw/vcs/main/ros-sensable-main.vcs
```

The manifest imports the repositories and revisions needed for this component, including:
- `cisstNetlib`
- `cisst`
- `cisst-ros`
- `sawKeyboard`
- `sawControllers`
- `sawSensablePhantom`
- `crtk_msgs`
- `crtk_python_client`

The manifest is maintained by JHU:

[JHU SAW VCS repository](https://github.com/jhu-saw/vcs)

If the import fails because of a transient network or SSH-related error, retry with fewer parallel workers:

```bash
vcs import --workers 1 --retry 10 --recursive --input https://raw.githubusercontent.com/jhu-saw/vcs/main/ros-sensable-main.vcs
```

## 8. Install ROS Dependencies & Build

Make sure the ROS 2 environment is sourced:

```bash
source /opt/ros/humble/setup.bash
```

Move to the workspace root:

```bash
cd ~/ros2_ws
```

Install dependencies declared by the packages in the workspace:

```bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

Build the workspace in Release mode:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

The first build may take some time because cisst/SAW consists of multiple repositories and libraries.

After a successful build, source the workspace overlay:

```bash
source ~/ros2_ws/install/setup.bash
```

Verify that ROS can find the package:

```bash
ros2 pkg list | grep -E 'sensable|cisst|crtk'
```

Check which executables were installed:

```bash
ros2 pkg executables sensable_phantom
```

If you open a new terminal, source both the ROS installation and the workspace overlay again:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

## 9. Test the Standalone Qt Application

The repository provides a standalone graphical application:

```bash
sawSensablePhantomQtExample -D
```

The `-D` option enables the dark Qt palette. This is a standalone cisst/SAW example; it is separate from launching the ROS node.

To configure a named device, create a JSON file such as `sawSensablePhantom.json`:

```json
{
    "devices": [
        {
            "name": "left"
        }
    ]
}
```

Run the application with that configuration:

```bash
sawSensablePhantomQtExample -D -j sawSensablePhantom.json
```

The `name` should match the name assigned in `PHANToMConfiguration`.

The upstream README documents these options:

| Option | Description |
|---|---|
| `-j <file>`, `--json-config <file>` | Optional device configuration JSON |
| `-m`, `--component-manager` | Optional component-manager configuration |
| `-D`, `--dark-mode` | Use a dark Qt palette |

If the application cannot initialize the device, return to the FireWire and vendor-driver checks before debugging ROS.

## 10. Run the ROS 2 Node

The ROS package is named `sensable_phantom`, and its executable is also named `sensable_phantom`.

First, make sure both environments are sourced:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Confirm the executable is available:

```bash
ros2 pkg executables sensable_phantom
```

For a single configured Omni, launch the node with:

```bash
ros2 run sensable_phantom sensable_phantom
```

To enable the dark Qt palette:

```bash
ros2 run sensable_phantom sensable_phantom --ros-args -- -D
```

To pass a device configuration file:

```bash
ros2 run sensable_phantom sensable_phantom --ros-args -- -j /absolute/path/to/sawSensablePhantom.json
```

The `--ros-args --` separator tells ROS 2 to stop parsing ROS arguments and pass the remaining arguments to the executable. If your ROS 2 distribution or executable wrapper handles arguments differently, inspect the executable's `--help` output.

The node supports the following application-specific options in the current source:

| Option | Description |
|---|---|
| `-j <file>`, `--json-config <file>` | Optional device configuration JSON |
| `-p <seconds>`, `--ros-period <seconds>` | Device-state publishing period; default is `0.002` seconds (2 ms / 500 Hz) |
| `-P <seconds>`, `--tf-ros-period <seconds>` | TF publishing period; default is `0.02` seconds (20 ms / 50 Hz) |
| `-m <file>`, `--component-manager <file>` | Optional component-manager configuration; may be specified more than once |
| `-D`, `--dark-mode` | Use a dark Qt palette |

The ROS publishing period is not a guarantee that the physical device itself updates at that rate. The upstream documentation notes that increasing the publishing rate beyond the device's update rate provides no benefit.

## 11. Inspect the Published Topics

Once the ROS node is running and the device is connected, list the topics:

```bash
ros2 topic list
```

For a device named `left`, the upstream README lists topics similar to:

```text
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

The device-specific namespace depends on the configured device name. If you named the Omni something other than `left`, expect the corresponding namespace to differ.

Read the measured Cartesian pose:

```bash
ros2 topic echo /left/measured_cp
```

Inspect a topic's type and publisher information:

```bash
ros2 topic info /left/measured_cp
```

Read the button state:

```bash
ros2 topic echo /left/button1
ros2 topic echo /left/button2
```

The `measured_*` topics report measured device state. The `servo_*` topics are command interfaces; do not publish to them until you understand the device's operating state and force/position-control behavior.

## 12. Access the Device from Python

The repository includes a CRTK-based Python client in the `sensable_phantom` package and example scripts under: [`sawSensablePhantom/ros/scripts`](https://github.com/jhu-saw/sawSensablePhantom/tree/main/ros/scripts)

The current Python wrapper is initialized with a CRTK RAL object and the device's arm name. Its interface is not the older one-argument example sometimes shown in notes.

The upstream example follows this pattern:

```python
import crtk
import sensable_phantom

ral = crtk.ral("my_phantom_client")

arm = sensable_phantom.arm(
    ral=ral,
    arm_name="left",
    expected_interval=0.01
)
```

The `arm_name` must match the device namespace. The example scripts show how to initialize the client, check connections, read measured Cartesian position, and issue commands.

Useful upstream example:

[`xyz-motions.py`](https://github.com/jhu-saw/sawSensablePhantom/blob/main/ros/scripts/xyz-motions.py)

## 13. Quick Setup Guide
Assuming steps 1-6 are complete.

### 13.1. Navigate to the workspace

```bash
cd ~/phantom_ws
```

Expected structure:

```text
phantom_ws/
├── src/                    # ROS 2 dependencies and SAW repositories
├── .venv/                  # Python virtual environment
├── build/
├── requirements.txt
├── install/
└── log/
...
```

### 13.2. Set up the Python virtual environment

Source ROS 2 first:

```bash
source /opt/ros/humble/setup.bash
```

Create the virtual environment if it doesn't already exist:

```bash
python3 -m venv --system-site-packages .venv
```

**Note:** `--system-site-packages` allows the virtual environment to access ROS 2's installed Python modules, including packages such as `rclpy`.

Activate it:

```bash
source .venv/bin/activate
```

Upgrade pip:

```bash
python -m pip install --upgrade pip
```

### 13.3. Install Python dependencies
```bash
python -m pip install -r phantom_mujoco/requirements.txt
```

### 13.4. Install ROS dependencies

Make sure ROS 2 and the virtual environment are active:

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
```

Update rosdep and install dependencies for the workspace:

```bash
sudo rosdep init  # Only needed once per machine
rosdep update

rosdep install --from-paths src --ignore-src -r -y
```

### 13.5. Build the workspace

From the workspace root:

```bash
colcon build --symlink-install
```

Source the resulting workspace:

```bash
source install/setup.bash
```

### 13.6. Run the PHANToM interface

With the environment sourced, start the PHANToM ROS 2 node:

```bash
ros2 run sensable_phantom sensable_phantom -D
```

Alternatively, specify a configuration file:

```bash
ros2 run sensable_phantom sensable_phantom -j /absolute/path/to/sawSensablePhantom.json
```

Keep this terminal running.

### 13.7. Run your Python implementation

Open a second terminal:

```bash
cd ~/phantom_ws

source /opt/ros/humble/setup.bash
source .venv/bin/activate
source install/setup.bash

python /absolute/path/to/main.py
```

## 14. Troubleshooting

### No `/dev/fw*` device appears

- Confirm the Omni is powered on.
- Check the FireWire cable and adapter.
- Confirm that the computer has a FireWire controller.
- Inspect `sudo dmesg -w` while reconnecting the device.
- Check whether the Linux kernel has detected the FireWire controller.

### The device appears, but the application cannot access it

- Check the permissions with `ls -l /dev/fw*`.
- Confirm the relevant device node belongs to `plugdev`.
- Add your user to `plugdev` if needed.
- Log out and back in after changing group membership.

### `PHANToMConfiguration` cannot find the device

- Confirm that the FireWire device appears in `/dev`.
- Check the vendor driver installation output.
- Run the configuration utility with `sudo`.
- Confirm that the correct PHANToM model is selected.
- Check the upstream driver notes for missing libraries such as `libglw`.

### `sawSensablePhantomQtExample` is not found

- Confirm the workspace built successfully.
- Source `/opt/ros/humble/setup.bash`.
- Source `~/ros2_ws/install/setup.bash`.
- Check whether the executable was installed and whether the SAW component was found during CMake configuration.

### `ros2 pkg executables sensable_phantom` returns nothing

- Check the `colcon build` output for the `sensable_phantom` package.
- Confirm that the ROS 2 environment was sourced before building.
- Confirm that the workspace overlay was sourced after building.
- Check that you imported the `ros-sensable-main.vcs` manifest.

### The build appears to select ROS 1 instead of ROS 2

The package's CMake configuration checks for `catkin` before checking for `ament_cmake`. Do not build in a terminal that has a ROS 1 environment sourced.

Open a clean terminal, source only ROS 2, and rebuild:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Python cannot import `sensable_phantom` or `crtk`

- Source the ROS 2 installation and workspace overlay in the Python terminal.
- Use the system Python environment used for the ROS build.
- Confirm that the Python package was installed by the build.
- Check the imports and initialization pattern in the current upstream scripts.

---

## 15. References

- [sawSensablePhantom repository](https://github.com/jhu-saw/sawSensablePhantom)
- [sawSensablePhantom driver instructions](https://github.com/jhu-saw/sawSensablePhantom/blob/main/drivers.md)
- [PHANToM Omni FireWire driver installer](https://github.com/jhu-cisst-external/phantom-omni-1394-drivers)
- [JHU SAW VCS repository](https://github.com/jhu-saw/vcs)
- [ROS 2 Humble installation on Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [ROS 2 colcon tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
