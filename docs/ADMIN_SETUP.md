# Administrator setup checklist

Computer: `csc-305-w06` · Users: `mokoneshnikov`, `phantom` · Shared device-access group: `cmput_312` (GID 6000).

installed: PHANToM driver/SDK, ROS 2 Jazzy, the main C++/Qt build tools, rosdep initialization, and a working NVIDIA driver.
needed: saved PHANToM configuration, a missing system OpenHaptics runtime dependency, and broader device access if all lab users should use it.

1. Install the missing OpenHaptics compatibility libraries

```bash
sudo apt install \
  /home/mokoneshnikov/projects/phantom-omni-1394-drivers/staging/libtinfo5_6.3-2_amd64.deb \
  /home/mokoneshnikov/projects/phantom-omni-1394-drivers/staging/libncurses5_6.3-2_amd64.deb
sudo ldconfig
```

Installs the legacy runtime needed by OpenHaptics for all users. The system `libHD.so` currently reports `libncurses.so.5 => not found`. These are the already-staged Ubuntu compatibility packages; on another machine, copy the two packages and adjust their paths. Do not substitute a symlink to libncurses6: it is a different ABI.

2. Allow lab users to access the PHANToM

```bash
getent group cmput_312
printf '%s\n' 'SUBSYSTEM=="firewire", ATTR{vendor}=="0x000b99", MODE="0660", GROUP="cmput_312", TAG+="uaccess"' | sudo tee /etc/udev/rules.d/71-sensable-phantom.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=firewire --attr-match=vendor=0x000b99 --action=change
sudo udevadm settle
```

Grants PHANToM access to the existing course group and to whichever user has the active local desktop session. The rule applies to SensAble devices only and overrides the installed `plugdev` rule. It does not make every FireWire device world-writable. Confirm `cmput_312` exists before installing this rule on another machine.

Add `phantom` to the confirmed course group:

```bash
sudo usermod -aG cmput_312 phantom
```

`mokoneshnikov` already belongs to `cmput_312` and `plugdev`; no membership change is needed. `phantom` currently belongs only to its own primary group and must log out and back in after this command. The device rule above uses `cmput_312`, so adding `phantom` to `plugdev` is unnecessary. Only one application/user should control a given device at a time.

3. Save the device configuration

```bash
sudo install -d -m 0755 /etc/SensAble/PHANToMDeviceDrivers
sudo PHANToMConfiguration
```

```bash
sudo find /etc/SensAble/PHANToMDeviceDrivers -type d -exec chmod a+rx {} +
sudo find /etc/SensAble/PHANToMDeviceDrivers -type f -exec chmod a+r {} +
```

Makes the saved device configuration readable by all users. It remains administrator-managed.

4. system dependencies

```bash
sudo apt update
sudo apt install \
  build-essential cmake git pkg-config \
  python3-dev python3-pip python3-venv \
  qtbase5-dev libqt5opengl5-dev libqt5xmlpatterns5-dev \
  libjsoncpp-dev libxml2-dev libraw1394-dev \
  libgl1-mesa-dev libglu1-mesa-dev libglw1-mesa-dev \
  libmotif-dev libglut-dev libxmu-dev libxi-dev \
  mesa-utils acl \
  python3-colcon-common-extensions python3-vcstool python3-rosdep \
  python3-pykdl ros-jazzy-desktop
```

Ensures shared build, graphics, Python, and ROS dependencies are available.

Use the fixed workspace location `~/ros_ws`, outside the repository. This is `/home/mokoneshnikov/ros_ws` for `mokoneshnikov` and `/home/phantom/ros_ws` for `phantom`. No repository clone or administrator-created directory is needed. Run the following in the intended user's terminal, without sudo:

```bash
mkdir -p ~/ros_ws/src
```

Import the cisst/SAW/CRTK dependencies into `~/ros_ws/src/` before running the commands below. The folder alone does not install those sources. Run these as the normal user; `rosdep install` requests sudo only for missing system packages:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros_ws
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
```

System packages are installed once for all users. Each user owns their workspace and can build without administrator help. Building and sourcing the workspace do not require sudo:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros_ws
colcon build --symlink-install
source install/setup.bash
```

Keep project source, the dependency manifest, and a future workspace setup/sync script in the repository. The script can link project ROS packages into `~/ros_ws/src/` and import external dependencies there; generated `build/`, `install/`, and `log/` stay outside the repository. The setup/sync script has not been created yet and needs no sudo. The separate `admin-audit` directory is not the deployment workspace.

Optional Graphviz support:

```bash
sudo apt install graphviz libgraphviz-dev
```

Installs graph-rendering tools and development libraries for cisst component-graph visualization system-wide. Not required for PHANToM teleoperation or ACT training.

5. Video output and optional headless rendering

```bash
sudo apt install ffmpeg libosmesa6
```

Installs video export tools and an optional software-rendering fallback for all users. MuJoCo already renders through NVIDIA EGL on this machine; OSMesa is not required for state-only training.

6. Gen3 Ethernet connection — robot-control computer only

```bash
nmcli device status
```

Identify the dedicated Ethernet/USB-Ethernet adapter connected to the Gen3. Do not repurpose the campus-network interface `enp128s31f6`. Confirm the robot's IP/subnet; the commands below assume robot address `192.168.1.10` and an unused computer address `192.168.1.11/24`.

```bash
sudo nmcli connection add type ethernet ifname ROBOT_IFACE con-name gen3-lab \
  ipv4.method manual ipv4.addresses 192.168.1.11/24 \
  ipv4.never-default yes ipv6.method disabled \
  connection.autoconnect no connection.permissions ""
sudo nmcli connection up gen3-lab
```

Replace `ROBOT_IFACE` with the dedicated adapter's name. No dedicated robot Ethernet adapter is currently detected; leave this command pending until one is connected. Creates a system-wide robot connection without replacing the Internet route. Add it only if `gen3-lab` does not already exist. Other users' activation rights depend on lab policy. The Kortex Python API can be installed in a virtual environment without sudo; robot credentials and network access must still be verified.

7. Additional training machines

If Python environment support is missing:

```bash
sudo apt install python3-venv python3-pip python3-dev build-essential
```

Installs shared Python/build prerequisites. Install PyTorch and other Python packages in each user's virtual environment, without sudo. Use a Blackwell-compatible CUDA wheel; standard PyTorch wheels do not require a system CUDA toolkit. Training-only machines do not need PHANToM or ROS packages.

If remote access is needed and SSH is missing/inactive:

```bash
sudo apt install openssh-server rsync tmux
sudo systemctl enable --now ssh
```

Enables SSH for authorized users and persistent terminal jobs with tmux; rsync is available for optional file transfers. Project files will be managed through the repository. SSH is already active on this machine. Verify account/firewall access and agree on the lab's suspend/reboot policy for long jobs.

References: [SAW dependencies](https://github.com/jhu-saw/vcs/blob/main/ros-sensable-main.vcs), [Blackwell PyTorch support](https://pytorch.org/blog/pytorch-2-7/), [Gen3 network setup](https://github.com/Kinovarobotics/kortex/blob/master/api_python/examples/readme.md).

Remaining blanks to confirm:
- `ROBOT_IFACE`: the dedicated Gen3 adapter; only campus Ethernet `enp128s31f6` is currently detected.
- Gen3 addresses: `192.168.1.10` and `192.168.1.11/24` are example defaults, not verified lab assignments. Robot credentials also remain unconfirmed; do not put passwords in this document.
- Training-machine hostnames, available user accounts, and Python environment paths: not supplied or checked remotely.

Known local paths: PHANToM configuration `/etc/SensAble/PHANToMDeviceDrivers`; ROS `/opt/ros/jazzy`; standalone build `/home/mokoneshnikov/environments/sensable`; audit Python `/home/mokoneshnikov/environments/admin-audit/venv/bin/python`.

8. Checks

```bash
id
id mokoneshnikov
id phantom
getent group cmput_312
ls -l /dev/fw*
getfacl /dev/fw1
ls -la /etc/SensAble/PHANToMDeviceDrivers
ldd /usr/lib64/libHD.so | grep 'not found'
ldd /usr/sbin/PHANToMConfiguration | grep 'not found'
glxinfo -B
nvidia-smi
```

Run without sudo. At this check only `/dev/fw0` (the controller) is present; reconnect the PHANToM before checking its access. `/dev/fw1` was its previous node; use the actual SensAble device node after reconnecting. Confirm user access, saved configuration files, and no missing-library output. No output and exit code 1 from `grep` means no missing libraries were found.

After importing the ROS workspace sources, run as the workspace owner:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros_ws
rosdep check --from-paths src --ignore-src --rosdistro jazzy
```

Checks system dependencies without installing anything. The audited SAW sources contain outdated ROS 1 references to `catkin` and `rviz`; exclude/update the legacy package metadata and use `rviz2`, rather than installing ROS 1 on Jazzy. All other resolvable system dependencies passed the audit.

In each training machine's actual Python environment:

```bash
python - <<'PYTHON'
import torch
assert torch.cuda.is_available(), 'CUDA unavailable'
print(torch.__version__, torch.version.cuda, torch.cuda.get_device_name(0))
x = torch.randn(512, 512, device='cuda', requires_grad=True)
loss = (x @ x.T).square().mean()
loss.backward()
torch.cuda.synchronize()
assert torch.isfinite(loss) and torch.isfinite(x.grad).all()
print('GPU forward/backward passed')
PYTHON
```

Confirms real GPU computation as a normal user. This passed on csc-305-w06 with PyTorch 2.7.1+cu128 and driver 580.178.04. A MuJoCo 4.0.1 EGL rendering test also passed. No new NVIDIA driver or system CUDA installation is currently indicated here; the other machines remain unverified.

Before the administrator leaves:
- Confirm the SAW application initializes the PHANToM and reports pose/buttons as a normal user; an open window alone is not enough. Resolve any remaining servo scheduler errors.
- Build the intended ROS workspace and confirm the PHANToM node publishes measured state.
- Test GPU computation on every training machine and headless rendering on machines used for visual evaluation.
- Test SSH and repository access using each intended account; keep each user's checkout and Python environment writable by that user.
- Verify Gen3 reachability and a read-only Kortex connection if pursuing the physical extension.

At the Nov 23 audit, Graphviz/dev libraries and the main build tools were installed, but PHANToM configuration was still empty and the system OpenHaptics library still lacked libncurses.so.5. No administrator commands were executed during the audit. Successful PHANToM initialization, the Gen3 connection, and the other training machines must pass the checks above before considering administrator setup complete. New hardware or dependencies may still require administrator access.
