# Remaining administrator setup

Computer: `csc-305-w06.cs.ualberta.ca` · Users: `mokoneshnikov`, `phantom` · Device-access group: `cmput_312` (GID 6000).

Connect from the administrator's terminal:

```bash
ssh -t csc-305-w06.cs.ualberta.ca
```

SSH uses the administrator's current username or their SSH configuration. Use an account authorized for SSH and sudo on this machine; logging in as `mokoneshnikov` does not grant administrator permissions. Run all commands below inside that SSH session. No graphical session or X forwarding is needed.

Already completed: driver/SDK installation, ROS 2 Jazzy, rosdep initialization, most build dependencies, Graphviz and its development package. Both PHANToM INI files are now installed and readable in `/etc/SensAble/PHANToMDeviceDrivers`. The example initialized using the user's configuration and reported calibration required. No configuration GUI needs to be rerun for this checklist.

1. Install the missing OpenHaptics compatibility libraries

```bash
sudo apt install \
  /home/mokoneshnikov/projects/phantom-omni-1394-drivers/staging/libtinfo5_6.3-2_amd64.deb \
  /home/mokoneshnikov/projects/phantom-omni-1394-drivers/staging/libncurses5_6.3-2_amd64.deb
sudo ldconfig
```

Installs the missing runtime system-wide. The isolated example environment already supplies private copies, but system `libHD.so` still cannot find `libncurses.so.5`. Both package files are present at the paths above.

2. Allow both users to access the PHANToM

```bash
sudo usermod -aG cmput_312 phantom
printf '%s\n' 'SUBSYSTEM=="firewire", ATTR{vendor}=="0x000b99", MODE="0660", GROUP="cmput_312", TAG+="uaccess"' | sudo tee /etc/udev/rules.d/71-sensable-phantom.rules
sudo chmod 0644 /etc/udev/rules.d/71-sensable-phantom.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=firewire --attr-match=vendor=0x000b99 --action=change
sudo udevadm settle
```

Grants access through the existing course group, including over SSH, and to the active local desktop user. `mokoneshnikov` already belongs to this group. `phantom` must log out and back in, including reconnecting SSH, after the change. The rule applies only to SensAble devices and will also apply when the device is reconnected.

3. Install the remaining build and diagnostic packages

```bash
sudo apt update
sudo apt install libglw1-mesa-dev libmotif-dev libglut-dev mesa-utils
```

Completes the original dependency list: development libraries for vendor graphics/examples and OpenGL diagnostic tools. These are not required to run the already-built isolated example, but support rebuilding and troubleshooting without another administrator visit.

The ROS workspace at `~/ros_ws` still needs to be created, populated, and built by each user without sudo. Further system dependencies discovered during that build would need administrator installation; the ROS build has not yet been verified. [Deferred setup](ADMIN_SETUP_DEFERRED.md) covers video export, Gen3 networking, and other training machines.

4. Checks

```bash
dpkg-query -W -f='${Package}: ${Status}\n' libtinfo5 libncurses5 libglw1-mesa-dev libmotif-dev libglut-dev mesa-utils
env -u LD_LIBRARY_PATH ldd /usr/lib64/libHD.so
env -u LD_LIBRARY_PATH ldd /usr/sbin/PHANToMConfiguration
id mokoneshnikov
id phantom
cat /etc/udev/rules.d/71-sensable-phantom.rules
ls -l /etc/SensAble/PHANToMDeviceDrivers
```

Every listed package should report `install ok installed`, neither library check should contain `not found`, and both users should belong to `cmput_312`. Configuration files should be readable by all users.

With the PHANToM powered on and connected:

```bash
found_phantom=0
for vendor_file in /sys/bus/firewire/devices/fw*/vendor; do
  [ -r "$vendor_file" ] || continue
  [ "$(cat "$vendor_file")" = "0x000b99" ] || continue
  found_phantom=1
  device_dir="${vendor_file%/vendor}"
  device_node="/dev/${device_dir##*/}"
  ls -l "$device_node"
  for device_user in mokoneshnikov phantom; do
    if sudo -u "$device_user" test -r "$device_node" && sudo -u "$device_user" test -w "$device_node"; then
      printf '%s can read/write %s\n' "$device_user" "$device_node"
    else
      printf 'FAIL: %s cannot read/write %s\n' "$device_user" "$device_node"
    fi
  done
done
if [ "$found_phantom" -eq 0 ]; then
  printf 'PHANToM not detected; reconnect/power on the device and repeat this check.\n'
fi
```

Checks the actual SensAble device node instead of assuming it is `/dev/fw1`. Both users must pass. At the latest check only the controller was present, so hardware access cannot be confirmed until the PHANToM reconnects. The operator should then finish calibration and verify live pose/buttons in the desktop example; do not launch the graphical example from this SSH checklist.
