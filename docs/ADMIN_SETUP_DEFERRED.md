# Deferred administrator setup

Moved from sections 5–7 of [the main checklist](ADMIN_SETUP.md). Complete these when video export, physical Gen3 control, or additional training machines are needed.

1. Video output and optional headless rendering

```bash
sudo apt install ffmpeg libosmesa6
```

Installs video export tools and an optional software-rendering fallback for all users. MuJoCo already renders through NVIDIA EGL on this machine; OSMesa is not required for state-only training.

2. Gen3 Ethernet connection — robot-control computer only

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

3. Additional training machines

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

References: [Blackwell PyTorch support](https://pytorch.org/blog/pytorch-2-7/), [Gen3 network setup](https://github.com/Kinovarobotics/kortex/blob/master/api_python/examples/readme.md).

Remaining blanks to confirm:
- `ROBOT_IFACE`: the dedicated Gen3 adapter; only campus Ethernet `enp128s31f6` is currently detected.
- Gen3 addresses: `192.168.1.10` and `192.168.1.11/24` are example defaults, not verified lab assignments. Robot credentials also remain unconfirmed; do not put passwords in this document.
- Training-machine hostnames, available user accounts, and Python environment paths: not supplied or checked remotely.

4. Checks

- Test GPU computation on every training machine and headless rendering on machines used for visual evaluation.
- Test SSH and repository access using each intended account; keep each user's checkout and Python environment writable by that user.
- Verify Gen3 reachability and a read-only Kortex connection if pursuing the physical extension.

Use the GPU computation test in [the main checklist](ADMIN_SETUP.md) on each training machine. These machines and the Gen3 connection remain unverified.
