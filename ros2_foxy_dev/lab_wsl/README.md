# Windows setup: Ubuntu 20.04 and ROS 2 Foxy in Docker

This profile is based on `../lab`. It provides an Ubuntu 20.04 x86_64 container
with ROS 2 Foxy, Gazebo Classic, RViz, Nav2 and workspace dependencies.
Docker Desktop runs the container on Windows through WSL2.

Follow the steps below yourself. Run Windows commands in PowerShell and Linux
commands in the Ubuntu WSL terminal, as indicated.

## 1. Install Ubuntu 20.04 with WSL2

Use an x86_64 Windows computer that supports Docker Desktop and WSLg.
Hardware virtualization must be enabled in the BIOS/UEFI.

Open **PowerShell as Administrator** and run:

```powershell
wsl --install -d Ubuntu-20.04
```

Restart Windows if prompted. Open **Ubuntu 20.04** from the Start menu and
create your Linux username and password when prompted.

Back in PowerShell, update WSL and check the distribution:

```powershell
wsl --update
wsl --list --verbose
```

The `Ubuntu-20.04` row must show `VERSION 2`. If it shows `1`, run:

```powershell
wsl --set-version Ubuntu-20.04 2
```

If Ubuntu 20.04 is already installed in WSL2, skip the installation command.
See the [Microsoft WSL command reference](https://learn.microsoft.com/en-us/windows/wsl/basic-commands).

## 2. Install and configure Docker Desktop

Download and install [Docker Desktop for Windows](https://docs.docker.com/desktop/setup/install/windows-install/).
Choose the **WSL2 backend** when the installer offers that choice.

Start Docker Desktop and complete its first-run setup. Then open **Settings**:

1. Under **General**, enable **Use WSL 2 based engine**, if the option is shown.
2. Under **Resources > WSL Integration**, enable **Ubuntu-20.04**.
3. Click **Apply / Apply & restart**.
4. Make sure Docker Desktop uses **Linux containers**.

Keep Docker Desktop running while using this workspace.
The [Docker WSL2 guide](https://docs.docker.com/desktop/features/wsl/)
describes these settings.

Do not run `ros2_foxy_dev/lab/setup-host.sh` in WSL. That script installs a
separate Linux Docker Engine; this Windows profile uses Docker Desktop.

## 3. Put the project inside Ubuntu

Copy the **complete project**, including both `workspace` and `ros2_foxy_dev`,
into the Ubuntu filesystem. Use this layout:

```text
/home/YOUR_LINUX_USERNAME/Limo/
  workspace/
    src/
  ros2_foxy_dev/
    lab/
    lab_wsl/
```

You can use Windows File Explorer. Enter this address, replacing
`YOUR_LINUX_USERNAME` with the username you created in Ubuntu:

```text
\\wsl.localhost\Ubuntu-20.04\home\YOUR_LINUX_USERNAME
```

Copy the project into a directory named `Limo` in your Linux home directory.
Using the Linux filesystem helps workspace build performance.

## 4. Check the setup

Open **Ubuntu 20.04**, then run:

```bash
cd ~/Limo/ros2_foxy_dev/lab_wsl
bash setup-host.sh
```

This script checks WSL, Docker Desktop integration, Compose and WSLg.
It does not install a second Docker Engine.

Create the configuration file once:

```bash
cp .env.example .env
```

If `.env` already exists, keep it instead of copying over it.
The default configuration works without an NVIDIA GPU.

## 5. Build and start the container

In the same **Ubuntu terminal**, run these commands one at a time:

```bash
bash dev.sh build
bash dev.sh up
bash dev.sh check
bash dev.sh build-workspace
bash dev.sh shell
```

The first command downloads and installs the container dependencies, so it
requires Internet access and can take several minutes.
`build-workspace` compiles the packages currently available in `workspace/src`.
The last command opens a shell inside the container with ROS loaded.

You do not need to install ROS directly on Windows or in the Ubuntu WSL host.
Build, install and log files for this profile are stored in a dedicated Docker
volume mounted at `/workspace/.lab_wsl`.

## 6. Start the simulation

Inside the **container shell**, after a successful workspace build, run:

```bash
ros2 launch custom_start limo_circuit.launch.py
```

To check RViz separately, run:

```bash
rviz2
```

GUI windows use [WSLg](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps).
The default configuration uses software OpenGL for compatibility with Ubuntu
20.04. Use a local Ubuntu WSL terminal for these steps.

## Everyday use

Run these commands from `ros2_foxy_dev/lab_wsl` in the Ubuntu terminal:

| Command | Purpose |
| --- | --- |
| `bash dev.sh up` | Start the container |
| `bash dev.sh shell` | Open a ROS shell in the container |
| `bash dev.sh build-workspace` | Rebuild workspace packages |
| `bash dev.sh deps` | Install dependencies after changing package manifests |
| `bash dev.sh check` | Check Python imports, ROS packages and native libraries |
| `bash dev.sh config` | Display the Compose configuration |
| `bash dev.sh down` | Stop and remove the container, keeping the build volume |

After rebuilding, reopen the container shell to load the updated workspace.

## Optional: NVIDIA GPU

First install a Windows NVIDIA driver with WSL support and update WSL.
Edit `.env` and set:

```dotenv
ENABLE_GPU=1
```

Then run in the Ubuntu terminal:

```bash
bash dev.sh up
bash dev.sh gpu
```

This enables NVIDIA compute support and checks it with `nvidia-smi`.
GUI rendering remains in software mode by default.
See [Docker Desktop GPU support](https://docs.docker.com/desktop/features/gpu/).

## Optional: external ROS devices

For communication with an external robot, set `ENABLE_HOST_NETWORK=1` in `.env`
and enable **Resources > Network > Enable host networking** in Docker Desktop
4.34 or later. Restart the container with `bash dev.sh up` and use the same
`ROS_DOMAIN_ID` on all participating ROS systems.

DDS discovery across Windows, WSL and the LAN still needs to be checked with
your network and firewall. Docker Desktop host networking has
[platform limitations](https://docs.docker.com/engine/network/drivers/host/).

USB cameras and serial devices need additional device configuration.
They are not automatically passed through by this profile. Refer to
[USB/IP with Docker Desktop](https://docs.docker.com/desktop/features/usbip/)
and [connecting USB devices to WSL](https://learn.microsoft.com/en-us/windows/wsl/connect-usb).

## Included dependencies and current source limitations

The image installs dependencies from all available `package.xml` files,
including drivers currently excluded from compilation with `COLCON_IGNORE`.
It also includes native camera libraries, a locally built YDLidar SDK,
Cartographer, navigation and simulation tools, and the Python perception
libraries inherited from `lab`. Python package versions are listed in
`requirements.txt`.

The current checkout has two source limitations:

- The five directories in `workspace/src/ros2_ws` currently contain Python
  caches but no `.py` sources, `setup.py` or package manifests. Restore their
  original sources before expecting those packages to build.
- The Astra driver references `openni2_redist/x64`, which is missing from the
  checkout. Restore the manufacturer's OpenNI2 files before enabling that
  driver. Existing `COLCON_IGNORE` files remain respected during compilation.

ROS 2 Foxy is out of support; the profile keeps it to match the existing lab
environment. The full image build and Windows GUI/device behavior have not
been verified here. The local image build was interrupted at your request.

`setup-windows.ps1` is also supplied as an optional Windows installation helper;
the manual steps above are sufficient and do not require running that script.
