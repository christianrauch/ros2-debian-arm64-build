# Docker Setup for Debian arm64 builds of ROS2

# Docker Installation
Follow the [Install Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/) instructions to install Docker and setup the `docker` group as documented in the [Post-installation steps for Linux](https://docs.docker.com/engine/install/linux-postinstall/) instructions.

# Start Container
This starts the Debian arm64 container, binds the current local host path to `/ros2` inside the container and keeps it alive for later use:
```sh
# install qemu
sudo apt update
sudo apt install -y qemu-user-static

# start container
export IMAGE=arm64v8/debian:bullseye
docker pull $IMAGE
docker run --name ros2_build --platform linux/arm64 --mount type=bind,source=$(pwd),target=/ros2 -di $IMAGE bash
```

# Installing and Building a Basic ROS2 Workspace
The `install_ros.sh` script will download the sources of a basic ROS2 configuration and install all build dependencies:
```sh
docker exec --interactive ros2_build /ros2/install_ros.sh
```

Finally, to build the basic workspace inside the docker container, run:
```sh
docker exec --interactive ros2_build /ros2/build_packages.sh
```
This will take a long time, depending on the amount of CPU cores, since the building will take place via qemu.

After the build has finished, you can compress the workspace via `tar -caf ros2-debian-arm64.tar.xz install/*`, move it to your target device and decompress it via `tar -xf ros2-debian-arm64.tar.xz` again. When installing the binary build on a new target system, you have to install the package dependencies via rosdep as described in the [“fat” archive installation instructions](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Binary.html#installing-the-missing-dependencies).



# Building a Custom Workspace

If you have a colcon workspace on the host, for example at `$HOME/dev/ws`, you can bind mount this workspace inside the container and build it with `/ros2/install` as the ROS2 installation.

```sh
# stop and commit container
docker stop ros2_build
docker commit ros2_build ros2_build/ros2
docker container rm ros2_build
# restart container with active session and an additional mount
docker run --name ros2_build --platform linux/arm64 \
    --mount type=bind,source=$(pwd),target=/ros2 \
    --mount type=bind,source=$HOME/dev/ws,target=/ws \
    -ti ros2_build/ros2 bash
```

In the interactive shell inside the container, you can now switch to your workspace and build the workspace as usual:
```sh
source /ros2/install/setup.bash
cd /ws
rosdep install --from-paths src --ignore-src -y
colcon build
```
