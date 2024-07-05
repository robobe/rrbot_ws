
# Build environment
### base docker
- ubuntu 22.04
- ROS2 humble base
- Gazebo 11
- Add user `user` as none root user
- Install tmux and tmuxp
- Install CycloneDDS rmw

```bash
docker build -f .devcontainer/Dockerfile.humble_dev --target gazebo_dev -t humble:dev .
```

## build dev packages

```
```
mkdir src
cd src
ros2 pkg create <name>_gazebo --build-type ament_cmake 
ros2 pkg create <name>_description --build-type ament_cmake 
ros2 pkg create <name>_bringup --build-type ament_cmake 
```
```