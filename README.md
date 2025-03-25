# Docker

## Build from Dockerfile

```shell
docker build . -t barn2025:latest --no-cache
```

## Tag and push to DockerHub

```
docker tag barn2025:latest mldarobotics/barn2025:latest
docker push mldarobotics/barn2025:latest
```

## Pull from the DockerHub

```shell
docker volume create barn2025_data

docker run -dt --name barn2025 \
    --gpus all \
    -e DISPLAY="$DISPLAY" \
    -e QT_X11_NO_MITSHM=1 \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v barn2025_data:/jackal_ws/src/mlda-barn-2025 \
    barn2025:latest
```

## ROS

- Run environment
```shell
# Run KUL algorithm
python run_rviz_kul.py --world_idx 0

# Run RL algorithm
python run_rviz_rl.py --world_idx 0

# Run combined algorithm (KUL + RL) with custom weights
python run_combined.py --world_idx 0 --kul_weight 0.7 --rl_weight 0.3

# To debug
rostopic echo /cmd_vel_kul
rostopic echo /cmd_vel_rl
rostopic echo /cmd_vel

# Other utilities
python check_cuda_gpu.py 
python rl_a2c/python/a2c_model.py
rostopic hz /cmd_vel
rostopic hz /front/scan
```

- Launch files
```shell
# Launch combined navigation stack with both KUL and RL algorithms
roslaunch jackal_helper move_base_rl_kul.launch kul_weight:=0.7 rl_weight:=0.3
```

- Compile ROS Setup
```shell
cd /jackal_ws
catkin_make
source devel/setup.bash
```

- Clear map

```shell
rosservice call /move_base/clear_costmaps "{}"
```



