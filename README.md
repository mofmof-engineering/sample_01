# sample_01


## ビルド
```
docker build -t ros2-wheels:latest .
```

```
ros2 run joy joy_node
ros2 topic info /joy
ros2 topic echo /joy
```

```
cd ~/ros2_ws
colcon build --packages-select motor_driver
source install/setup.bash
```


```
docker run -it --rm \
  --net host \
  --privileged \
  --device /dev/input \
  --device /dev/js0 \
  --env="DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  ros2-wheels:latest 
```


```
docker run -it --rm \
  --privileged \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /dev:/dev \
  -v /sys:/sys \
  -v /run/udev:/run/udev \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/ros2_ws:/home/rosuser/ros2_ws \
  --device /dev/gpiomem \
  --device /dev/mem \
  --name ros2-wheels \
  ros2-wheels:latest
```


