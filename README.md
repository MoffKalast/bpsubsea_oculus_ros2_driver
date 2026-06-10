# Blueprint Subsea Oculus ROS 2 Driver

ROS 2 driver for Blueprint Subsea Oculus forward looking sonars, initially aimed at the Oculus M750d.

This package publishes:

- `/oculus/status` (`bpsubsea_oculus_ros2_driver/msg/Status`)
- `/oculus/ping` (`bpsubsea_oculus_ros2_driver/msg/Ping`)
- `/oculus/image` (`sensor_msgs/msg/Image`)
- `/oculus/fan_image` (`sensor_msgs/msg/Image`) for human visualization

The driver listens for Oculus status broadcasts on UDP port `52102`. With the default
`sonar_ip:=auto`, it connects to the source address of the latest valid status packet.
For a fixed sonar address, set `sonar_ip` explicitly.

## Build

```bash
colcon build --packages-select bpsubsea_oculus_ros2_driver
```

## Run

```bash
source install/setup.bash
ros2 launch bpsubsea_oculus_ros2_driver oculus_driver.launch.py
```

With a fixed address:

```bash
ros2 launch bpsubsea_oculus_ros2_driver oculus_driver.launch.py sonar_ip:=192.168.2.10
```

Run only the acquisition driver:

```bash
ros2 launch bpsubsea_oculus_ros2_driver oculus_driver.launch.py start_fan_image:=false
```

## Common Parameters

### Acquisition Driver

- `sonar_ip`: sonar IPv4 address, or `auto` to use status broadcast source address.
- `frame_id`: frame used in published message headers. Default: `oculus`.
- `mode`: simple fire master mode. For M750d, `1` is the 750 kHz/navigation mode and `2` is the 1.2 MHz/identification mode.
- `range`: requested range in metres.
- `gain`: requested gain percentage.
- `speed_of_sound`: fixed speed of sound in m/s. Use `0.0` to let the sonar calculate it.
- `salinity`: salinity in ppt, used by the sonar when `speed_of_sound` is `0.0`.
- `gain_assist`: enables gain assist in the simple fire command.
- `enable_512_beams`: requests 512 beams, matching the SDK viewer behaviour.
- `gamma_correction`: simple fire gamma correction byte.
- `network_speed_limit`: network speed limit byte. `255` disables the limiter, matching the SDK viewer behaviour for gigabit links.

### Fan Image Renderer

The launch file starts `oculus_fan_image_node` by default. It subscribes to `/oculus/ping` and publishes `/oculus/fan_image`.

- `start_fan_image`: start the visualization node from the launch file. Default: `true`.
- `fan_output_width`: rendered image width in pixels. Default: `800`.
- `fan_output_height`: rendered image height in pixels. Default: `800`.
- `fan_meters_per_pixel`: visual scale used for the fan remap. `0.0` auto-fits each ping to the image. Default: `0.0`.
- `fan_max_range`: maximum displayed range in metres. `0.0` uses each ping's full range.
- `fan_center_x`: sonar origin x pixel. `-1.0` means horizontal centre.
- `fan_center_y`: sonar origin y pixel. `-1.0` chooses a small top margin when `head_down` is true.
- `fan_palette`: `mono8`, `amber`, `blue_green`, or `fire`. Non-mono palettes publish `rgb8`.
- `fan_flip_x`: mirror the rendered fan horizontally.
- `fan_flip_y`: mirror the rendered fan vertically.
- `fan_head_down`: render the fan opening downward. Default: `true`.
- `fan_background_value`: background intensity from `0` to `255`.

## Notes

The wire protocol definitions are derived from Blueprint Subsea's GPL-licensed Oculus Viewer SDK, so this package is licensed as `GPL-3.0-or-later`.
