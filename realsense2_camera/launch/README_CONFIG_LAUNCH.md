# Configuration-Based Multi-Camera Launch

This directory contains launch files for dynamically launching multiple RealSense cameras based on configuration files.

## Files

- `rs_config_multi_camera_launch.py` - Main launch file that reads camera configurations and launches cameras
- `rs_dynamic_multi_camera_launch.py` - Alternative advanced launch file with more features  
- `example_config_launch.py` - Example showing how to use the configuration launch
- `config/configs.py` - Configuration file defining camera setups

## Configuration Format

The `config/configs.py` file defines different robot configurations:

```python
configs = {
    "bimanual-i2rt": {
        "cameras": [
            {"name": "left_wrist_camera", "serial": "216322072395"},
            {"name": "right_wrist_camera", "serial": "1234567890"},
            {"name": "main_camera", "serial": "1234567890"},
        ]
    },
    "default": {
        "cameras": [
            {"name": "left_wrist_camera", "serial": "216322072395"},
            {"name": "right_wrist_camera", "serial": "216322072395"},
            {"name": "main_camera", "serial": "1234567890"},
        ]
    }
}
```

## Usage

### Basic Usage

```bash
# Use default configuration (from environment ROBOT_TYPE or 'default')
ros2 launch realsense2_camera rs_config_multi_camera_launch.py

# Specify configuration type
ros2 launch realsense2_camera rs_config_multi_camera_launch.py robot_type:=bimanual-i2rt

# Use example configuration
ros2 launch realsense2_camera example_config_launch.py
```

### With Parameters

All standard RealSense parameters can be applied to all cameras:

```bash
# Enable depth and color streams
ros2 launch realsense2_camera rs_config_multi_camera_launch.py \
    robot_type:=bimanual-i2rt \
    enable_depth:=true \
    enable_color:=true

# Enable pointclouds for all cameras
ros2 launch realsense2_camera rs_config_multi_camera_launch.py \
    robot_type:=bimanual-i2rt \
    pointcloud.enable:=true \
    align_depth.enable:=true

# Set resolution profiles
ros2 launch realsense2_camera rs_config_multi_camera_launch.py \
    robot_type:=bimanual-i2rt \
    rgb_camera.color_profile:=640,480,30 \
    depth_module.depth_profile:=640,480,30
```

### Environment Variables

Set the robot type using environment variable:

```bash
export ROBOT_TYPE=bimanual-i2rt
ros2 launch realsense2_camera rs_config_multi_camera_launch.py
```

## Features

### Automatic Device Detection

The launch file automatically:
- Detects connected RealSense devices using `pyrealsense2`
- Matches serial numbers from config to connected devices
- Only launches cameras that are physically connected
- Provides clear logging about which cameras are found/missing

### Error Handling

- Gracefully handles missing `pyrealsense2` library
- Falls back to launching all configured cameras if device detection fails
- Provides clear error messages for missing cameras
- Uses fallback logging if `python_utils` is not available

### Transform Publishers

Automatically creates static transform publishers between cameras for multi-camera setups.

## Camera Namespaces and Topics

Each camera is launched in its own namespace based on the camera name:

```
/left_wrist_camera/
├── color/
│   ├── image_raw
│   └── camera_info
├── depth/
│   ├── image_rect_raw
│   └── camera_info
└── ...

/right_wrist_camera/
├── color/
│   ├── image_raw
│   └── camera_info
└── ...
```

## Troubleshooting

### No cameras launched

1. Check that devices are connected: `rs-enumerate-devices`
2. Verify serial numbers in `config/configs.py` match your devices
3. Check that `pyrealsense2` is installed: `pip install pyrealsense2`
4. Set robot type: `export ROBOT_TYPE=your_config_name`

### Import errors

If you get import errors for `python_utils` or `pyrealsense2`, the launch file will still work but with reduced functionality:
- Without `pyrealsense2`: All configured cameras will be launched (no connection checking)
- Without `python_utils`: Basic logging will be used instead of colored logging

### Serial number format

Make sure serial numbers in the config are strings, not integers:
```python
# Correct
{"name": "camera1", "serial": "216322072395"}

# Incorrect  
{"name": "camera1", "serial": 216322072395}
```

## Extending the Configuration

To add a new robot configuration:

1. Add your configuration to `config/configs.py`:
```python
configs = {
    "my_robot": {
        "cameras": [
            {"name": "front_camera", "serial": "your_serial_here"},
            {"name": "back_camera", "serial": "another_serial"},
        ]
    },
    # ... existing configs
}
```

2. Use it:
```bash
ros2 launch realsense2_camera rs_config_multi_camera_launch.py robot_type:=my_robot
```

## Advanced Usage

For more advanced features, use `rs_dynamic_multi_camera_launch.py` which includes:
- More granular parameter control per camera
- Advanced TF handling
- Additional configuration options
