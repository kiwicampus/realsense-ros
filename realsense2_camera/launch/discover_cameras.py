#!/usr/bin/env python3

"""
Camera Discovery Script

This script helps you discover connected RealSense cameras and their serial numbers.
Use this to update your configs.py file with the correct serial numbers.

Usage:
    python3 discover_cameras.py
"""

import sys

def discover_cameras():
    """Discover connected RealSense cameras."""
    try:
        import pyrealsense2 as rs
    except ImportError:
        print("❌ Error: pyrealsense2 is not installed.")
        print("Install it with: pip install pyrealsense2")
        return False
    
    try:
        # Create context and query devices
        ctx = rs.context()
        devices = ctx.query_devices()
        
        if len(devices) == 0:
            print("❌ No RealSense devices found.")
            print("Make sure your cameras are connected and recognized by the system.")
            return False
        
        print(f"🎥 Found {len(devices)} RealSense device(s):")
        print("=" * 60)
        
        for i, device in enumerate(devices):
            try:
                # Get device information
                serial = device.get_info(rs.camera_info.serial_number)
                name = device.get_info(rs.camera_info.name)
                firmware = device.get_info(rs.camera_info.firmware_version)
                product_line = device.get_info(rs.camera_info.product_line)
                
                print(f"\nDevice {i+1}:")
                print(f"  Name: {name}")
                print(f"  Serial: {serial}")
                print(f"  Product Line: {product_line}")
                print(f"  Firmware: {firmware}")
                
                # Get available sensors
                sensors = device.query_sensors()
                print(f"  Sensors: {len(sensors)} available")
                
                for j, sensor in enumerate(sensors):
                    sensor_name = sensor.get_info(rs.camera_info.name)
                    print(f"    - {sensor_name}")
                
            except Exception as e:
                print(f"  ⚠️  Error getting info for device {i+1}: {e}")
        
        print("\n" + "=" * 60)
        print("📝 To use these cameras in your launch file:")
        print("\n1. Update config/configs.py with the serial numbers above")
        print("2. Example configuration:")
        print('   "my_robot": {')
        print('       "cameras": [')
        
        for i, device in enumerate(devices):
            try:
                serial = device.get_info(rs.camera_info.serial_number)
                print(f'           {{"name": "camera_{i+1}", "serial": "{serial}"}},')
            except:
                pass
        
        print('       ]')
        print('   }')
        
        print("\n3. Launch cameras:")
        print("   ros2 launch realsense2_camera rs_config_multi_camera_launch.py robot_type:=my_robot")
        
        return True
        
    except Exception as e:
        print(f"❌ Error discovering cameras: {e}")
        return False

def test_rs_enumerate():
    """Test if rs-enumerate-devices command works."""
    import subprocess
    
    try:
        result = subprocess.run(['rs-enumerate-devices'], 
                              capture_output=True, text=True, timeout=10)
        
        if result.returncode == 0:
            print("\n🔧 rs-enumerate-devices output:")
            print("-" * 40)
            print(result.stdout)
            return True
        else:
            print(f"\n⚠️  rs-enumerate-devices failed: {result.stderr}")
            return False
            
    except FileNotFoundError:
        print("\n⚠️  rs-enumerate-devices command not found.")
        print("Make sure RealSense SDK is properly installed.")
        return False
    except subprocess.TimeoutExpired:
        print("\n⚠️  rs-enumerate-devices timed out.")
        return False
    except Exception as e:
        print(f"\n⚠️  Error running rs-enumerate-devices: {e}")
        return False

def main():
    """Main function."""
    print("🔍 RealSense Camera Discovery Tool")
    print("=" * 60)
    
    success = discover_cameras()
    
    if success:
        print("\n" + "=" * 60)
        test_rs_enumerate()
    
    return 0 if success else 1

if __name__ == "__main__":
    exit(main())
