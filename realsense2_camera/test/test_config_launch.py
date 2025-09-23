#!/usr/bin/env python3

"""
Test script for the configuration-based multi-camera launch.

This script tests the launch file functionality without actually launching ROS nodes.
It validates the configuration loading and camera detection logic.
"""

import os
import sys
import pathlib

# Add current directory to path
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))

def test_config_loading():
    """Test configuration loading."""
    print("Testing configuration loading...")
    
    try:
        from config.configs import configs as realsense_configs
        print(f"✅ Successfully loaded configs: {list(realsense_configs.keys())}")
        
        # Test each config
        for config_name, config_data in realsense_configs.items():
            cameras = config_data.get('cameras', [])
            print(f"  - {config_name}: {len(cameras)} cameras")
            for camera in cameras:
                print(f"    * {camera['name']}: {camera['serial']}")
        
        return True
    except Exception as e:
        print(f"❌ Failed to load configs: {e}")
        return False

def test_serial_detection():
    """Test serial number detection."""
    print("\nTesting serial number detection...")
    
    try:
        import pyrealsense2 as rs
        print("✅ pyrealsense2 is available")
        
        try:
            ctx = rs.context()
            devices = ctx.query_devices()
            serials = [str(d.get_info(rs.camera_info.serial_number)) for d in devices]
            print(f"✅ Found {len(serials)} connected devices: {serials}")
            return serials
        except Exception as e:
            print(f"⚠️  Could not query devices: {e}")
            return []
            
    except ImportError:
        print("⚠️  pyrealsense2 not available - serial detection will be skipped")
        return []

def test_launch_logic():
    """Test the launch logic without actually launching."""
    print("\nTesting launch logic...")
    
    try:
        from config.configs import configs as realsense_configs
        
        # Test with different robot types
        test_configs = ['default', 'bimanual_i2rt']
        connected_serials = test_serial_detection()
        
        for robot_type in test_configs:
            if robot_type not in realsense_configs:
                print(f"❌ Config '{robot_type}' not found")
                continue
                
            config = realsense_configs[robot_type]
            cameras_config = config.get('cameras', [])
            
            print(f"\n📋 Testing config: {robot_type}")
            print(f"   Configured cameras: {len(cameras_config)}")
            
            launchable_cameras = []
            for camera_config in cameras_config:
                camera_name = camera_config['name']
                camera_serial = str(camera_config['serial'])
                
                if not connected_serials or camera_serial in connected_serials:
                    launchable_cameras.append(camera_name)
                    print(f"   ✅ {camera_name} ({camera_serial}) - would launch")
                else:
                    print(f"   ❌ {camera_name} ({camera_serial}) - not connected")
            
            print(f"   Result: {len(launchable_cameras)} cameras would be launched")
            
        return True
        
    except Exception as e:
        print(f"❌ Launch logic test failed: {e}")
        return False

def test_import_fallbacks():
    """Test import fallback mechanisms."""
    print("\nTesting import fallbacks...")
    
    # Test python_utils fallback
    try:
        from python_utils.vision_utils import printlog
        print("✅ python_utils.vision_utils available")
    except ImportError:
        print("⚠️  python_utils not available - using fallback logging")
        
        # Test fallback function
        def printlog(msg, msg_type="INFO"):
            colors = {
                "OKGREEN": "\033[92m",
                "ERROR": "\033[91m",
                "INFO": "\033[94m", 
                "ENDC": "\033[0m"
            }
            color = colors.get(msg_type, colors["INFO"])
            print(f"{color}[{msg_type}] {msg}{colors['ENDC']}")
        
        printlog("Fallback logging works", "OKGREEN")
        print("✅ Fallback logging functional")

def main():
    """Run all tests."""
    print("🧪 Testing configuration-based multi-camera launch")
    print("=" * 50)
    
    success = True
    
    # Run tests
    success &= test_config_loading()
    test_serial_detection()  # This can fail without affecting overall success
    success &= test_launch_logic()
    test_import_fallbacks()  # This tests fallback mechanisms
    
    print("\n" + "=" * 50)
    if success:
        print("🎉 All critical tests passed!")
        print("\nTo use the launch file:")
        print("ros2 launch realsense2_camera rs_config_multi_camera_launch.py")
        print("ros2 launch realsense2_camera rs_config_multi_camera_launch.py robot_type:=bimanual-i2rt")
    else:
        print("❌ Some tests failed. Check the configuration and dependencies.")
        
    return 0 if success else 1

if __name__ == "__main__":
    exit(main())
