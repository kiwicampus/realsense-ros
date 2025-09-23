"""
This file contains the configurations for the realsense cameras.
"""

configs = {
    "bimanual-i2rt": {
        "cameras": [
            {"name": "left_wrist_camera", "serial": "017322071831"},      # Only use the working D435
            {"name": "right_wrist_camera", "serial": "216322072395"}, # D435I has hardware issues - disabled
            {"name": "main_camera", "serial": "1234567890"},
        ]
    },
    "default": {
        "cameras": [
            {"name": "left_wrist_camera", "serial": "017322071831"},      # Only use the working D435  
            {"name": "right_wrist_camera", "serial": "216322072395"}, # D435I has hardware issues - disabled
            {"name": "main_camera", "serial": "1234567890"},
        ]
    }
}