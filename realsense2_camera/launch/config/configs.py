"""
This file contains the configurations for the realsense cameras.
"""

configs = {
    "bimanual_i2rt": {
        "cameras": [
            # {"name": "main_camera", "serial": "017322071831"},      # Only use the working D435
            {"name": "right_wrist_camera", "serial": "151222079231"}, # D435I has hardware issues - disabled
            {"name": "left_wrist_camera", "serial": "151222078659"},
        ]
    },
    "default": {
        "cameras": [
            {"name": "main_camera", "serial": "017322071831"},      # Only use the working D435  
            {"name": "right_wrist_camera", "serial": "151222078659"}, # D435I has hardware issues - disabled
            {"name": "left_wrist_camera", "serial": "151222079231"},
        ]
    }
}