#!/bin/bash
# 🚀 Setup environment for ADAS project

# Activate virtual environment
source ~/adas_project/adas_venv/bin/activate

# Export ROS2 Python path
export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH

# Go to project root
cd ~/adas_project

echo "✅ Environment ready. Now run your scripts!"
echo "Example: python3 src/scripts/camera_publisher_KITTY_images.py"
