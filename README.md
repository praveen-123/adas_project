# ADAS Camera & ECU Simulation Project

A professional-grade simulation of an automotive camera sensor and Electronic Control Unit (ECU), demonstrating core concepts of data acquisition, processing, and CAN bus communication used in Advanced Driver-Assistance Systems (ADAS).

## 🚀 Project Overview

This project simulates the data pipeline of a camera-based ADAS feature:
1.  **Camera Sensor Simulation:** Generates synthetic image frames and assigns a system status.
2.  **ECU Logging Simulation:** Processes the camera data and formats it into simulated CAN bus messages, including Diagnostic Trouble Codes (DTCs).

The simulation demonstrates key automotive software principles: modular design, system diagnostics, and structured logging.

## 📁 Project Structure
adas_project/
├── src/ # Future C++/ROS2 nodes for sensor fusion
├── simulations/ # Python scripts for sensor simulation
│ ├── camera_simulator.py # Simulates camera & generates frames
│ ├── ecu_logger.py # Simulates ECU CAN bus logging
│ ├── camera_log.csv # Output: Camera event log (Step 4)
│ ├── can_messages.log # Output: CAN bus message log (Step 5)
│ └── frame_*.png # Output: Generated sample images
├── docs/ # Diagrams and documentation
├── tests/ # Unit and integration tests
└── README.md # This file

## ⚙️ Environment Setup

This project was developed and tested in the following environment:
- **OS:** Ubuntu 24.04 (via WSL 2 on Windows 11)
- **Python:** 3.12+
- **Key Python Packages:** opencv-python, numpy
- **Development Tool:** Visual Studio Code with WSL extension

## 🧪 How to Run the Simulation

1.  **Navigate to the simulations directory:**
    ```bash
    cd simulations
    ```

2.  **Run the camera sensor simulator (Step 4):**
    ```bash
    python camera_simulator.py
    ```
    *Output:* Generates `frame_000.png` through `frame_009.png` and `camera_log.csv`.

3.  **Run the ECU logging simulator (Step 5):**
    ```bash
    python ecu_logger.py
    ```
    *Output:* Processes `camera_log.csv` and generates `can_messages.log` with simulated CAN messages.

## 📊 System Architecture

The following flowchart illustrates the simulated data pipeline:

[View System Architecture Diagram (Text Format)](./docs/architecture.txt)

**Data Flow Explanation:**
1.  **Camera Simulator:** Acts as the physical sensor, generating raw image data and status.
2.  **Camera Log:** Represents the sensor's internal storage or initial processing stage.
3.  **ECU Logger:** Simulates the main ECU, reading sensor data, converting it to standardized CAN messages, and handling diagnostics (DTCs).
4.  **CAN Log:** Represents the final output broadcast on the vehicle's CAN bus for other systems to consume.

## 🔧 Key Features Implemented

- **Realistic Sensor Simulation:** Image generation with status codes (OK/NO_SIGNAL)
- **Automotive-Grade Logging:** Structured CSV output mimicking automotive data formats
- **DTC Simulation:** Implementation of standard Diagnostic Trouble Codes (U0100)
- **Error Handling:** Professional try-catch blocks for robust operation
- **Virtual Environment:** Isolated Python environment for dependency management

## 📝 Reflection & Learning Outcomes

Through this project, I implemented and deepened my understanding of:

- The role of ECUs and sensor data flow in automotive systems
- The structure and purpose of CAN bus messages and DTCs
- The importance of structured logging for system diagnostics
- Professional Python development practices in an automotive context
- Using virtual environments for project dependency isolation

## 🚧 Future Enhancements

- [ ] Integrate with ROS 2 for a realistic middleware implementation
- [ ] Add a real object detection model (YOLO) to process generated images
- [ ] Convert CAN log to a standard format like .ASC or .MF4
- [ ] Develop a simple dashboard to visualize the data flow in real-time

## 📄 License

## Day 2 Progress: ROS 2 Communication
- Successfully built and tested a publisher-subscriber pair in ROS 2.
- The `camera_publisher` node publishes `sensor_msgs/Image` messages to the `/camera/image_raw` topic.
- The `display_subscriber` node subscribes to the topic, converts messages to OpenCV format, and displays them in a window.
- Verified the system using ROS 2 CLI tools (`ros2 node list`, `ros2 topic list`, `ros2 topic hz`).

## Day 3: Object Detection with YOLOv8

Implemented real-time object detection using YOLOv8 integrated with ROS 2.

### Features:
- YOLOv8 object detection integrated as a ROS 2 node
- Support for both video files and KITTI dataset images
- Standard ROS message types for detections (vision_msgs/Detection2DArray)
- Visualization of bounding boxes and confidence scores

### How to Run:
1. Start the camera publisher (choose one):
   ```bash
   # For video files
   python3 src/scripts/camera_publisher_video.py
   
   # For KITTI dataset images
   python3 src/scripts/camera_publisher_KITTI_images.py

2. Start the object detector:
   python3 src/scripts/detector_node.py

3. Start the display node:
   python3 src/scripts/display_subscriber.py


## Day 4: Updated detector_node.py file for GPU Acceleration

The object detection node now supports GPU acceleration when an NVIDIA GPU is available. Key features:

- Automatic detection of CUDA-capable devices
- Performance logging (FPS, confidence scores, class counts)
- Frame output with detection visualizations
- Support for multiple YOLO models (YOLOv8s currently enabled)

To use GPU acceleration:
1. Ensure you have NVIDIA drivers and CUDA toolkit installed
2. Install GPU-enabled PyTorch: `pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118`
3. The detector will automatically use GPU if available

## Day 5: Radar Simulation & Calibration System

**Objective:** Implemented a production-grade radar simulation with proper calibration and realistic sensor models.

### Key Features:
- **Camera Calibration System:** Configurable YAML-based calibration parameters
- **Pinhole Camera Model:** Accurate pixel-to-3D coordinate transformation
- **Realistic Radar Simulation:** Noise models based on automotive radar specifications:
  - Range accuracy: ±15cm
  - Angular accuracy: ±0.2°
  - Velocity accuracy: ±0.1 m/s
- **Multiple Verification Methods:** Terminal monitoring, rqt visualization, and command-line tools

### Files Added:
- `src/scripts/calibration_loader.py` - Camera calibration and coordinate transformation
- `src/scripts/radar_emulator_node.py` - Enhanced radar simulator with realistic noise
- `simulations/camera_calibration.yaml` - Camera calibration parameters

### Visualization Challenges:
Due to compatibility issues with RViz2, the following verification methods are recommended:
1. **rqt Image View:** `rqt` → Plugins → Visualization → Image View → `/camera/image_raw`



This project is for portfolio and educational purposes.
