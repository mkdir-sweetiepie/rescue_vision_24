# Rescue Vision 24

<a href="https://github.com/mkdir-sweetiepie"><img src="https://img.shields.io/badge/Ji Hyeon Hong-white?style=flat&logo=github&logoColor=red"/></a>

Victim detection system for rescue robot Victim Box mission using Intel RealSense and Thermal camera. Tested on ROS Noetic.

## Table of Contents
- [Overview](#overview)
- [System Requirements](#system-requirements)
- [Package Structure](#package-structure)
- [Installation](#installation)
- [Usage](#usage)
- [Topic Structure](#topic-structure)
  
## Overview

This project is a victim detection system designed for rescue robot competitions. It utilizes both Intel RealSense RGB camera and thermal camera to detect victims in disaster scenarios.

### Key Features
- **Victim Detection**: Real-time victim detection using RGB images
- **Thermal Imaging**: Thermal camera integration for heat signature detection
- **Request Manager**: Automated node management based on mission start/end signals
- **RealSense Integration**: Intel RealSense camera support

## System Requirements

| Component | Version |
|-----------|---------|
| Ubuntu | 20.04 LTS |
| ROS | Noetic |
| OpenCV | 4.x |
| Qt | 5.x |
| Intel RealSense SDK | 2.x |

### Hardware
- Intel RealSense Depth Camera (D435/D435i/D455)
- Thermal Camera

## Package Structure
```
rescue_vision_24/
├── rescue_vision_24/           # Main vision processing node
│   ├── include/
│   ├── src/
│   └── launch/
├── rescue_vision_ui/           # Test GUI node (for testing only)
│   ├── include/
│   ├── src/
│   └── ui/
└── victim_request_manager/     # Mission request manager node
    ├── include/
    └── src/
```

### Package Description

| Package | Description |
|---------|-------------|
| `rescue_vision_24` | Main victim detection and image processing |
| `rescue_vision_ui` | Qt-based test GUI (not for competition use) |
| `victim_request_manager` | Manages node activation based on start/end signals |

## Installation

### 1. Set up ROS Workspace
```bash
mkdir -p ~/${workspace_name}_ws/src
cd ~/${workspace_name}_ws/src
```

### 2. Clone Repository
```bash
git clone https://github.com/RO-BIT-Intelligence-Robot-Team/rescue_vision_24.git
```

### 3. Build
```bash
cd ~/${workspace_name}_ws
catkin_make
source devel/setup.bash
```

## Usage

### For Competition
> ⚠️ **Note**: `rescue_vision_ui` is for testing purposes only. Delete it before competition use.

The `rescue_vision_24` node automatically starts and stops when receiving `start` and `end` signals from `/victim_start` topic.
```bash
rosrun victim_request_manager victim_request_manager
```

### For Testing
```bash
# Terminal 1: Launch vision node
roslaunch rescue_vision_24 rescue_vision_24.launch

# Terminal 2: Run test GUI
rosrun rescue_vision_ui rescue_vision_ui
```

## Topic Structure

### Communication with Main UI

| Type | Topic | Description |
|------|-------|-------------|
| Publish | `victim_image` | Result image with detection |
| Publish | `img_result_thermal` | Processed thermal image |
| Subscribe | `victim_start` | Mission start/end signal |

### Communication with Cameras

| Type | Topic | Description |
|------|-------|-------------|
| Subscribe | `/camera/color/image_raw` | RealSense RGB image |
| Subscribe | `thermal_camera/image_colored` | Thermal camera image |
