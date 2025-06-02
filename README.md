# RBR CTD ROS2 Driver

This repository provides a ROS 2 driver for [RBR CTD](https://rbr-global.com/) - an instrument for oceanographic measurements, including conductivity, temperature and depth. The repository contains two ROS2 packages:

- **`rbr_ctd_driver`**: A Python package that handles serial communication with the CTD sensor and data publishing. It contains one single ROS2 nodes:
  - `rbr_ctd_driver`: subscribes to the raw data, e.g. published by `serial_parser` package. Process the messages and publish them into custom ROS2 `RBRCTD` messages.
- **`rbr_ctd_interface`**: A C++ package defining custom ROS 2 messages and service types for interfacing with the CTD data. It contains one single ROS2 custom message:
  - `RBRCTD.msg`: Defines the structure of a CTD message.

## Dependencies

The driver has been tested with the following set up:
- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [Ubuntu 22.04 LTS](https://releases.ubuntu.com/jammy/)
- Python 3.10.12
- [`pyserial`](https://github.com/pyserial/pyserial) (for serial communication in `serial_reader_ctd`)

To run the unit tests, `Werkzeug` is needed and can be installed via:
```bash
pip install "Werkzeug>=2.2,<3.0"
```

---

## Installation

Clone the repository into your ROS 2 workspace and build the packages:

```bash
cd ~/ros2_ws/src
git clone https://github.com/smarc-project/rbr_ctd_driver.git
colcon build && source install/setup.bash
```
## Usage
### 1. Launch file
The easiest option to run the driver is using the launch file in `rbr_ctd_driver` package:
```bash
ros2 launch rbr_ctd_driver rbr_ctd_driver.launch
```
This will:
- Start rbr_ctd_driver that parses the strings from `input_topic` and publish structured RBRCTD messages to `output_topic`.

### 2. Run the nodes individually
#### 2.2 Start the CTD driver node:
```bash
ros2 run rbr_ctd_driver rbr_ctd_driver --ros-args -p input_topic:=/rbr_ctd/raw_data -p output_topic:=/rbr_ctd/data
```

## Testing
The `rbr_ctd_driver` package contains a unit test that verifies correct parsing of raw CTD strings into custom RBRCTD messages. From the root of your ROS2 workspace, there are two ways of running the tests:
### 1. Run the test with colcon
```bash
colcon test --packages-select rbr_ctd_driver
colcon test-result --verbose
```
### 2. Run the test directly with `pytest`
```bash
pytest ros2_ws/src/rbr_ctd_driver/rbr_ctd_driver/test/test_ctd_parser.py
```
