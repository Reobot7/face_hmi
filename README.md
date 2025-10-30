# faceHMI

**ROS2 HMI package for robot face display with dual eyes showing attention, activity state, and health status.**

This package provides a simple, lightweight human-machine interface (HMI) for robots, displaying a pair of expressive eyes on an HDMI-connected screen. It is designed to run efficiently on single-board computers like the Raspberry Pi.

The display visualizes:
- **Attention (Gaze):** Where the robot is looking.
- **Activity State:** The robot's current operational mode (e.g., idle, working).
- **Health Status:** Key metrics like battery level and temperature, visualized as rings around the eyes.

![faceHMI Concept](docs/concept.png)  <!-- Conceptual image placeholder -->

## Features

- **Gaze Visualization:** Subscribes to `geometry_msgs/Vector3` for direct gaze control or `geometry_msgs/PointStamped` to track a 3D point in the robot's TF tree.
- **State Display:** Shows the robot's activity state (`idle`, `working`, `announce_move`, `moving`) through color and animation.
- **Health Monitoring:** Displays battery percentage and temperature as colored rings. Also shows icons for charging and network status.
- **Customizable HUD:** Overlays text information for activity, health, and custom labels.
- **Lightweight Renderer:** Uses `pygame` for simple and efficient rendering.
- **Flexible Configuration:** All key parameters (TF frames, camera FOV, etc.) are configurable via ROS2 parameters.

## System Architecture

```mermaid
graph TD
    A[/hmi/attention] --> N[face_hmi_node]
    B[/hmi/attention_target] --> N
    C[/hmi/attention_label] --> N
    D[/hmi/activity] --> N
    E[/hmi/health] --> N
    N -- Renders --> S[HDMI Display]
    subgraph "ROS2 Topics"
        A
        B
        C
        D
        E
    end
```

## Installation

### Prerequisites

- ROS2 (Humble Hawksbill recommended)
- Python 3
- `pygame` library (`pip3 install pygame`)
- A display connected via HDMI

### Build from Source

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/Reobot7/face_hmi.git
    cd face_hmi
    ```

2.  **Install dependencies:**
    ```bash
    rosdep install -i --from-path src --rosdistro humble -y
    ```

3.  **Build the packages:**
    ```bash
    colcon build
    ```

## Usage

1.  **Source the workspace:**
    ```bash
    source install/setup.bash
    ```

2.  **Launch the node:**
    ```bash
    ros2 launch face_hmi face_hmi.launch.py
    ```

    You can override parameters at launch time. For example, to match a specific camera setup:
    ```bash
    ros2 launch face_hmi face_hmi.launch.py \
        camera_frame:=my_camera_link \
        fov_x_deg:=85.0 \
        fov_y_deg:=55.0
    ```

### Running on Raspberry Pi

For optimal performance on a Raspberry Pi, ensure you are using a lightweight desktop environment or running in a console-only environment. The `fullscreen:=true` parameter (default) is recommended.

## Topics and Parameters

### Subscribed Topics

-   **/hmi/attention** (`geometry_msgs/Vector3`): Directly sets the gaze direction. `x` and `y` should be in the range `[-1, 1]`.
-   **/hmi/attention_target** (`geometry_msgs/PointStamped`): A 3D point for the eyes to look at. The node transforms this point into the `camera_frame` and projects it onto the screen.
-   **/hmi/attention_label** (`std_msgs/String`): A custom text label to display on the HUD.
-   **/hmi/activity** (`std_msgs/String`): Sets the robot's activity state. Valid values: `idle`, `working`, `announce_move`, `moving`.
-   **/hmi/health** (`face_hmi_msgs/Health`): Provides health status, including battery, temperature, and network connectivity.

### Parameters

-   `camera_frame` (string, default: `camera_optical_frame`): The TF frame to use for projecting `attention_target`.
-   `fov_x_deg` (double, default: `90.0`): Horizontal field of view of the camera in degrees.
-   `fov_y_deg` (double, default: `60.0`): Vertical field of view of the camera in degrees.
-   `fullscreen` (bool, default: `true`): If `true`, the display runs in fullscreen mode.
-   `fps` (int, default: `60`): Rendering frame rate.

## Testing

To test the display with random eye movements:

```bash
ros2 launch face_hmi test_face_hmi.launch.py
```

This launches both the display and a test publisher that randomly moves the eyes.

## Configuration

faceHMI uses YAML configuration files to manage appearance and motion settings separately.

### Quick Start with Config Files

```bash
# Use default configuration
ros2 launch face_hmi face_hmi_with_config.launch.py

# Use large eyes preset
ros2 launch face_hmi face_hmi_with_config.launch.py \
    config_file:=$(ros2 pkg prefix face_hmi)/share/face_hmi/config/large_eyes.yaml

# Use quick motion preset
ros2 launch face_hmi face_hmi_with_config.launch.py \
    config_file:=$(ros2 pkg prefix face_hmi)/share/face_hmi/config/quick_motion.yaml
```

### Available Configuration Files

- `config/default.yaml` - Default settings
- `config/eye_appearance.yaml` - Appearance-only settings
- `config/eye_motion.yaml` - Motion-only settings
- `config/large_eyes.yaml` - Preset for large expressive eyes
- `config/quick_motion.yaml` - Preset for quick responsive motion

See the [Configuration Guide](docs/CONFIGURATION.md) for detailed parameter reference and custom configuration creation.

## Documentation

- [Usage Guide](docs/USAGE.md) - Detailed usage instructions and examples
- [Configuration Guide](docs/CONFIGURATION.md) - YAML configuration file reference
- [Customization Guide](docs/CUSTOMIZATION.md) - Code-level customization (deprecated, use config files instead)
- [Roadmap](docs/ROADMAP.md) - Future features and development plans

## Future Work

See the [ROADMAP.md](docs/ROADMAP.md) for planned features and long-term vision.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
