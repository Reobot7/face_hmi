# Roadmap for faceHMI

This document outlines the future development plans for the `faceHMI` project, categorized into short, mid, and long-term goals.

## Short-Term (Next 1-3 Months)

-   **Gaze Smoothing:** Implement a low-pass filter or interpolation to smooth out pupil movements, making the gaze appear more natural and less jittery.
-   **Priority Arbiter:** Introduce a simple mechanism to prioritize attention sources. For example, a direct `/hmi/attention` command could override an `/hmi/attention_target` point for a few seconds.
-   **Exact Projection with `camera_info`:** Subscribe to the `/camera/camera_info` topic to use the camera intrinsic matrix for a more accurate projection of 3D points, rather than relying on FOV approximations.
-   **Automatic Brightness Control:** Add a subscriber for `sensor_msgs/Illuminance` to automatically adjust the display brightness based on ambient light, reducing power consumption and improving visibility.

## Mid-Term (Next 3-9 Months)

-   **Target Array and Automatic Selection:** Allow publishing an array of potential attention targets. The HMI could then autonomously switch its gaze between them based on simple heuristics (e.g., proximity, time since last viewed).
-   **Privacy and Anti-Dazzle Features:**
    -   Implement a 
privacy mode" where the eyes close or look away when a person is detected very close to the camera.
    -   Add a feature to dim the display in dark environments to avoid dazzling people.
-   **Universal Design (UD) Considerations:** Introduce alternative color schemes to ensure readability for users with various forms of color vision deficiency.
-   **Telemetry and Dashboard:** Publish internal state and performance metrics (e.g., render time, current state) as ROS2 topics. Create a simple Rviz or web-based dashboard for monitoring.

## Long-Term (1+ Year)

-   **Advanced Renderer:** Evolve the renderer from `pygame` to a more powerful backend like OpenGL or a native Wayland client for improved performance, effects (e.g., realistic reflections), and lower overhead.
-   **Multi-Display Support:** Extend the architecture to support multiple coordinated displays, allowing for more complex facial expressions or auxiliary information displays.
-   **Full-Face Derivative (`FaceHMI`):** Create a more advanced version of the package that includes a mouth and other facial features, allowing for a wider range of expressions and emotions.
-   **Mono-Eye Derivative:** Create a simplified, single-eye version for smaller robots or different aesthetic goals.
-   **Anticipatory Gestures:** Develop features where the HMI can signal the robot's next intended action *before* it happens. For example, the eyes might glance towards a door just before the robot begins to move through it.
