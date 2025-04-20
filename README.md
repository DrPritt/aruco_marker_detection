# marker_detection

ar_detection_ws => workspace 

camera_calibration => holds the cameras calibration files (ost.yaml)

# First go into the workspace and install all the dependencies:

1. Import git-based dependencies into workspace src/
```bash
vcs import src < workspace.rosinstall
```

2. Install apt-based dependencies onto the system
```bash
rosdep install --from-paths src --ignore-src -r -y
```

# Second build and source the workspace:
```bash
colcon build
```

Sourcing:

```bash
source ~/marker_detection/ar_detection_ws/install/setup.bash
```
