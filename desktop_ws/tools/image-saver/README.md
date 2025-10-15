# Dual Camera Image Saver Node

A ROS2 node that captures and saves images from top and bottom cameras on mouse click events. The node subscribes to two camera topics, displays the live feeds in separate windows, and allows users to save images by clicking on either window.

## Features

- Real-time display of two camera feeds (top and bottom)
- Save images on-demand with mouse clicks
- Automatic creation of timestamped directories for saved images
- Support for YUV422 image format(and only YUV422, which is what we use for NAO)
- Separate directories for top and bottom camera images

## Prerequisites

- Docker environment(runswift Dev docker)

## Usage

1. Within your Docker container, run the script:
```bash
python3 image_saver_node.py
```

2. The node will create two windows labeled 'top' and 'bottom' showing the camera feeds.

3. To save an image:
   - Left-click on the 'top' window to save the current top camera image
   - Left-click on the 'bottom' window to save the current bottom camera image

4. Images are saved in the following directory structure:
```
saved-images/
└── [timestamp]/
    ├── top/
    │   ├── 0.png
    │   ├── 1.png
    │   └── ...
    └── bottom/
        ├── 0.png
        ├── 1.png
        └── ...
```

## Node Details

### Subscribed Topics

- `/camera/top/raw_image` (sensor_msgs/msg/Image)
- `/camera/bot/raw_image` (sensor_msgs/msg/Image)

### Parameters

None

### File Naming Convention

- Images are saved with incrementing numbers (0.png, 1.png, etc.)
- Separate counters for top and bottom images
- Files are organized in timestamp-based directories

## Troubleshooting

1. If windows don't appear:
   - Ensure VNC or X11Forwarding is setted properly on docker.
   - Check if camera topics are publishing data
   - Verify DISPLAY environment variable is properly set(e.g export DISPLAY=:0)

2. If images aren't saving:
   - Verify write permissions in the save directory
   - Ensure enough disk space is available
   - Check if the Docker container has proper volume mounting for saving files

3. If windows are black all the time
   - Check if you are on the same network as robot and you can actaully get data by using ros2 topic echo.