# Overhead Field Analyser

![](https://media.discordapp.net/attachments/1304685620882444288/1345773618201624647/image.png?ex=67c5c4d9&is=67c47359&hm=17ae533e94dc2a5db7168e13660d0f82bc693f72918b5dfdb1b40ca84e0c5282&=&width=1569&height=856)

Provides a top-down view of the field from the perspective of the lab camera.

## Usage
You can simply do
```
sudo python3 unwrap_field.py -c -p [your_checkerboard_images/*.jpg]
```
to calibrate the camera from a set of checkerboards captured. 

One such dataset for the runswift lab camera exists in `desktop_ws/resources/lab_camera_checkerboards`, which is the default.

This will give you the camera intrinsic matrix K and distortion coefficients D, which should match up with what is in `unwrap_lab_camera.sh`.

You can also just use that if you want to use the parameters already found for our lab camera.

```
sudo ./unwrap_lab_camera.sh
```

## Tips
- See `--help` for information on parameters.
- If you moved the camera, you need to supply `p1`, `p2`, `p3`, and `p4`. You can click on the original image and the tool will print your click coordinates.
- You might not have permissions to open the right video device (specify with `-d`) in the devcontainer, you can try running the script with `sudo`.