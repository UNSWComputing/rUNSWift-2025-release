#ifndef PERCEPTION_VISION_CAMERA_CAMERADEFINITIONS_H_
#define PERCEPTION_VISION_CAMERA_CAMERADEFINITIONS_H_

/*
 * Arbitary focal length of the camera
 **/
#define FOCAL_LENGTH 1
/**
 * Pixel size calculated from FOV of camera and the arbitary focal length
 **/

//#define CAMERA_FOV_W 0.8098327729
#define CAMERA_FOV_W 1.06290551
#define TOP_PIXEL_SIZE (tan(CAMERA_FOV_W / 2) / 640)
#define BOT_PIXEL_SIZE (tan(CAMERA_FOV_W / 2) / 320)
#endif
