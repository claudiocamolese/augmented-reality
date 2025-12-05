# Augmented Reality

This project demonstrates an Augmented Reality (AR) application in C++ using OpenCV. It detects a chessboard in a video or webcam stream, calibrates the camera, estimates the pose of the chessboard and projects a 3D cube onto the scene in real-time.

![output_video (online-video-cutter com)](https://github.com/user-attachments/assets/07397888-336f-4567-a12e-554ef22c4ed7)
---

## Features

- **Chessboard detection** using `findChessboardCorners`
- **Corner refinement** with `cornerSubPix` for sub-pixel accuracy
- **Camera calibration** using `calibrateCamera`
- **Pose estimation** using `solvePnPRansac`
- **3D cube projection** using `projectPoints`
- **Real-time video visualization**
- **Video output** saved in `output/output_video.avi`


## Installation

1. Clone the repository:
```bash
git clone <repository_url>
cd augmented-reality
g++ main.cpp figures/cube.cpp -o main \
    -I/usr/include/opencv4 \
    -L/usr/lib/x86_64-linux-gnu \
    -lopencv_core -lopencv_imgproc -lopencv_highgui \
    -lopencv_imgcodecs -lopencv_calib3d -lopencv_videoio
```

Run the program with a video file or webcam:
```bash
./main <video_file_or_webcam> #path_video or 0 for webcam
```
Press `c` to calibrate the camera using the detected chessboard.
Press `ESC` to exit the program.
The augmented video is saved in `output/output_video.avi`.

Then, to convert the `.avi` into a `.mp4` use:
```bash
sudo apt install ffmpeg
ffmpeg -i ./output/output_video.avi -c:v libx264 ./output/<your-name>.mp4
```
## Notes

- Ensure the chessboard is clearly visible and well-lit for better calibration.
- The program currently supports only the cube, but other 3D shapes can be added in the `figures/` folder.
- The `output/` folder will be automatically created if it does not exist.
- Use high-quality video for smoother AR performance.
