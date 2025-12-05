#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "figures/cube.h"

int main(int argc, char* argv[]) {

    // --- Read video or webcam ---
    std::string videoFile = (argc > 1) ? argv[1] : "";
    cv::VideoCapture cap(videoFile.empty() ? 0 : videoFile);
    if(!cap.isOpened()){
        std::cerr << "Error: can't open video or webcam!\n";
        return -1;
    }

    // --- Initialize ---
    cv::Mat frame, gray;
    cv::Size gridSize(6,5);
    std::vector<std::vector<cv::Point3f>> points3d;
    std::vector<std::vector<cv::Point2f>> points2d;
    cv::Mat intrinsics, distortion;
    cv::namedWindow("AR_demo");
    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);
    int image_counter = 0;

    std::cout << "Show the chessboard and press 'c' to calibrate. ESC to exit.\n";

    // --- Create 3D points of the chessboard ---
    std::vector<cv::Point3f> objectPoints;
    for(int j=0;j<gridSize.height;j++){
        for(int i=0;i<gridSize.width;i++){
            objectPoints.emplace_back(i,j,0);
        }
    }

    // --- Define cube 3D points ---
    std::vector<cv::Point3f> cube = {
        {0,0,0}, {0,3,0}, {3,3,0}, {3,0,0},
        {0,0,-3}, {0,3,-3}, {3,3,-3}, {3,0,-3}
    };

    // --- Main loop ---
    while(true){
        bool ret = cap.read(frame);
        if(!ret || frame.empty()) break;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        int key = cv::waitKey(1);
        if(key==27) break; // ESC

        std::vector<cv::Point2f> corners;

        /** 
         * bool cv::findChessboardCorners(
         *      InputArray image,        // grayscale input image
         *      Size patternSize,        // number of internal corners per chessboard row/column
         *      OutputArray corners      // detected 2D points
         * )
         * 
         * Detects the internal corners of a chessboard pattern in the image.
         * Returns true if all corners were found.
         * Detected points are ordered left-to-right, top-to-bottom.
         */

        bool found = cv::findChessboardCorners(gray, gridSize, corners);

        if(found && corners.size() == gridSize.area()){

            /** 
             * void cv::cornerSubPix(
             *      InputArray image,        // grayscale input image
             *      InputOutputArray corners,// initial corners to refine
             *      Size winSize,            // search window size
             *      Size zeroZone,           // zone around the corner to ignore (often (-1,-1))
             *      TermCriteria criteria    // termination criteria (precision + max iterations)
             * )
             * 
             * Refines the corner positions to sub-pixel accuracy.
             * Improves the quality of camera calibration and pose estimation.
             */
            cv::cornerSubPix(gray, corners, cv::Size(11,11), cv::Size(-1,-1), criteria);
            image_counter++;

            if(intrinsics.empty()){
                if(image_counter%10==0){
                    points3d.push_back(objectPoints);
                    points2d.push_back(corners);
                }

                /** 
                 * void cv::drawChessboardCorners(
                 *      InputOutputArray image,  // image to draw corners on
                 *      Size patternSize,        // chessboard size
                 *      InputArray corners,      // 2D points detected
                 *      bool patternWasFound     // true if corners were found
                 * )
                 * 
                 * Draws the detected chessboard corners for visualization.
                 * Helps verify that corners are correctly detected.
                 */
                cv::drawChessboardCorners(frame, gridSize, corners, found);
            }
            else{
                
                /* ----------------------
                    POSE ESTIMATION
                -----------------------*/
                cv::Mat rvec, tvec;

                /** 
                 * bool cv::solvePnPRansac(
                 *      InputArray objectPoints, // 3D points of the chessboard
                 *      InputArray imagePoints,  // corresponding detected 2D points
                 *      InputArray cameraMatrix, // intrinsic camera matrix
                 *      InputArray distCoeffs,   // distortion coefficients
                 *      OutputArray rvec,        // rotation vector (3x1)
                 *      OutputArray tvec,        // translation vector (3x1)
                 *      bool useExtrinsicGuess=false,
                 *      int iterationsCount=100,
                 *      float reprojectionError=8.0,
                 *      int confidence=0.99,
                 *      OutputArray inliers=noArray(),
                 *      int flags=SOLVEPNP_ITERATIVE
                 * )
                 * 
                 * Estimates the pose (rotation + translation) of the chessboard relative to the camera.
                 * RANSAC makes the estimation robust to outliers.
                 * rvec and tvec can be used to project other 3D objects into the image.
                 */
                cv::solvePnPRansac(objectPoints, corners, intrinsics, distortion, rvec, tvec);

                std::vector<cv::Point2f> projectedPoints;

                /** 
                 * void cv::projectPoints(
                 *      InputArray objectPoints, // 3D points to project
                 *      InputArray rvec,         // rotation vector (3x1)
                 *      InputArray tvec,         // translation vector (3x1)
                 *      InputArray cameraMatrix, // intrinsic camera matrix
                 *      InputArray distCoeffs,   // distortion coefficients
                 *      OutputArray imagePoints  // resulting 2D projected points
                 * )
                 * 
                 * Projects 3D points into the image plane using the estimated pose.
                 * Useful for rendering virtual 3D objects (cube, prism, etc.) onto a real scene.
                 */
                cv::projectPoints(cube, rvec, tvec, intrinsics, distortion, projectedPoints);

                if(projectedPoints.size() == cube.size()){
                    frame = drawCube(frame, projectedPoints);
                }
            }
        }

        cv::imshow("AR_demo", frame);

        if(key=='c' && intrinsics.empty() && !points2d.empty()){
            cv::calibrateCamera(points3d, points2d, frame.size(), intrinsics, distortion, cv::noArray(), cv::noArray());
            std::cout << "Calibration completed!\n";
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
