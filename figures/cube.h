#ifndef CUBE_H
#define CUBE_H

#include <opencv2/opencv.hpp>
#include <vector>

cv::Mat drawCube(cv::Mat image, const std::vector<cv::Point2f>& pts);

#endif
