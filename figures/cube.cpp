#include <stdio.h>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "cube.h"

cv::Mat drawCube(cv::Mat image, const std::vector<cv::Point2f>& points2d) {
    std::vector<cv::Point> pts(points2d.begin(), points2d.end());

    // disegna il pavimento (primi 4 punti)
    std::vector<cv::Point> floor(pts.begin(), pts.begin() + 4);
    cv::drawContours(image, std::vector<std::vector<cv::Point>>{floor}, -1, cv::Scalar(0, 255, 0), 3);

    // disegna gli spigoli verticali
    for (int i = 0; i < 4; ++i) {
        cv::line(image, pts[i], pts[i + 4], cv::Scalar(255, 0, 0), 3);
    }

    // disegna la parte superiore
    std::vector<cv::Point> top(pts.begin() + 4, pts.end());
    cv::drawContours(image, std::vector<std::vector<cv::Point>>{top}, -1, cv::Scalar(0, 0, 255), 3);

    return image;
}
