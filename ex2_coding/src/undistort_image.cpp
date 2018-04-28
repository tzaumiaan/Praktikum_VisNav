//
// Created by Xiang on 2017/12/15.
//

#include <opencv2/opencv.hpp>
#include <string>

using namespace std;

string image_file = "../data/test.png";   // the tested image file

int main(int argc, char **argv) {

    // Please implement the undistort code by yourself, DON'T call OpenCV's undistort function
    // Rad-Tan distortion params
    double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;
    // intrinsics
    double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

    cv::Mat image = cv::imread(image_file,0);   // the grayscale image
    int rows = image.rows, cols = image.cols;
    cv::Mat image_undistort = cv::Mat(rows, cols, CV_8UC1);   // image after undistortion

    // fill the undistorted image
    for (int v = 0; v < rows; v++)
        for (int u = 0; u < cols; u++) {

            // here u,v is the coordinates in the undistorted image

            double u_distorted = 0, v_distorted = 0;
            // TODO compute the coordinates in the original image (u_distorted, v_distorted) (~6 lines)
            // start your code here
            double x = (u - cx)/fx;
            double y = (v - cy)/fy;
            double r2 = x*x + y*y;
            double x_distorted = x*(1 + k1*r2 + k2*r2*r2) + 2*p1*x*y + p2*(r2 + 2*x*x); 
            double y_distorted = y*(1 + k1*r2 + k2*r2*r2) + p1*(r2 + 2*y*y) + 2*p2*x*y;
            u_distorted = fx*x_distorted + cx;
            v_distorted = fy*y_distorted + cy;
            // end your code here

            // fill the grayscale value
            if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows) {
                image_undistort.at<uchar>(v, u) = image.at<uchar>((int) v_distorted, (int) u_distorted);
            } else {
                image_undistort.at<uchar>(v, u) = 0;
            }
        }

    // draw the undistorted image
    cv::imshow("image undistorted", image_undistort);
    cv::waitKey();

    return 0;
}
