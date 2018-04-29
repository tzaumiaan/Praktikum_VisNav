//
// Created by Xiang on 2017/12/15.
//

#include <opencv2/opencv.hpp>
#include <string>
#include <Eigen/Core>
#include <pangolin/pangolin.h>
#include <unistd.h>

using namespace std;
using namespace Eigen;

// file paths, please adjust them if they are different in your computer
string left_file = "../data/left.png";
string right_file = "../data/right.png";
string disparity_file = "../data/disparity.png";

// show point cloud in pangolin
// Point cloud is defined as a vector of Vector4d, where the first three components are XYZ and the last one is the gray-scale value
void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud);

int main(int argc, char **argv) {

    // intrinsic
    double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
    // baseline
    double b = 0.573;

    // read the images
    cv::Mat left = cv::imread(left_file, 0);
    cv::Mat right = cv::imread(right_file, 0);
    cv::Mat disparity = cv::imread(disparity_file, 0); // disparty is CV_8U

    // generate point cloud
    vector<Vector4d, Eigen::aligned_allocator<Vector4d>> pointcloud;

    // TODO Compute point cloud using disparity
    // NOTE if your computer is slow, change v++ and u++ to v++2 and u+=2 to generate a sparser point cloud
    for (int v = 0; v < left.rows; v++)
        for (int u = 0; u < left.cols; u++) {
            Vector4d point(0, 0, 0,
                           left.at<uchar>(v, u) / 255.0); // first three components are XYZ and the last is color

            /// start your code here (~6 lines)
            point[2] = fx * b / (double)(disparity.at<uchar>(v,u));
            point[0] = (u - cx)/fx * point[2];
            point[1] = (v - cy)/fy * point[2];
            point[3] = (double)(left.at<uchar>(v, u)/255.0);
            pointcloud.push_back(point);
            /// end your code here
        }

    // draw the point cloud
    showPointCloud(pointcloud);
    return 0;
}

void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud) {

    if (pointcloud.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            glColor3f(p[3], p[3], p[3]);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}
