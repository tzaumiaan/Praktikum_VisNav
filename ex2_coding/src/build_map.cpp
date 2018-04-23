#include <opencv2/opencv.hpp>
#include <string>
#include <Eigen/Core>
#include <pangolin/pangolin.h>
#include <boost/format.hpp>
#include <sophus/se3.hpp>

using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>> PointCloudType;

// show point cloud in pangolin
// Point cloud is defined as a vector of Vector6d, where the first three components are XYZ and the last three one are RGB
void showPointCloud(const PointCloudType &pointcloud);

// use boost::format to read image files
boost::format fmt_rgb("./RGBD%d.png");
boost::format fmt_depth("./depth%d.pgm");
string extrinsic_file = "./RGBD-extrinsic.txt";

int main() {

    vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> poses;
    vector<cv::Mat> RGBImgs, DepthImgs;
    int width = 640, height = 480;
    double depth_scale = 1000.0;

    // intrinsics
    double fx = 518.0;
    double fy = 519.0;
    double cx = 325.5;
    double cy = 253.5;

    // read image files
    for (int i = 0; i < 5; i++) {
        RGBImgs.push_back(cv::imread((fmt_rgb % (i + 1)).str())); // CV_8UC3 images
        DepthImgs.push_back(cv::imread((fmt_depth % (i + 1)).str(), -1)); // CV_16UC1 images
        assert(RGBImgs[i].data != nullptr);
        assert(DepthImgs[i].data != nullptr);
    }

    // read poses
    ifstream fin(extrinsic_file);
    while (fin.eof() == false) {
        double time = 0, tx, ty, tz, qx, qy, qz, qw;
        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        poses.push_back(Sophus::SE3d(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz)));
        if (!fin.good() || time == 0) break;
    }
    fin.close();

    // build the point cloud map
    PointCloudType pointcloud;

    for (int i = 0; i < 5; i++) {
        // compute each pixel's position in the world
        Sophus::SE3d Twc = poses[i];
        for (int v = 0; v < height; v++) {
            for (int u = 0; u < width; u++) {
                unsigned short d = DepthImgs[i].ptr<unsigned short>(v)[u];
                if (d == 0) continue;
                Vector3d pw;
                // TODO compute the world coordinates for (u,v,d)
                /// start your code here

                /// end your code here

                // read color
                uchar b = RGBImgs[i].data[v * RGBImgs[i].step + u * RGBImgs[i].channels()];
                uchar g = RGBImgs[i].data[v * RGBImgs[i].step + u * RGBImgs[i].channels() + 1];
                uchar r = RGBImgs[i].data[v * RGBImgs[i].step + u * RGBImgs[i].channels() + 2];

                Vector6d p;
                p << pw[0], pw[1], pw[2], r, g, b;
                pointcloud.push_back(p);
            }
        }
    }

    // Show the point cloud
    showPointCloud(pointcloud);

    return 0;
}

void showPointCloud(const PointCloudType &pointcloud) {

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
            glColor3f(p[3] / 255.0, p[4] / 255.0, p[5] / 255.0);
            glVertex3d(p[0], p[1], p[2]);
        }

        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}
