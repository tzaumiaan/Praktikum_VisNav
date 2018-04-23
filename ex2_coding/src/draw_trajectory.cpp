#include <sophus/se3.hpp>
#include <string>
#include <iostream>
#include <fstream>

// for usleep in mac osx
#include <unistd.h>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;

// path to trajectory file
string estimated_trajectory_file = "../data/estimated.txt";
string ground_truth_trajectory_file = "../data/groundtruth.txt";

// trajectory is a vector of SE3 poses
typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue

void DrawTrajectory(const TrajectoryType &estimated, const TrajectoryType &groundtruth);

int main(int argc, char **argv) {

    TrajectoryType estimated, groundtruth;

    /// implement pose reading code
    // start your code here (5~10 lines)
    // end your code here

    // draw trajectory in pangolin
    DrawTrajectory(estimated, groundtruth);

    /// you can also use this code to compute the ATE and RMSE alignment error.

    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(const TrajectoryType &estimated, const TrajectoryType &groundtruth) {

    if (estimated.empty() || groundtruth.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
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

        glLineWidth(2);
        glColor3f(1.0, 0.0f, 0.0f); // red for estimated
        for (size_t i = 0; i < estimated.size() - 1; i++) {
            glBegin(GL_LINES);
            auto p1 = estimated[i], p2 = estimated[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        glColor3f(0.0, 0.0f, 1.0f); // blue for ground-truth
        for (size_t i = 0; i < groundtruth.size() - 1; i++) {
            glBegin(GL_LINES);
            auto p1 = groundtruth[i], p2 = groundtruth[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}
