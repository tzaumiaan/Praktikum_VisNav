//
// Created by xiang on 1/4/18.
// this program shows how to perform photometric bundle adjustment
// we use g2o as an example, if you use Ceres please change this code!
//
#include <iostream>

using namespace std;

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>

#include <pangolin/pangolin.h>
#include <boost/format.hpp>

typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> VecSE3;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVec3d;

// global variables
string pose_file = "../data/poses.txt";
string points_file = "../data/points.txt";

// intrinsics
float fx = 277.34;
float fy = 291.402;
float cx = 312.234;
float cy = 239.777;

// bilinear interpolation
inline float GetPixelValue(const cv::Mat &img, float x, float y)
{
  uchar *data = &img.data[int(y) * img.step + int(x)];
  float xx = x - floor(x);
  float yy = y - floor(y);
  return float(
      (1 - xx) * (1 - yy) * data[0] +
      xx * (1 - yy) * data[1] +
      (1 - xx) * yy * data[img.step] +
      xx * yy * data[img.step + 1]
  );
}

// g2o vertex that use sophus::SE3 as pose
class VertexSophus : public g2o::BaseVertex<6, Sophus::SE3d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexSophus() {}

  ~VertexSophus() {}

  bool read(std::istream &is){return 0;}

  bool write(std::ostream &os) const{return 0;}

  virtual void setToOriginImpl()
  {
    _estimate = Sophus::SE3d();
  }

  virtual void oplusImpl(const double *update_)
  {
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> update(update_);
    setEstimate(Sophus::SE3d::exp(update) * estimate());
  }
};

// TODO edge of projection error, implement it
// 16x1 error, which is the errors in patch
typedef Eigen::Matrix<double, 16, 1> Vector16d;

class EdgeDirectProjection : public g2o::BaseBinaryEdge<16, Vector16d, g2o::VertexSBAPointXYZ, VertexSophus>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  EdgeDirectProjection(float *color, cv::Mat &target)
  {
    this->origColor = color;
    this->targetImg = target;
  }

  ~EdgeDirectProjection() {}

  virtual void computeError() override
  {
    // TODO START YOUR CODE HERE
    Eigen::Matrix3d K;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
    // compute projection error ...
    const g2o::VertexSBAPointXYZ* points = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
    const VertexSophus* poses = static_cast<const VertexSophus*>(_vertices[1]);
    const Sophus::SE3d T = poses->estimate();
    Eigen::Vector3d pt_orig = points->estimate();
    Eigen::Vector3d pt_proj = T * pt_orig.matrix();
    pt_proj = K*pt_proj;
    double x = pt_proj[0]/pt_proj[2], y = pt_proj[1]/pt_proj[2];
    // boundary check
    if( x<2 || x>=(targetImg.cols-2) ||
        y<2 || y>=(targetImg.rows-2) 
    ){
      // outside boundary, error invalid
      for(int n=0; n<16; n++){ _error[n] = 0.0; }
    }else{
      // compute error from projection and original
      int n = 0;
      for(int i=-2; i<2; i++){
        for(int j=-2; j<2; j++){
          double i_orig = origColor[n];
          double i_proj = GetPixelValue(targetImg, x+i, y+j);
          _error[n] = abs(i_orig - i_proj);
          n++;
        }
      }
    }
    // END YOUR CODE HERE
  }

  // Let g2o compute jacobian for you

  virtual bool read(istream &in){}

  virtual bool write(ostream &out) const{}

private:
  cv::Mat targetImg;  // the target image
  float *origColor = nullptr;   // 16 floats, the color of this point
};

// plot the poses and points for you, need pangolin
void Draw(const VecSE3 &poses, const VecVec3d &points);

int main(int argc, char **argv)
{

  // read poses and points
  VecSE3 poses;
  VecVec3d points;
  ifstream fin(pose_file);

  while (!fin.eof())
  {
    double timestamp = 0;
    fin >> timestamp;
    if (timestamp == 0) break;
    double data[7];
    for (auto &d: data) fin >> d;
    poses.push_back(Sophus::SE3d(
        Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
        Eigen::Vector3d(data[0], data[1], data[2])
    ));
    if (!fin.good()) break;
  }
  fin.close();


  vector<float *> color;
  fin.open(points_file);
  while (!fin.eof())
  {
    double xyz[3] = {0};
    for (int i = 0; i < 3; i++) fin >> xyz[i];
    if (xyz[0] == 0) break;
    points.push_back(Eigen::Vector3d(xyz[0], xyz[1], xyz[2]));
    float *c = new float[16];
    for (int i = 0; i < 16; i++) fin >> c[i];
    color.push_back(c);

    if (fin.good() == false) break;
  }
  fin.close();

  cout << "poses: " << poses.size() << ", points: " << points.size() << endl;

  // read images
  vector<cv::Mat> images;
  boost::format fmt("../data/%d.png");
  for (int i = 0; i < 7; i++)
  {
    images.push_back(cv::imread((fmt % i).str(), 0));
  }

  // build optimization problem
  // setup g2o
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block;  // pose is 6x1, landmark is 3x1
  std::unique_ptr<Block::LinearSolverType> linearSolver(
      new g2o::LinearSolverDense<Block::PoseMatrixType>()); // linear solver

  // use levernberg-marquardt here (or you can choose gauss-newton)
  g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(
      g2o::make_unique<Block>(std::move(linearSolver)));
  g2o::SparseOptimizer optimizer;     // graph optimizer
  optimizer.setAlgorithm(solver);   // solver
  optimizer.setVerbose(true);       // open the output

  // TODO add vertices, edges into the graph optimizer
  // START YOUR CODE HERE
  Eigen::Matrix3d K;
  K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
  int id = 0;
  // add camera poses
  vector<VertexSophus*> poses_opt;
  for(int i=0; i<images.size(); i++){
    VertexSophus* ps = new VertexSophus();
    ps->setId(id); id++;
    ps->setEstimate(poses[i]);
    poses_opt.push_back(ps);
    optimizer.addVertex(ps);
  }
  // add points
  vector<g2o::VertexSBAPointXYZ*> points_opt;
  for(int i=0; i<points.size(); i++){
    g2o::VertexSBAPointXYZ* pt = new g2o::VertexSBAPointXYZ();
    pt->setId(id); id++;
    pt->setEstimate(points[i]);
    pt->setMarginalized(true);
    points_opt.push_back(pt);
    optimizer.addVertex(pt);
  }
  // add edges
  int edge_id = 0;
  for(int i=0; i<images.size(); i++){
    for(int j=0; j<points.size(); j++){
      Eigen::Matrix3d R = poses[i].rotationMatrix();
      Eigen::Vector3d t = poses[i].translation();
      Eigen::Vector3d pt_proj = R * points[j].matrix() + t;
      pt_proj = K*pt_proj;
      int u = pt_proj[0]/pt_proj[2], v = pt_proj[1]/pt_proj[2];
      if( u < 2 || u >= (images[i].cols - 2) ||
          v < 2 || v >= (images[i].rows - 2)
      ){
        continue;
      }
      EdgeDirectProjection* e = new EdgeDirectProjection(color[j], images[i]);
      e->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(j+images.size())));
      e->setVertex(1, dynamic_cast<VertexSophus*>(optimizer.vertex(i)));
      e->setInformation(Eigen::Matrix<double, 16, 16>::Identity()*1e5);
      e->setId(edge_id); edge_id++;
      optimizer.addEdge(e);
    }
  }
  cout
    << "Problem loaded." << endl
    << "  Number of images: " << images.size() << endl
    << "  Number of points: " << points.size() << endl
    << "  Number of edges: " << optimizer.edges().size() << endl;
  // plot the original points and poses
  //Draw(poses, points);
  // END YOUR CODE HERE

  // perform optimization
  optimizer.initializeOptimization(0);
  optimizer.optimize(200);

  // TODO fetch data from the optimizer
  // START YOUR CODE HERE
  // update poses 
  cout << endl;
  cout << "Before optimization:" << endl;
  for(int i=0; i<poses.size(); i++){
    cout
      << "poses " << i << ": " << endl
      << poses[i].matrix() << endl;
  }
  cout << "After optimization:" << endl;
  for(int i=0; i<poses_opt.size(); i++){
    poses[i] = poses_opt[i]->estimate();
    cout
      << "poses " << i << ": " << endl
      << poses[i].matrix() << endl;
  }
  // update points
  for(int i=0; i<points_opt.size(); i++){
    points[i] = points_opt[i]->estimate();
  }

  // END YOUR CODE HERE
  // plot the optimized points and poses
  Draw(poses, points);

  // delete color data
  for (auto &c: color) delete[] c;
  return 0;
}

void Draw(const VecSE3 &poses, const VecVec3d &points)
{
  if (poses.empty() || points.empty())
  {
    cerr << "parameter is empty!" << endl;
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


  while (pangolin::ShouldQuit() == false)
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    d_cam.Activate(s_cam);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

    // draw poses
    float sz = 0.1;
    int width = 640, height = 480;
    for (auto &Tcw: poses)
    {
      glPushMatrix();
      Sophus::Matrix4f m = Tcw.inverse().matrix().cast<float>();
      glMultMatrixf((GLfloat *) m.data());
      glColor3f(1, 0, 0);
      glLineWidth(2);
      glBegin(GL_LINES);
      glVertex3f(0, 0, 0);
      glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
      glVertex3f(0, 0, 0);
      glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
      glVertex3f(0, 0, 0);
      glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
      glVertex3f(0, 0, 0);
      glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
      glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
      glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
      glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
      glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
      glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
      glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
      glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
      glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
      glEnd();
      glPopMatrix();
    }

    // points
    glPointSize(2);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < points.size(); i++)
    {
      glColor3f(0.0, points[i][2] / 4, 1.0 - points[i][2] / 4);
      glVertex3d(points[i][0], points[i][1], points[i][2]);
    }
    glEnd();

    pangolin::FinishFrame();
    usleep(5000);   // sleep 5 ms
  }
}
