#ifndef DEF_BAL_HPP
#define DEF_BAL_HPP

#include <iostream>
using namespace std;
// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
// Ceres for AutoDiff
#include <ceres/internal/autodiff.h>
// g2o
#include "g2o/core/batch_stats.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
// local library
#include "lib_math.hpp"
#include "lib_project.hpp"

// vertex calss for camera
class VertexCameraBAL : public g2o::BaseVertex<9, Eigen::VectorXd>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexCameraBAL(){} // nothing
    virtual bool read(std::istream& /*is*/){ return false; } // not implemented
    virtual bool write(std::ostream& /*os*/) const{ return false; } // not implemented
    virtual void setToOriginImpl(){} // not implemented
    virtual void oplusImpl(const double* update){
      Eigen::VectorXd::ConstMapType v(update, VertexCameraBAL::Dimension);
      _estimate += v;
    }
};

// vertex class for points
class VertexPointBAL : public g2o::BaseVertex<3, Eigen::Vector3d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexPointBAL(){} // nothing
    virtual bool read(std::istream& /*is*/){ return false; } // not implemented
    virtual bool write(std::ostream& /*os*/) const{ return false; } // not implemented
    virtual void setToOriginImpl(){} // not implemented
    virtual void oplusImpl(const double* update){
      Eigen::Vector3d::ConstMapType v(update);
      _estimate += v;
    }
};

// edge representation for a point observed by a camera
class EdgeObservationBAL : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexCameraBAL, VertexPointBAL>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeObservationBAL(){} // nothing
    virtual bool read(std::istream& /*is*/){ return false; } // not implemented
    virtual bool write(std::ostream& /*os*/) const{ return false; } // not implemented

    template<typename T>
    bool operator()(const T* camera, const T* point, T* error) const
    {
      T prediction[2];
      cam_project_w_dist(camera, point, prediction);
      error[0] = prediction[0] - T(measurement()(0));
      error[1] = prediction[1] - T(measurement()(1));
      return true;
    }

    void computeError()
    {
      const VertexCameraBAL* cam = static_cast<const VertexCameraBAL*>(vertex(0));
      const VertexPointBAL* point = static_cast<const VertexPointBAL*>(vertex(1));
      (*this)(cam->estimate().data(), point->estimate().data(), _error.data());
    }

    void linearizeOplus()
    {
      const VertexCameraBAL* cam = static_cast<const VertexCameraBAL*>(vertex(0));
      const VertexPointBAL* point = static_cast<const VertexPointBAL*>(vertex(1));
      typedef ceres::internal::AutoDiff<EdgeObservationBAL, double, VertexCameraBAL::Dimension, VertexPointBAL::Dimension> BalAutoDiff;

      Eigen::Matrix<double, Dimension, VertexCameraBAL::Dimension, Eigen::RowMajor> dError_dCamera;
      Eigen::Matrix<double, Dimension, VertexPointBAL::Dimension, Eigen::RowMajor> dError_dPoint;
      double *parameters[] = { const_cast<double*>(cam->estimate().data()), const_cast<double*>(point->estimate().data()) };
      double *jacobians[] = { dError_dCamera.data(), dError_dPoint.data() };
      double value[Dimension];
      bool diffState = BalAutoDiff::Differentiate(*this, parameters, Dimension, value, jacobians);

      // copy over the Jacobians (convert row-major -> column-major)
      if (diffState) {
        _jacobianOplusXi = dError_dCamera;
        _jacobianOplusXj = dError_dPoint;
      } else {
        assert(0 && "Error while differentiating");
        _jacobianOplusXi.setZero();
        _jacobianOplusXi.setZero();
      }
    }
};

// data structure for a BAL dataset
class bal_dataset
{
  public:
    bal_dataset(){}
    bal_dataset(const string infile){
      cameras.clear();
      points.clear();
      obs.clear();
      read_from_text(infile);
    }
    
    // read in from file
    void read_from_text(const string& infile){
      cout << "Loading BAL dataset " << infile << endl;
      ifstream ifs(infile.c_str());
      int numCameras, numPoints, numObservations;
      ifs >> numCameras >> numPoints >> numObservations;
      
      cerr 
        << PVAR(numCameras) << " " 
        << PVAR(numPoints) << " " 
        << PVAR(numObservations) << endl;
      
      int id = 0;

      cameras.reserve(numCameras);
      for (int i = 0; i < numCameras; i++) {
        VertexCameraBAL* cam = new VertexCameraBAL;
        cam->setId(id); id++;
        cameras.push_back(cam);
      }
      
      points.reserve(numPoints);
      for (int i = 0; i < numPoints; i++) {
        VertexPointBAL* p = new VertexPointBAL;
        p->setId(id); id++;
        p->setMarginalized(true);
        points.push_back(p);
      }
      
      // read in the observation
      obs.reserve(numObservations);
      for (int i = 0; i < numObservations; i++) {
        int camIndex, pointIndex;
        double obsX, obsY;
        ifs >> camIndex >> pointIndex >> obsX >> obsY;
        assert(camIndex >= 0 && (size_t)camIndex < cameras.size() && "Index out of bounds");
        VertexCameraBAL* cam = cameras[camIndex];
        assert(pointIndex >= 0 && (size_t)pointIndex < points.size() && "Index out of bounds");
        VertexPointBAL* point = points[pointIndex];
        EdgeObservationBAL* e = new EdgeObservationBAL;
        e->setVertex(0, cam);
        e->setVertex(1, point);
        e->setInformation(Eigen::Matrix2d::Identity());
        e->setMeasurement(Eigen::Vector2d(obsX, obsY));
        obs.push_back(e);
      }
      
      // read in the camera params
      Eigen::VectorXd cameraParameter(9);
      for (int i = 0; i < numCameras; i++) {
        for (int j = 0; j < 9; j++){ ifs >> cameraParameter(j); }
        VertexCameraBAL* cam = cameras[i]; // get the reference of camera pushed in a vector
        cam->setEstimate(cameraParameter); // append the parameter to it
      }
      
      // read in the points
      Eigen::Vector3d p;
      for (int i = 0; i < numPoints; ++i) {
        ifs >> p(0) >> p(1) >> p(2);
        VertexPointBAL* point = points[i]; // get the reference of point pushed in a vector
        point->setEstimate(p); // append the parameter to it
      }
      
      // add cameras, points, and obs in optimizer
      for(auto& c: cameras){ optimizer.addVertex(c); }
      for(auto& p: points){ optimizer.addVertex(p); }
      for(auto& o: obs){ optimizer.addEdge(o); }

      cout << "done." << endl;
    }// end of read_from_text()
    
    // write to VRML
    void write_to_vrml(const string& outfile){
      assert(outfile.size()>0);
      ofstream fout(outfile.c_str()); // loadable with meshlab
      fout 
        << "#VRML V2.0 utf8\n"
        << "Shape {\n"
        << "  appearance Appearance {\n"
        << "    material Material {\n"
        << "      diffuseColor " << 1 << " " << 0 << " " << 0 << "\n"
        << "      ambientIntensity 0.2\n"
        << "      emissiveColor 0.0 0.0 0.0\n"
        << "      specularColor 0.0 0.0 0.0\n"
        << "      shininess 0.2\n"
        << "      transparency 0.0\n"
        << "    }\n"
        << "  }\n"
        << "  geometry PointSet {\n"
        << "    coord Coordinate {\n"
        << "      point [\n";
      for (vector<VertexPointBAL*>::const_iterator it = points.begin(); it != points.end(); ++it) {
        fout << (*it)->estimate().transpose() << endl;
      }
      fout << "    ]\n" << "  }\n" << "}\n" << "  }\n";
    } // end of write_to_vrml()
    
    // write to PLY
    void write_to_ply(const string outfile){
      assert(outfile.size()>0);
      ofstream fout(outfile.c_str()); // loadable with meshlab
      fout
        << "ply\n" 
        << "format ascii 1.0\n"
        << "element vertex " << cameras.size() + points.size() << '\n'
        << "property float x\n"
        << "property float y\n"
        << "property float z\n"
        << "property uchar red\n"
        << "property uchar green\n"
        << "property uchar blue\n"
        << "end_header" << std::endl;
      
      // Export extrinsic data (i.e. camera centers) as green points.
      double cam[9], aa[3], ct[3]; 
      for(int i = 0; i < cameras.size(); i++){
        for(int j = 0; j<9; j++){ cam[j] = (double)cameras[i]->estimate()[j]; }
        cam_to_aa_ct(cam, aa, ct);
        fout << ct[0] << ' ' << ct[1] << ' ' << ct[2] << " 0 255 0\n";
      }
      
      // Export the structure (i.e. 3D Points) as white points.
      for(int i = 0; i < points.size(); i++){
        for(int j = 0; j < 3; j++){ fout << (double)points[i]->estimate()[j] << ' '; }
        fout << "255 255 255\n";
      }
      
      fout.close();
    }

    void init_opt(){
      typedef g2o::BlockSolver< g2o::BlockSolverTraits<9, 3> >  BalBlockSolver;
      typedef g2o::LinearSolverCholmod<BalBlockSolver::PoseMatrixType> BalLinearSolver;
      std::unique_ptr<g2o::LinearSolver<BalBlockSolver::PoseMatrixType>> linearSolver;
      auto cholesky = g2o::make_unique<BalLinearSolver>();
      cholesky->setBlockOrdering(true);
      linearSolver = std::move(cholesky);
      g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BalBlockSolver>(std::move(linearSolver))
      );
      optimizer.setAlgorithm(solver);
    } // end of init_opt()

    void run_opt(const int max_iter){
      cout << "Optimizer initializing ... " << flush;
      optimizer.initializeOptimization();
      cout << "done." << endl;
      optimizer.setVerbose(true);
      cout << "Start to optimize" << endl;
      optimizer.optimize(max_iter);
    } // end of run_opt()

  private:
    vector<VertexCameraBAL*> cameras;
    vector<VertexPointBAL*> points;
    vector<EdgeObservationBAL*> obs;
    g2o::SparseOptimizer optimizer; 
};

#endif
