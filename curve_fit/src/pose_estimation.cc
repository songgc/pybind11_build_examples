#include "ceres/ceres.h"
#include "sophus/geometry.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <iostream>
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

namespace py = pybind11;

using se = Sophus::SE3d;

class BAGNCostFunctor : public ceres::SizedCostFunction<2, 6> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BAGNCostFunctor(Eigen::Vector2d observed_p, Eigen::Vector3d observed_P) :
            observed_p_(observed_p), observed_P_(observed_P) {}

    virtual ~BAGNCostFunctor() {}

    virtual bool Evaluate(
      double const* const* parameters, double *residuals, double **jacobians) const {

        Eigen::Map<const Eigen::Matrix<double,6,1>> T_se3(*parameters);

        se T_SE3 = se::exp(T_se3);

        Eigen::Vector3d Pc = T_SE3 * observed_P_;

        Eigen::Matrix3d K;
        // hard coded !!!
        double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
        K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

        Eigen::Vector2d residual =  observed_p_ - (K * Pc).hnormalized();

        residuals[0] = residual[0];
        residuals[1] = residual[1];

        if(jacobians != NULL) {

            Eigen::Matrix<double, 2, 6> J;

            double x = Pc[0];
            double y = Pc[1];
            double z = Pc[2];

            double x2 = x*x;
            double y2 = y*y;
            double z2 = z*z;

            J(0,0) = -fx/z;
            J(0,1) =  0;
            J(0,2) =  fx*x/z2;
            J(0,3) =  fx*x*y/z2;
            J(0,4) = -fx-fx*x2/z2;
            J(0,5) =  fx*y/z;
            J(1,0) =  0;
            J(1,1) = -fy/z;
            J(1,2) =  fy*y/z2;
            J(1,3) =  fy+fy*y2/z2;
            J(1,4) = -fy*x*y/z2;
            J(1,5) = -fy*x/z;

            int k=0;
            for(int i=0; i<2; ++i) {
                for(int j=0; j<6; ++j) {
                    jacobians[0][k++] = J(i,j);
                }
            }
        }

        return true;
    }

private:
    const Eigen::Vector2d observed_p_;
    const Eigen::Vector3d observed_P_;
};


int solve_pnp(py::array_t<double> points_3d, py::array_t<double> points_2d,
              py::array_t<double> se_matrix) {
  auto p3ds = points_3d.unchecked<2>();
  auto p2ds = points_2d.unchecked<2>();
  auto sem = se_matrix.mutable_unchecked<2>();
  Sophus::Vector6d se3;
  ceres::Problem problem;
  auto loss_function = new ceres::CauchyLoss(2.0);

  std::cout << p3ds.shape(0) << "\n";
  for(int i=0; i < p3ds.shape(0); ++i) {
      ceres::CostFunction *cost_function;
      Eigen::Vector2d p2d = {p2ds(i, 0), p2ds(i, 1)};
      Eigen::Vector3d p3d = {p3ds(i, 0), p3ds(i, 1), p3ds(i, 2)};
      cost_function = new BAGNCostFunctor(p2d, p3d);
      problem.AddResidualBlock(cost_function, loss_function, se3.data());
  }

  ceres::Solver::Options options;
  options.dynamic_sparsity = true;
  options.max_num_iterations = 100;
  options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
  options.minimizer_type = ceres::TRUST_REGION;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.trust_region_strategy_type = ceres::DOGLEG;
  options.minimizer_progress_to_stdout = true;
  options.dogleg_type = ceres::SUBSPACE_DOGLEG;
  options.logging_type = ceres::SILENT;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  auto sem_result = se::exp(se3).matrix();
   
  for (auto i = 0; i < 4; ++i) 
    for (auto j = 0; j < 4; ++j)
      sem(i, j) = sem_result(i, j);
  
//   std::cout << summary.BriefReport() << "\n";
//   std::cout << "estimated pose: " << se3 << "\n"
    // << se::exp(se3).matrix() << "\n";
  
  return summary.termination_type;
}


PYBIND11_MODULE(pose_estimation, m) {
    m.doc() = "pose_estimation";
    m.def("solve_pnp", &solve_pnp);
}