#include "ceres/ceres.h"
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <iostream>
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
// Data generated using the following octave code.
//   randn('seed', 23497);
//   m = 0.3;
//   c = 0.1;
//   x=[0:0.075:5];
//   y = exp(m * x + c);
//   noise = randn(size(x)) * 0.2;
//   y_observed = y + noise;
//   data = [x', y_observed'];
namespace py = pybind11;


struct ExponentialResidual {
  ExponentialResidual(double x, double y)
      : x_(x), y_(y) {}
  template <typename T> bool operator()(const T* const m,
                                        const T* const c,
                                        T* residual) const {
    residual[0] = y_ - exp(m[0] * x_ + c[0]);
    return true;
  }
 private:
  const double x_;
  const double y_;
};

int solve_pnp(py::array_t<double> data, py::array_t<double> m, py::array_t<double> c) {
  auto r = data.unchecked<2>();
  auto mm = m.mutable_unchecked<1>();
  auto cc = c.mutable_unchecked<1>();
  // py::buffer_info info = m.request();
  // auto ptr_m = static_cast<double *>(info.ptr);
  // py::buffer_info info_c = c.request();
  // auto ptr_c = static_cast<double *>(info_c.ptr);

  Problem problem;
  for (int i = 0; i < r.shape(0); ++i) {
    problem.AddResidualBlock(
        new AutoDiffCostFunction<ExponentialResidual, 1, 1, 1>(
            new ExponentialResidual(r(i, 0), r(i, 1))),
        NULL,
        &mm(0), &cc(0));
  }
  // *ptr_c = c;
  Solver::Options options;
  options.max_num_iterations = 25;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;
  Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";
  // std::cout << "Initial m: " << 0.0 << " c: " << 0.0 << "\n";
  // std::cout << "Final   m: " << m_ << " c: " << c_ << "\n";
  return 0;
}


PYBIND11_PLUGIN(curve_fit) {
    py::module m("curve_fit", "auto-compiled c++ extension");
    m.def("solve_pnp", &solve_pnp);
    return m.ptr();
}