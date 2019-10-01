<%
cfg['compiler_args'] = ['-std=c++11', '-stdlib=libc++', '-mmacosx-version-min=10.7']
setup_pybind11(cfg)
%>
#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

class MyData{

private:
	double* data;
public:
	MyData(int d) {
		data = new double[1000000 * d];
	}
	~MyData() {
		delete data;
		std::cout << "destrctor ran" << "\n";
	}
	int get(int i) {return *data;}
};

// Passing in an array of doubles
void twice(py::array_t<double> xs) {
    py::buffer_info info = xs.request();
    auto ptr = static_cast<double *>(info.ptr);

    int n = 1;
    for (auto r: info.shape) {
      n *= r;
    }

    for (int i = 0; i <n; i++) {
        *ptr++ *= 2;
    }
}

// Passing in a generic array
double sum(py::array_t<double> xs) {
    // py::buffer_info info = xs.request();
    auto r = xs.unchecked<2>(); // x must have ndim = 3; can be non-writeable
    std::cout << typeid(r).name() << std::endl;
    auto mem = new MyData(10000);
    mem->get(0);
   // delete mem;
    double s = 0;
    for (auto i = 0; i < r.shape(0); ++i)
	for (auto j = 0; j < r.shape(1); ++j)
		s += r(i, j);
    return s;
}

PYBIND11_PLUGIN(code) {
    pybind11::module m("code", "auto-compiled c++ extension");
    m.def("sum", &sum);
    m.def("twice", &twice);
    return m.ptr();
}
