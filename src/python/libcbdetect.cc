#include <pybind11/pybind11.h>
#include "libcbdetect/boards_from_corners.h"
#include "libcbdetect/config.h"
#include "libcbdetect/find_corners.h"
#include "libcbdetect/plot_boards.h"
#include "libcbdetect/plot_corners.h"
#include "cvnp/cvnp.h"
#include <opencv2/opencv.hpp>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

cv::Mat add(cv::Mat a, cv::Mat b) {
    return a + b;
}

namespace py = pybind11;

PYBIND11_MODULE(cbdetect_py, m) {
    m.doc() = R"pbdoc(
        Corner detector
        -----------------------

        .. currentmodule:: cbdetect_py

        .. autosummary::
           :toctree: _generate

           add
           subtract
    )pbdoc";

    m.def("add", &add, R"pbdoc(
        Add two matrices

        Some other explanation about the add function.
    )pbdoc");

    // m.def("subtract", [](int i, int j) { return i - j; }, R"pbdoc(
    //     Subtract two numbers
    //
    //     Some other explanation about the subtract function.
    // )pbdoc");

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
