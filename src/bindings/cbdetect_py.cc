#include "cvnp/cvnp.h"
#include "libcbdetect/boards_from_corners.h"
#include "libcbdetect/config.h"
#include "libcbdetect/find_corners.h"
#include <opencv2/opencv.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;
using namespace cbdetect;
using namespace pybind11::literals;

PYBIND11_MODULE(cbdetect_py, m) {
  m.doc() = R"pbdoc(
        Corner detector
        -----------------------

        .. currentmodule:: cbdetect_py

        .. autosummary::
           :toctree: _generate

    )pbdoc";

  py::enum_<DetectMethod>(m, "DetectMethod")
      .value("TemplateMatchFast", DetectMethod::TemplateMatchFast)
      .value("TemplateMatchSlow", DetectMethod::TemplateMatchSlow)
      .value("HessianResponse", DetectMethod::HessianResponse)
      .value("LocalizedRadonTransform", DetectMethod::LocalizedRadonTransform)
      .export_values();

  py::enum_<CornerType>(m, "CornerType")
      .value("SaddlePoint", CornerType::SaddlePoint)
      .value("MonkeySaddlePoint", CornerType::MonkeySaddlePoint)
      .export_values();

  py::class_<Params>(m, "Params")
      .def(py::init<>())
      .def_readwrite("show_processing", &Params::show_processing)
      .def_readwrite("show_debug_image", &Params::show_debug_image)
      .def_readwrite("show_grow_processing", &Params::show_grow_processing)
      .def_readwrite("norm", &Params::norm)
      .def_readwrite("polynomial_fit", &Params::polynomial_fit)
      .def_readwrite("norm_half_kernel_size", &Params::norm_half_kernel_size)
      .def_readwrite("polynomial_fit_half_kernel_size",
                     &Params::polynomial_fit_half_kernel_size)
      .def_readwrite("init_loc_thr", &Params::init_loc_thr)
      .def_readwrite("score_thr", &Params::score_thr)
      .def_readwrite("strict_grow", &Params::strict_grow)
      .def_readwrite("overlay", &Params::overlay)
      .def_readwrite("occlusion", &Params::occlusion)
      .def_readwrite("detect_method", &Params::detect_method)
      .def_readwrite("corner_type", &Params::corner_type)
      .def_readwrite("radius", &Params::radius);

  py::class_<Corner>(m, "Corner")
      .def(py::init<>())
      .def_readwrite("p", &Corner::p)
      .def_readwrite("r", &Corner::r)
      .def_readwrite("v1", &Corner::v1)
      .def_readwrite("v2", &Corner::v2)
      .def_readwrite("v3", &Corner::v3)
      .def_readwrite("score", &Corner::score);

  py::class_<Board>(m, "Board")
      .def(py::init<>())
      .def_readwrite("idx", &Board::idx)
      .def_readwrite("energy", &Board::energy)
      .def_readwrite("num", &Board::num);
  m.def("find_corners", &find_corners, "Find corners in the image", "img"_a,
        "params"_a);
  m.def("boards_from_corners", &boards_from_corners,
        "Generate boards from the corners", "img"_a, "corners"_a, "params"_a);

#ifdef VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif
}
