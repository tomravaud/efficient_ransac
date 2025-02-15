// nanobind
// core
#include <nanobind/nanobind.h>
// implicit type conversion
#include <nanobind/stl/filesystem.h>
#include <nanobind/stl/string.h>

// custom
#include "detector/detector.h"
#include "viewer/viewer.h"

// aliases
namespace nb = nanobind;
using namespace nb::literals;
using namespace efficient_ransac;

// bindings
NB_MODULE(_efficient_ransac_mod, m) {
  m.doc() = "Efficient RANSAC Python bindings";

  // viewer
  nb::class_<Viewer>(m, "Viewer")
      .def(nb::init<>())
      // disambiguate overloaded methods
      .def("show_cloud",
           static_cast<void (Viewer::*)(const std::filesystem::path&, bool)>(
               &Viewer::showCloud),
           "filepath"_a, "show_normals"_a = false);

  // detector
  nb::class_<Detector>(m, "Detector")
      .def(nb::init<>())
      .def("detect", &Detector::detect, "filepath"_a);
}
