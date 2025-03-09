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
           static_cast<void (Viewer::*)(const std::filesystem::path&)>(
               &Viewer::showCloud),
           "filepath"_a);

  // detector
  nb::class_<Detector>(m, "Detector")
      .def(nb::init<const std::filesystem::path&>(), "config_path"_a)
      .def("detect", &Detector::detect, "input_path"_a, "output_path"_a);
}
