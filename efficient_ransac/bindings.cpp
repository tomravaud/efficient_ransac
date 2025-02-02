// nanobind
// core
#include <nanobind/nanobind.h>
// implicit type conversion
#include <nanobind/stl/string.h>
#include <nanobind/stl/filesystem.h>

// custom
#include "viewer/viewer.h"

// aliases
namespace nb = nanobind;
using namespace nb::literals;


// bindings
NB_MODULE(_efficient_ransac_mod, m){
    m.doc() = "Efficient RANSAC Python bindings";

    // viewer
    nb::class_<Viewer>(m, "Viewer")
        .def(nb::init<>())
        .def(
            "show_cloud",
            static_cast<void(Viewer::*)(
                const std::filesystem::path &
            )>(&Viewer::showCloud),
            "filepath"_a
        )
        ;
}
