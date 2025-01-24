// nanobind
// core
#include <nanobind/nanobind.h>

// custom
#include "addition.h"

// aliases
namespace nb = nanobind;
using namespace nb::literals;


// bindings
NB_MODULE(_efficient_ransac_mod, m){
    m.doc() = "Python bindings";

    m.def("add", &add, "a"_a, "b"_a, "Add two numbers");
}
