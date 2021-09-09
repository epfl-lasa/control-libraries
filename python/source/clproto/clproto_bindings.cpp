#include "clproto_bindings.h"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

PYBIND11_MODULE(clproto, m) {
m.doc() = "Python bindings for control_libraries clproto";

#ifdef MODULE_VERSION_INFO
m.attr("__version__") = MACRO_STRINGIFY(MODULE_VERSION_INFO);
#else
m.attr("__version__") = "dev";
#endif

py::module_::import("state_representation");
bind_clproto(m);
}