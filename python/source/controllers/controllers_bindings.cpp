#include "controllers_bindings.h"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

PYBIND11_MODULE(controllers, m) {
  m.doc() = "Python bindings for control libraries controllers";

  #ifdef MODULE_VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(MODULE_VERSION_INFO);
  #else
  m.attr("__version__") = "dev";
  #endif

  py::module_::import("state_representation");

  bind_controller_type(m);
  bind_computational_space(m);
  bind_cartesian_controllers(m);
  bind_joint_controllers(m);
}
