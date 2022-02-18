#include "dynamical_systems_bindings.h"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

PYBIND11_MODULE(dynamical_systems, m) {
  m.doc() = "Python bindings for control libraries dynamical_systems";

  #ifdef MODULE_VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(MODULE_VERSION_INFO);
  #else
  m.attr("__version__") = "dev";
  #endif

  bind_type(m);
  bind_cartesian(m);
  bind_joint(m);
}
