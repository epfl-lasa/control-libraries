#include "robot_model_bindings.h"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

PYBIND11_MODULE(robot_model, m) {
  m.doc() = "Python bindings for control libraries robot_model";

  #ifdef MODULE_VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(MODULE_VERSION_INFO);
  #else
  m.attr("__version__") = "dev";
  #endif

  bind_model(m);
}