#pragma once

#include <controllers/IController.hpp>

#include "py_parameter_map.h"

template<class S>
class PyController
    : public IController<S>, public PyParameterMap, public std::enable_shared_from_this<PyController<S>> {
public:
  using IController<S>::IController;

  [[nodiscard]] S compute_command(const S& command_state, const S& feedback_state) override {
    PYBIND11_OVERRIDE_PURE(S, IController<S>, compute_command, command_state, feedback_state);
  }
};
