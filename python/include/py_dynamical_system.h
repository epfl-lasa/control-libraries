#include <dynamical_systems/IDynamicalSystem.hpp>
#include <state_representation/parameters/ParameterInterface.hpp>

template<class S>
class PyDynamicalSystem : public IDynamicalSystem<S>, public std::enable_shared_from_this<PyDynamicalSystem<S>> {
public:
  using IDynamicalSystem<S>::IDynamicalSystem;

  [[nodiscard]] bool is_compatible(const S& state) const override {
    PYBIND11_OVERRIDE(bool, IDynamicalSystem<S>, is_compatible, state);
  }

  void set_base_frame(const S& base_frame) override {
    PYBIND11_OVERRIDE(void, IDynamicalSystem<S>, set_base_frame, base_frame);
  }

protected:
  [[nodiscard]] S compute_dynamics(const S& state) const override {
    PYBIND11_OVERRIDE_PURE(S, IDynamicalSystem<S>, compute_dynamics, state);
  }

  void validate_and_set_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter) override {
    PYBIND11_OVERRIDE_PURE(void, IDynamicalSystem<S>, validate_and_set_parameter, parameter);
  }
};
