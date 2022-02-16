# Controllers

This library introduces a set of controllers to be used in robotic control-loop schemes. 

All controllers have a common interface inheriting from the `IController<S>` class, which is templated to 
operate in a particular space `S`, namely joint space or Cartesian space.

## Constructing a controller

The `ControllerFactory<S>` provides construction helpers for controllers using a factory pattern.
Specific controllers are created and injected into a common `shared_ptr<IController<S>>`.

The `CartesianControllerFactory` and `JointControllerFactory` are shortcuts for the Cartesian and joint space 
controller factories.

The factory provides a static function `create_controller`, which can take a number of inputs.
The first input argument is always the controller type (defined in `controllers/ControllerType.hpp`).

```c++
#include "controllers/ControllerFactory.hpp"

using namespace controllers;
using namespace state_representation;

// create a Cartesian impedance controller
std::shared_ptr<IController<CartesianState>> cart_ctrl;
cart_ctrl = CartesianControllerFactory::create_controller(CONTROLLER_TYPE::IMPEDANCE);

// create a compliant Cartesian twist controller using "auto" to avoid verbose typing
auto twist_ctrl = CartesianControllerFactory::create_controller(CONTROLLER_TYPE::COMPLIANT_TWIST);
```

The `create_controller` function also has an input for the number of control dimensions. The Cartesian factory
ignores this input as the dimensionality can be automatically inferred for Cartesian controllers.
For the joint-space factory however, the number of joints should be provided so that the controller can initialize
all internal properties to the correct dimensionality.

```c++
unsigned int number_of_joints = 7;
auto joint_ctrl = JointControllerFactory::create_controller(CONTROLLER_TYPE::IMPEDANCE, number_of_joints);
```

Some controllers use a `robot_model::Model` object for internal computation. This can be provided as an input
to the `create_controller` function, in which case the number of dimensions is automatically inferred from the model.

```c++
#include "controllers/ControllerFactory.hpp"
#include "robot_model/Model.hpp"

using namespace controllers;

// create a robot model
auto robot = robot_model::Model("my_robot", "/path/to/robot.urdf");

// create a joint-space dissipative controller with a robot model
auto joint_ctrl = JointControllerFactory::create_controller(CONTROLLER_TYPE::DISSIPATIVE, robot);

// create a Cartesian impedance controller operating only on velocity error with a robot model
auto twist_ctrl = CartesianControllerFactory::create_controller(CONTROLLER_TYPE::VELOCITY_IMPEDANCE, robot);
```

Finally, initial controller parameters can be passed to `create_controller` as a list. See the next section
for more details on parameter configuration.

```c++
#include "controllers/ControllerFactory.hpp"
#include "state_representation/parameters/Parameter.hpp"

using namespace controllers;
using namespace state_representation;

std::list<std::shared_ptr<ParameterInterface>> parameters;
parameters.emplace_back(make_shared_parameter("damping", 10.0));
parameters.emplace_back(make_shared_parameter("stiffness", 5.0));
parameters.emplace_back(make_shared_parameter("inertia", 1.0));

auto ctrl = CartesianControllerFactory::create_controller(CONTROLLER_TYPE::IMPEDANCE, parameters);
```

## Using a controller

The controller factory returns a base class pointer `shared_ptr<IController<S>>` for the chosen state type `S`,
pointing to an instance of a specific derived controller. This allows all controllers to share the same consistent
interface for usage and configuration.

### Parameters

Controllers support custom parameterization through the base interface using `state_representation::ParameterMap`
methods:
- `get_parameters()`
- `get_parameter(name)`
- `get_parameter_value<T>(name)`
- `set_parameters(parameters)`
- `set_parameter(parameter)`
- `set_parameter_value(name, value)`

These methods can be used after construction to get or set controller parameters. Refer to the documentation
on `controllers::IController<>` and `state_representation::ParameterMap` for more information.

For controllers that use a `robot_model::Model` object, there are additional interface methods that can be used
to get or set the current robot model. Note that changing the robot model dimensionality, reference frame or joint
frames at runtime could cause unexpected behaviour.
- `get_robot_model()`
- `set_robot_model(robot_model)`

### Compute command

The main purpose of each controller is the `compute_command` function, which calculates an output state from a 
desired command state and feedback state. For example, the impedance controller takes the error between the command
state and the feedback state and multiplies it by some linear gains to compute the output state. 

```c++
#include "controllers/ControllerFactory.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"

using namespace controllers;

// create a Cartesian impedance controller
auto ctrl = CartesianControllerFactory::create_controller(CONTROLLER_TYPE::IMPEDANCE);

auto command_state = CartesianState::Random("command");
auto feedback_state = CartesianState::Random("feedback");

// compute the command output
auto command_output = ctrl->compute_command(command_state, feedback_state);
```

For Cartesian controllers, `compute_command` is overloaded with extra parameters to enable direct computation of a
joint-space command output. This is done using the Jacobian matrix relating task-space forces to joint-space torques.
The Jacobian can be passed to `compute_command` directly if it is known. Alternatively, if the controller has been
configured with an associated robot model, the Jacobian can be automatically derived from the current joint positions.

```c++
#include "robot_model/Model.hpp"
#include "state_representation/space/cartesian/JointState.hpp"

using namespace state_representation;

// create a Cartesian impedance controller
auto ctrl = CartesianControllerFactory::create_controller(CONTROLLER_TYPE::IMPEDANCE);

// create a robot model
auto robot = robot_model::Model("my_robot", "/path/to/robot.urdf");

// create some state variables
auto current_joints = JointState::Random(robot.get_robot_name(), robot.get_number_of_joints());
auto command_state = CartesianState::Random("command");
auto feedback_state = CartesianState::Random("feedback");

// compute the command output in joint space using a manually calculated Jacobian
auto jacobian = robot.compute_jacobian(current_joints);
auto joint_command_output_1 = ctrl->compute_command(command_state, feedback_state, jacobian);

// the above is simply equivalent to:
// joint_command = jacobian.transpose() * CartesianWrench(ctrl->compute_command(command_state, feedback_state));

// if controller has a robot model, compute the joint command by passing the current joint positions
ctrl->set_robot_model(robot);
auto joint_command_output_2 = ctrl->compute_command(command_state, feedback_state, current_joints);
```

## Developing a new controller

To implement a new controller, you need to create a class that derives from the `IController` base class or any derived
controller such as `Impedance`. This class can be templated to accept different input spaces (e.g. `CartesianState`
or `JointState`) or specify the desired input space.

The derived controller should override `compute_command` to produce custom behaviour. In addition, if the controller
has any parameters, you should:
- Add parameter pointers as class properties
- Initialize and declare parameters in the constructor
- Override the protected `validate_and_set_parameter` method

```c++
// MyCartesianController.hpp
#include "controllers/IController.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"

class MyCartesianController : public controllers::IController<state_representation::CartesianState> {
public:
  // initialize and declare parameters
  MyCartesianController();
  
  // override this method to implement custom control logic
  state_representation::CartesianState compute_command(
      const state_representation::CartesianState& desired_state,
      const state_representation::CartesianState& feedback_state
  ) override;
  
protected:
  // override this method to update controller configurations when a parameter is modified
  void validate_and_set_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter) override;
  
  // add any additional parameters as class properties
  std::shared_ptr<state_representation::Parameter<int>> foo_;
  std::shared_ptr<state_representation::Parameter<double>> bar_;
};
```

```c++
// MyCartesianController.cpp
#include "MyCartesianController.hpp"

#include "controllers/exceptions/InvalidParameterException.hpp"

using namespace state_representation;

namespace controllers {

// initialize parameters
MyCartesianController::MyCartesianController() :
    foo_(make_shared_parameter<int>(1)),
    bar_(make_shared_parameter<double>(2.0)) {
  // "declare" parameters by inserting them into the parameter list with an associated name
  this->parameters_.insert(std::make_pair("foo", foo_));
  this->parameters_.insert(std::make_pair("bar", bar_));
}

// implement the command logic
CartesianState MyCartesianController::compute_command(
    const CartesianState& desired_state, const CartesianState& feedback_state
) {
    CartesianState command = ...;
    return command;
}

// implement parameter validation
void MyCartesianController::validate_and_set_parameter(
    const std::shared_ptr<state_representation::ParameterInterface>& parameter
) {
  if (parameter->get_name() == "foo") {
    auto value = std::static_pointer_cast<state_representation::Parameter<int>>(parameter);
    // if a parameter value is not supported by the controller, throw an InvalidParameterException
    if (value < 0 || value > 10) {
      throw exceptions::InvalidParameterException("Parameter foo must be in range [0 - 10]");
    }
    this->foo_->set_value(value);
  } else if (parameter->get_name() == "bar") {
    this->bar_->set_value(std::static_pointer_cast<state_representation::Parameter<double>>(parameter));
  }
}

}
```

The main goal of this library is to reuse as much as possible the functionalities between controllers. Therefore, if
your controller is a variation of an impedance controller that simply adds extra way of computing the damping or
stiffness matrix (e.g. `Dissipative`), it should extend from it and reuse the main function calls.
