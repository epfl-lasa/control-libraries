# controllers library

This library introduces a set of controllers to be used in robotic control-loop schemes. Each controller are templated
to accepted an input space from `StateRepresentation` and is derived from the `Controller` base class, e.g.,

```cpp
Impedance<StateRepresentation::JointState> joint_space_impedance;
Disspative<StateRepresentation::CartesianState> task_space_dissipative;
```

The main functions of each controllers are the `compute_command` variations. They implement the main algorithms of the
controllers. Depending on the controller, they are used to define a command from some input parameters.

```cpp
using namespace StateRepresentation;

Impedance<JointState> joint_space_impedance;
JointVelocities desired_velocities;
JointVelocities real_robot_velocities;

JointTorques command = joint_space_impedance.compute_command(desired_velocities,
                                                             real_robot_velocities)
```

Some `compute_command` variations allows to transform one space to the other using extra parameters.

```cpp
using namespace StateRepresentation;

Dissipative<CartesianState> task_space_dissipative;
CartesianTwist desired_twist;
CartesianTwist real_eef_twist;
Jacobian robot_jacobian;

JointTorques command = task_space_dissipative.compute_command(desired_twist,
                                                              real_eef_twist,
                                                              jacobian)
```

The above formulation is equivalent to:

```cpp
using namespace StateRepresentation;

Dissipative<CartesianState> task_space_dissipative;
CartesianTwist desired_twist;
CartesianTwist real_eef_twist;
Jacobian robot_jacobian;

CartesianWrench task_command = task_space_dissipative.compute_command(desired_twist,
                                                                      real_eef_twist)

JointTorques joint_command = robot_jacobian.transpose() * task_command;
```

## Creating a new controller

To create a new controller, you need to create a class that derives from the `Controller` base class or any derived
controller such as `Impedance`. This class can be template to accept different input spaces (e.g. `CartesianState`
or `JointState`) or specify the desired input space.

```cpp
using namespace StateRepresentation;

class MyController:
public Controller<CartesianState> {
private:
  ...
public:
  CartesianState compute_command(const CartesianState& desired_state,
                                 const CartesianState& feedback_state);
};
```

The only thing to implement is then the desired variation of the `compute_command` functions.

```cpp
CartesianState MyController::compute_command(const CartesianState& desired_state,
                                             const CartesianState& feedback_state) {
  CartesianState command = ...;
  return command;
}
```

The main goal of this library is to reuse as much as possible the functionalities between controllers. Therefore, if
your controller is a variation of an impedance controller that simply adds extra way of computing the damping or
stiffness matrix (e.g. `Dissipative`), it should extend from it and reuse the main function calls.

```cpp
using namespace StateRepresentation;

class MyController:
public Controller<CartesianState> {
private:
  ...
public:
  CartesianState compute_command(const CartesianState& desired_state,
                                 const CartesianState& feedback_state);
};

CartesianState MyImpedanceController::compute_command(const CartesianState& desired_state,
                                                      const CartesianState& feedback_state) {
  // do extra stuff like compute the damping or stiffness matrix
  ...
  CartesianWrench command = this->Impedance<CartesianState>(desired_state, feedback_state);
  // eventually do extra stuff before returning the command
  ...
  return command;
}
```

With this implementation, you don't need to specify anything specific for using the space changing functionalities, it will be derived from the inheritance chain.

```cpp
CartesianState MyImpedanceController::compute_command(const CartesianState& desired_state,
                                                      const CartesianState& feedback_state,
                                                      const Jacobian& jacobian) {
  // simply call the compute_command from impedance
  return this->Impedance<CartesianState>(desire_state, feedback_state, jacobian);
}
```

