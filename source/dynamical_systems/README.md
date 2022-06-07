# Dynamical Systems

This library provides a set of classes to represent **Dynamical Systems**: 
functions which map a state to a state derivative.

All controllers have a common interface inheriting from the `IDynamicalSystem<S>` class, which is templated to operate
in a particular space `S`, namely Cartesian or joint space.

## Table of contents:
* [Constructing a dynamical system](#constructing-a-dynamical-system)
* [Using a dynamical system](#using-a-dynamical-system)
  * [Parameters](#parameters)
  * [Evaluate](#evaluate)
* [Developing a new dynamical system](#developing-a-new-dynamical-system)
* [About](#about)
  * [Base frame](#base-frame)
  * [Reference frames](#reference-frames)
* [Point attractor](#point-attractor)
  * [Configuring the Linear DS](#configuring-the-point-attractor-ds)
  * [Evaluating the Linear DS](#evaluating-the-point-attractor-ds)
* [Circular](#circular)
  * [Configuring the Circular DS](#configuring-the-circular-ds)
* [Ring](#ring)
  * [Configuring the Ring DS](#configuring-the-ring-ds)

## Constructing a dynamical system

The `DynamicalSystemFactory<S>` provides construction helpers for dynamical systems using a factory pattern. Specific
dynamical systems are created and injected into a common `shared_ptr<IDynamicalSystem<S>>`.

The `CartesianDynamicalSystemFactory` and `JointDynamicalSystemFactory` are shortcuts for the Cartesian and
joint space dynamical system factories.

The factory provides a static function `create_dynamical_system`, which can take a number of inputs. The first input
argument is always the controller type (defined in `dynamical_systems/DynamicalSystemType.hpp`).

```c++
#include "dynamical_systems/DynamicalSystemFactory.hpp"

using namespace dynamical_systems;
using namespace state_representation;

// create a Cartesian dynamical system
std::shared_ptr<IDynamicalSystem<CartesianState>> cart_ds;
cart_ds = CartesianDynamicalSystemFactory::create_dynamical_system(DYNAMICAL_SYSTEM_TYPE::POINT_ATTRACTOR);

// create a Cartesian dynamical system using "auto" to avoid verbose typing
auto cart_ds = CartesianDynamicalSystemFactory::create_dynamical_system(DYNAMICAL_SYSTEM_TYPE::POINT_ATTRACTOR);
```

Initial dynamical system parameters can be passed to `create_dynamical_system` as a list. See the next section for more
details on parameter configuration.

```c++
#include "dynamical_systems/DynamicalSystemFactory.hpp"
#include "state_representation/parameters/Parameter.hpp"

using namespace dynamical_systems;
using namespace state_representation;

std::list<std::shared_ptr<ParameterInterface>> parameters;
parameters.emplace_back(make_shared_parameter("attractor", CartesianPose::Random("target")));
parameters.emplace_back(make_shared_parameter("gain", 5.0));

auto cart_ds = CartesianDynamicalSystemFactory::create_dynamical_system(DYNAMICAL_SYSTEM_TYPE::POINT_ATTRACTOR, parameters);
```

## Using a dynamical system

The dynamical system factory returns a base class pointer `shared_ptr<IDynamicalSystem<S>>` for the chosen state type
`S`, pointing to an instance of a specific derived dynamical system. This allows all dynamical systems to share the same
consistent interface for usage and configuration.

### Parameters

Dynamical systems support custom parametrization through the base interface using `state_representation::ParameterMap`
methods:

- `get_parameters()`
- `get_parameter(name)`
- `get_parameter_value<T>(name)`
- `set_parameters(parameters)`
- `set_parameter(parameter)`
- `set_parameter_value(name, value)`

These methods can be used after construction to get or set dynamical system parameters. Refer to the documentation
on `dynamical_systems::IDynamicalSystem<S>` and `state_representation::ParameterMap` for more
information.

### Evaluate

The main purpose of each dynamical system is the `evaluate` function, which calculates the state derivative from the
current state. For example, the point attractor dynamical system takes the difference between the current pose and
the desired pose and multiplies it by a linear gain to compute the desired twist.

```c++
#include "dynamical_systems/DynamicalSystemFactory.hpp"
#include "state_representation/space/cartesian/CartesianPose.hpp"

using namespace dynamical_systems;

std::list<std::shared_ptr<ParameterInterface>> parameters;
parameters.emplace_back(make_shared_parameter("attractor", CartesianPose::Random("target")));
parameters.emplace_back(make_shared_parameter("gain", 5.0));

auto cart_ds = CartesianDynamicalSystemFactory::create_dynamical_system(DYNAMICAL_SYSTEM_TYPE::POINT_ATTRACTOR, parameters);
auto current_state = CartesianPose::Random("state");

// compute the twist
auto desired_twist = cart_ds->evaluate(current_state);
```

## Developing a new dynamical system

To implement a new dynamical system, you need to create a class that derives from the `IDynamicalSystem` base class or
any derived dynamical system such as `PointAttractor`. This class can be templated to accept different input spaces
(e.g. `CartesianState` or `JointState`) or specify the desired input state.

The derived dynamical system should override `compute_dynamics` to produce some custom behaviour. in addition, if the dynamical
system has any parameters, you should:
- Add parameter pointers as class properties
- Initialize and declare parameters in the constructor
- Override the `is_compatible` method, if necessary
- Override the protected `validate_and_set_parameter` method

```c++
// MyCartesianDS.hpp
#include "dynamical_systems/IDynamicalSystem.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"

class MyCartesianDS : public dynamical_systems::IDynamicalSystem<state_representation::CartesianState> {
public:
  // initialize and declare parameters
  MyCartesianDS();

protected:
  // override this method to implement custom logic
  state_representation::CartesianState compute_dynamics(
      const state_representation::CartesianState& state
  ) const override;
  
  // override this method to update dynamical system configurations when a parameter is modified
  void validate_and_set_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter) override;
  
  // add any additional parameters as class properties
  std::shared_ptr<state_representation::Parameter<int>> foo_;
  std::shared_ptr<state_representation::Parameter<double>> bar_;
};
```

```c++
// MyCartesianDS.cpp
#include "MyCartesianDS.hpp"

#include "state_representation/exceptions/InvalidParameterException.hpp"

using namespace state_representation;

namespace dynamical_systems {

// initialize parameters
MyCartesianDS::MyCartesianDS() :
    foo_(make_shared_parameter<int>(1)),
    bar_(make_shared_parameter<double>(2.0)) {
  // "declare" parameters by inserting them into the parameter list with an associated name
  this->parameters_.insert(std::make_pair("foo", foo_));
  this->parameters_.insert(std::make_pair("bar", bar_));
}

// implement the command logic
CartesianState MyCartesianDS::compute_dynamics(const CartesianState& state) const {
    CartesianState twist = ...;
    return twist;
}

// implement parameter validation
void MyCartesianController::validate_and_set_parameter(
    const std::shared_ptr<state_representation::ParameterInterface>& parameter
) {
  if (parameter->get_name() == "foo") {
    auto value = std::static_pointer_cast<state_representation::Parameter<int>>(parameter);
    // if a parameter value is not supported by the dynamical system, throw an InvalidParameterException
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

## About

### Base frame

The `IDynamicalSystem` base class has a private `base_frame` property, which can be thought of as the DS origin. 
The functions `get_base_frame()` and `set_base_frame(const S& state)` can be used to access or modify this base frame.

Whilst the term *base frame* makes more sense for a `CartesianState` DS, it rather refers to a specific robot with corresponding 
joint names in the case of a `JointState` DS.

### Reference frames

The following section applies to derived DS classes using the `CartesianState` type. The 
[Point Attractor DS](#point-attractor) will be used as an example.

The `evaluate` function will always return a twist expressed in the same reference frame as the input state,
provided that the input state is compatible with the DS.

The input state must be expressed in one of two supported reference frames:
1. The reference frame of the input is the DS base frame
2. The reference frame of the input matches the reference frame of the DS base frame

The following snippet illustrates the difference in these two options.
```c++
// create a point attractor DS with attractor B in frame A
state_representation::CartesianState BinA = state_representation::CartesianState::Identity("B", "A");
auto ds = CartesianDynamicalSystemFactory::create_dynamical_system(DYNAMICAL_SYSTEM_TYPE::POINT_ATTRACTOR);
ds->set_attractor_value("attractor", BinA);

ds->get_parameter_value("attractor").get_name();             // "B"
ds->get_parameter_value("attractor").get_reference_frame();  // "A"
ds->get_base_frame().get_name();                             // "A"
ds->get_base_frame().get_reference_frame();                  // "A"

// evaluate a point C in frame A
state_representation::CartesianState CinA = state_representation::CartesianState::Random("C", "A");
auto twist0 = ds->evaluate(CinA); // valid, twist is expressed in frame A

// set the base from of the DS to be A expressed in the world frame
state_representation::CartesianState AinWorld = state_representation::CartesianState::Identity("A", "world");
ds->set_base_frame(AinWorld);

ds->get_parameter_value("attractor").get_name();             // "B"
ds->get_parameter_value("attractor").get_reference_frame();  // "A"
ds->get_base_frame().get_name();                             // "A"
ds->get_base_frame().get_reference_frame();                  // "world"

// option 1: reference frame of the input is the same as the base frame 
// -> reference frame of the input: "A"
// -> DS base frame: "A"
// -> reference frame of the output: "A"
auto twist1 = ds->evaluate(CinA);

// option 2: reference frame of the input is the same as the base frame 
// -> reference frame of the input: "world"
// -> DS base frame reference frame: "world"
// -> reference frame of the output: "world"
auto CinWorld = AinWorld * CinA;
auto twist2 = ds->evaluate(CinWorld);

// as a note, you can mix and match the approach as necessary.
// the following is using option 1 with an additional external operation
// to yield a final result equivalent to option 2.
auto twist3 =  AinWorld * ds->evaluate(CinA);
// twist2 === twist3
```

Note that the base frame can have its own velocity or other state properties, which
are automatically combined with the DS result with respect to the common reference frame.

Setting the base frame of the DS has some benefits. In some cases, the state variable to be
evaluated is not directly expressed in the frame of the DS.
Similarly, the output twist may need to be expressed in a different reference frame.

As a practical example, consider a case where the state of an end-effector is
reported in the reference frame of a robot, while a linear attractor is expressed
in some moving task frame.
The robot controller expects a twist expressed in the robot frame.
By updating the DS base frame with respect to the robot frame,
any pre-transformation of the end-effector state or post-transformation of the twist can be avoided.

```c++
state_representation::CartesianState EE("end_effector", "robot");
state_representation::CartesianState attractor = state_representation::CartesianState::Random("attractor", "task");
state_representation::CartesianState taskInRobot("task", "robot");

auto ds = CartesianDynamicalSystemFactory::create_dynamical_system(DYNAMICAL_SYSTEM_TYPE::POINT_ATTRACTOR);
ds->set_attractor_value("attractor", attractor);

// control loop
while (...) {
  // update the EE state from robot feedback 
  EE.set_pose(...);

  // update the state of the task with respect to the robot (for example, from optitrack)
  taskInRobot.set_pose(...);
  taskInRobot.set_linear_velocity(...);
  taskInRobot.set_angular_velocity(...);
  
  // now update the DS base frame
  linearDS.set_base_frame(taskInRobot);
  
  // find the twist in the robot reference frame 
  // directly from the end-effector position in the robot reference frame
  auto twist = ds->evaluate(EE);
  
  // send twist command to controller
  update_controller(twist);
}
```

## Point Attractor

The **Point Attractor** DS generates a velocity that is linearly proportional to the distance of the current state 
from the attractor. It is currently implemented for the `CartesianState` and `JointState` types.

```c++
// construction with the DS factory
auto ds = CartesianDynamicalSystemFactory::create_dynamical_system(DYNAMICAL_SYSTEM_TYPE::POINT_ATTRACTOR);
```

### Configuring the Point Attractor DS

The Point Attractor DS has the following core parameters:
- **attractor**; the `CartesianState` or `JointState` type object defining the attractor pose relative to the DS base frame
- **gain**; the proportional gain acting towards the attractor

The gain defines the proportionality between a distance unit and a velocity unit, and is internally stored as a 
square matrix with a size corresponding to the degrees of freedom in the state representation. For example, 
the `CartesianState` has six degrees of freedom (XYZ in linear and angular space), while the `JointState` would
have as many degrees of freedom as joints.
The gain can be defined as a matrix directly, as a diagonal vector of the appropriate
length, or as a scalar (which sets the value along the diagonal elements of the matrix).
```c++
// set a gain (scalar, vector or matrix)
double gain = 10;
ds->set_parameter_value("gain", gain);
// or
std::vector<double> gains = {1, 2, 3, 4, 5, 6};
ds->set_parameter_value("gain", gain);

// update the attractor
state_representation::CartesianState csB = state_representation::CartesianState::Random("B");
ds->set_parameter_value("attractor", csB);
```

### Evaluating the Point Attractor DS

To get the velocity from a state, simply call the `evaluate()` function.

```c++
auto ds = CartesianDynamicalSystemFactory::create_dynamical_system(DYNAMICAL_SYSTEM_TYPE::POINT_ATTRACTOR);
state_representation::CartesianState csA = state_representation::CartesianState::Identity("A");
ds->set_parameter_value("attractor", csA);

state_representation::CartesianState csB = state_representation::CartesianState::Random("B");
// note: the return type of evaluate() is a CartesianState, but
// it can be directly assigned to a CartesianTwist because the =operator
// has been defined for that purpose
state_representation::CartesianTwist twist = ds->evaluate(csB);
```

The returned velocity will always be expressed in the same reference frame as the input state.

## Circular

The **Circular** DS is a limit cycle that rotates around a center point in an elliptical orbit,
converging to a desired radius on a plane.

The direction of the rotation is positive around the local Z axis;
this appears as a counter-clockwise rotation when viewed from "above".
 
This DS is defined only for the `CartesianState` type. In addition, it only acts in linear space,
determining a linear velocity for a given position. It does not produce any angular velocity.

The Circular DS can be constructed with a `CartesianState` state and radius as an argument;
the state position defines the center of the limit cycle, while the state orientation defines the inclination
of the limit cycle plane. The radius has a default value of 1.

The internal representation of the limit cycle is a `state_representation::Ellipsoid` type. 

```c++
// construction with the DS factory
auto ds = CartesianDynamicalSystemFactory::create_dynamical_system(DYNAMICAL_SYSTEM_TYPE::CIRCULAR);
```

### Configuring the Circular DS

The Circular DS has the following core parameters:

- **limit_cycle**; the `Ellipsoid` object defining the limit cycle center, shape and inclination. 
- **planar_gain**; the proportional gain acting in the local plane towards the limit cycle radius.
- **normal_gain**; the proportional gain acting towards the local plane.
- **circular_velocity**; the expected angular orbital velocity around the local origin. 
Setting this value negative reverses the direction of rotation. [rad/s]

```c++
// update the parameters of the circular DS
state_representation::CartesianState center = state_representation::CartesianState::Identity("center");
double radius = 2.0;
state_representation::Ellipsoid ellipse("limit_cycle");
ellipse.set_center_state(center);
ellipse.set_axis_lengths({radius, 2 * radius});
ds->set_parameter_value("limit_cycle", ellipse);

double planar_gain = 1.0;
ds->set_parameter_value("planar_gain", planar_gain);

double circular_velocity = M_PI / 2;
ds->set_parameter_value("circular_velocity", circular_velocity);
```

## Ring

The **Ring** DS is similar to the **Circular** DS but is more parameterizable.
In brief, the strength of the limit cycle can be configured with a width around the ring radius.

The direction of the ring orbit is a positive rotation around the local Z axis;
this appears as a counter-clockwise rotation when viewed from "above".
A clockwise rotation can be achieved by rotating the ring 180 degrees about its local X or Y axis.

It only supports the `CartesianState` type, and always acts in a circular ring. 

```c++
// construction with the DS factory
auto ds = CartesianDynamicalSystemFactory::create_dynamical_system(DYNAMICAL_SYSTEM_TYPE::RING);
```

### Configuring the Ring DS

The Ring DS has the following core parameters:

- **center**; the ring center `CartesianPose` expressed in the DS base reference frame.
This sets both the origin center and the inclination of the ring plane.
- **rotation_offset**; the orientation offset of the orientational attractor in the ring frame. 
- **radius**; the ring radius. [m]
- **width**; the distance from the radius where the velocity has tangential components around the ring.
Beyond this width, the velocity is always perpendicular towards the radius. [m]
- **speed**; the desired linear speed when travelling along the circle radius.
The limit cycle is only stable when the speed is positive. [m/s]
- **field_strength**; the scale factor applied to the ring speed outside of the radius + width zone.
- **normal_gain**; the scale factor for the speed normal to the ring plane.
- **angular_gain**; the scale factor for angular error restitution.

Each parameter has corresponding `set_` and `get_` functions.

The constructor takes additional optional arguments to define the ring DS parameters.
```c++
// update the ring DS parameters
state_representation::CartesianState center = state_representation::CartesianState::Identity("center");
ds->set_parameter_value("center", center);
double radius = 1.0;
ds->set_parameter_value("radius", radius);
double width = 0.5;
ds->set_parameter_value("width", width);
double speed = 1.0;
ds->set_parameter_value("speed", speed);
double field_strength = 1.0;
ds->set_parameter_value("field_strength", field_strength);
double normal_gain = 1.0;
ds->set_parameter_value("normal_gain", normal_gain);
double angular_gain = 1.0;
ds->set_parameter_value("angular_gain", angular_gain);
```