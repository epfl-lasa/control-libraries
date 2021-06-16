# dynamical_systems

This library provides a set of classes to represent **Dynamical Systems**: 
functions which map a state to a state derivative.
Those are a set of helpers functions to handle common concepts in robotics such as transformations between frames and 
the link between them and the robot state.
This description covers most of the functionalities starting from the spatial transformations.

## Table of contents:
* [Base DynamicalSystem](#base-dynamicalsystem)
  * [Base frame](#base-frame)
  * [Reference frames](#reference-frames)
* [Linear](#linear)
  * [Configuring the Linear DS](#configuring-the-linear-ds)
  * [Evaluating the Linear DS](#evaluating-the-linear-ds)
* [Circular](#circular)
  * [Configuring the Circular DS](#configuring-the-circular-ds)
* [Ring](#ring)
  * [Configuring the Ring DS](#configuring-the-ring-ds)

## Base DynamicalSystem

The base class DynamicalSystem defines the main public interface pattern which the derived classes follow.
 
It is templated for any state type, though the intention is to use the **State Representation** library
(for example, `CartesianState` or `JointState`).

The main function is:
```c++
template<class S>
S DynamicalSystem<S>::evaluate(const S& state) const
```
where `S` is the state representation type.
The purpose of the evaluate function is to calculate a state derivative from a state.
The current implementations support calculating a velocity from a position in either joint space or Cartesian space,
though higher derivatives and other space types can be extended in a similar fashion.

In the case of a `CartesianState`, only the position and orientation is evaluated, and the returned object
updates only the linear and angular velocity properties.
In the case of `JointState`, only the joint position is evaluated, and the returned object contains only the
joint velocity.

### Base frame

The **DynamicalSystem** base class has a private `base_frame` property, which can be thought of as the DS origin. 
The functions `get_base_frame()` and `set_base_frame(const S& state)` can be used to access or modify this base frame.

The DynamicalSystem can be constructed with a `state` to set the base frame, or with string frame name. In 
the latter case the base frame is set as a null / Identity frame with the specified name.
For example, `DynamicalSystem<CartesianState>("base")` will create a dynamical system with the null base frame "base",
expressed in its own frame "base".

In most cases, the constructor for the base DynamicalSystem should not be used directly,
and rather the derived DS classes should construct the base accordingly.

However, the `set_base_frame()` method remains useful in combination with derived classes.

### Reference frames

The following section applies to derived DS classes using the `CartesianState` type. The [Linear DS](#linear) 
will be used as an example.

The `evaluate()` function will always return a twist expressed in the same reference frame as the input state,
provided that the input state is compatible with the DS.

The input state must be expressed in one of two supported reference frames:
1. The reference frame of the input is the DS base frame
2. The reference frame of the input matches the reference frame of the DS base frame

The following snippet illustrates the difference in these two options.
```c++
// create a linear DS with attractor B in frame A
state_representation::CartesianState BinA("B", "A");
dynamical_systems::Linear<state_representation::CartesianState> linearDS(BinA);

linearDS.get_attractor().get_name();             // "B"
linearDS.get_attractor().get_reference_frame();  // "A"
linearDS.get_base_frame().get_name();            // "A"
linearDS.get_base_frame().get_reference_frame(); // "A"

// evaluate a point C in frame A
state_representation::CartesianState CinA("C", "A");
auto twist0 = linearDS.evaluate(CinA); // valid, twist is expressed in frame A

// set the base from of the DS to be A expressed in the world frame
state_representation::CartesianState AinWorld("A", "world");
linearDS.set_base_frame(AinWorld);

linearDS.get_attractor().get_name();             // "B"
linearDS.get_attractor().get_reference_frame();  // "A"
linearDS.get_base_frame().get_name();            // "A"
linearDS.get_base_frame().get_reference_frame(); // "world"

// option 1: reference frame of the input is the same as the base frame 
// -> reference frame of the input: "A"
// -> DS base frame: "A"
// -> reference frame of the output: "A"
auto twist1 = linearDS.evaluate(CinA);

// option 2: reference frame of the input is the same as the base frame 
// -> reference frame of the input: "world"
// -> DS base frame reference frame: "world"
// -> reference frame of the output: "world"
auto CinWorld = AinWorld * CinA;
auto twist2 = linearDS.evaluate(CinWorld);

// as a note, you can mix and match the approach as necessary.
// the following is using option 1 with an additional external operation
// to yield a final result equivalent to option 2.
auto twist3 =  AinWorld * linearDS.evaluate(CinA);
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
state_representation::CartesianState attractor("attractor", "task");
state_representation::CartesianState taskInRobot("task", "robot");

dynamical_systems::Linear<state_representation::CartesianState> linearDS(attractor);

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
  auto twist = linearDS.evaluate(EE);
  
  // send twist command to controller
  update_controller(twist);
}
```

## Linear

The **Linear** DS can be thought of as a point attractor, with a velocity that is linearly proportional
to the distance of the current state from the attractor.
It is currently implemented for the `CartesianState` and `JointState` types.

```c++
// empty construction results in empty base frame and attractor
dynamical_systems::Linear<S> emptyDS;

// construction with an attractor and default value for the gain
state_representation::CartesianState cartesianAttractor("A");
dynamical_systems::Linear<state_representation::CartesianState> linearDS1(cartesianAttractor);

state_representation::JointState jointAttractor("B");
dynamical_systems::Linear<state_representation::JointState> linearDS2(jointAttractor);
```

### Configuring the Linear DS

The Linear DS has the following core parameters:
- **attractor**; the `CartesianState` or `JointState` type object defining the attractor pose relative to the DS base frame
- **gain**; the proportional gain acting towards the attractor

Each parameter has corresponding `set_` and `get_` functions.

To change the strength of the attractor, a gain can be passed as the second argument during construction or
passed to the `set_gain()` member function. The gain defines the proportionality between
a distance unit and a velocity unit, and is internally stored as a square matrix with a size corresponding
to the degrees of freedom in the state representation. For example, the `CartesianState` has six degrees of freedom
(XYZ in linear and angular space), while the `JointState` would have as many degrees of freedom as joints.
The gain can be defined as a matrix directly, as a diagonal vector of the appropriate
length, or as a scalar (which sets the value along the diagonal elements of the matrix).
```c++
// set a gain (scalar, vector or matrix during construction)
double gain = 10;
state_representation::CartesianState csA("A");
dynamical_systems::Linear<state_representation::CartesianState> linear(csA, gain);

// or set / update the gain for the created object
std::vector<double> gains = {1, 2, 3, 4, 5, 6};
linear.set_gain(gains);

// update the attractor
state_representation::CartesianState csB("B");
linear.set_attractor(csB);
```

### Evaluating the Linear DS

To get the velocity from a state, simply call the `evaluate()` function.

```c++
state_representation::CartesianState csA("A"), csB("B");
dynamical_systems::Linear<state_representation::CartesianState> linear(csA);

// note: the return type of evaluate() is a CartesianState, but
// it can be directly assigned to a CartesianTwist because the =operator
// has been defined for that purpose
state_representation::CartesianTwist twist = linear.evaluate(csB);
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

The internal representation of the limit cycle is a `state_representation::Ellipsoid` type. When a state is used
in the constructor, the limit cycle has a constant radius.
If an elliptical limit cycle is desired, the DS can be constructed directly from an `Ellipsoid` type.

```c++
// empty construction results in empty base frame and limit cycle
dynamical_systems::Circular emptyDS;
emptyDS.set_limit_cycle(ellipse); // sets a null base frame according to the reference frame of the limit cycle

// construct the circular DS limit cycle using a CartesianState center
state_representation::CartesianState center("center");

// default constructor (radius = 1)
dynamical_systems::Circular circularDS1(center);

// constructor with radius
double radius = 2.0;
dynamical_systems::Circular circularDS2(center, radius);

// construct the circular DS limit cycle directly using an Ellipsoid type
state_representation::Ellipsoid ellipse("limit_cycle");
ellipse.set_center_state(center);
ellipse.set_axis_lengths({radius, 2 * radius});
dynamical_systems::Circular ellipticalDS(ellipse);
```

### Configuring the Circular DS

The Circular DS has the following core parameters:

- **limit_cycle**; the `Ellipsoid` object defining the limit cycle center, shape and inclination. 
- **planar_gain**; the proportional gain acting in the local plane towards the limit cycle radius.
- **normal_gain**; the proportional gain acting towards the local plane.
- **circular_velocity**; the expected angular orbital velocity around the local origin. 
Setting this value negative reverses the direction of rotation. [rad/s]

Each parameter has corresponding `set_` and `get_` functions.

The constructor takes additional optional arguments to define the gain and circular velocity.
The scalar value for the gain sets both the planar and normal gain of the DS. 

Note that the argument for the radius is only present when a `CartesianState` is provided,
and not when an `Ellipsoid` is provided, as the radii are already parameters of the `Ellipsoid`.
```c++
// construct the circular DS with optional parameters (default values are shown)
double radius = 1.0;
double gain = 1.0;
double circular_velocity = M_PI / 2;

// construction with a CartesianState center
dynamical_systems::Circular circularDS(center, radius, gain, circular_velocity);

// construction with an Ellipsoid ellipse
dynamical_systems::Circular circularDS(ellipse, gain, circular_velocity);
```

In addition, the following helper are defined to configure the limit cycle behaviour.

```c++
.set_center(CartesianState("center"));  // sets the center position and orientation of the limit cycle
.set_gain(1.0);                         // sets both planar and normal gain to the gain value
.set_rotation_angle(0.0);               // sets a rotation offset around the local Z axis
.set_radius(1.0);                       // sets both elliptical axes to the given radius (forces circle)
.set_radiuses({1.0, 2.0});              // sets the length of each elliptical axis individually  
```


## Ring

The **Ring** DS is similar to the **Circular** DS but is more parameterizable.
In brief, the strength of the limit cycle can be configured with a width around the ring radius.

The direction of the ring orbit is a positive rotation around the local Z axis;
this appears as a counter-clockwise rotation when viewed from "above".
A clockwise rotation can be achieved by rotating the ring 180 degrees about its local X or Y axis.

It only supports the `CartesianState` type, and always acts in a circular ring. 

```c++
// empty construction results in empty base frame and center state
dynamical_systems::Ring emptyDS;

// construction with center state and default values for parameters
state_representation::CartesianState center("center");
dynamical_systems::Ring ringDS(center);
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
// construct the ring DS with optional parameters (default values are shown)
double radius = 1.0;
double width = 0.5;
double speed = 1.0;
double field_strength = 1.0;
double normal_gain = 1.0;
double angular_gain = 1.0;
dynamical_systems::Ring ringDS(center, radius, width, speed, field_strength, normal_gain, angular_gain);
```