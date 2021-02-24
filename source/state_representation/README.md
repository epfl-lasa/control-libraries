# state_representation

This library provides a set of classes to represent **states** in **cartesian** or **joint** spaces, **parameters**, or **geometrical shapes** that can be used as obstacles.
Those are a set of helpers functions to handle common concepts in robotics such as transformations between frames and the link between them and the robot state.
This description covers most of the functionalities starting from the spatial transformations.

## Cartesian state

A `CartesianState` represents the transformations between frames in space as well as their dynamic properties (velocities, accelerations and forces).
It comprises the name of the frame it is associated to and is expressed in a reference frame (by default `world`).
A state contains all the variables that define its dynamic properties, i.e `position`, `orientation`, `linear_velocity`, `angular_velocity`, `linear_acceleration`, `angular_acceleration`, `force` and `torque`.
All those state variables use `Eigen::Vector3d` internally, except for the orientation that is `Eigen::Quaterniond` based.
All getters and setters are implemented.

```cpp
StateRepresentation::CartesianState s1("a"); // frame a expressed in world (default)
StateRepresentation::CartesianState s2("b", "a"); // frame b expressed in a

s1.set_position(Eigen::Vector3d(0, 1, 0)); // 1 meter in y direction
s1.set_orientation(Eigen::Quaterniond(0, 1, 0, 0));
```

By default, quaternions are normalized on setting, therefore:

```cpp
// will be rendered as Eigen::Quaterniond(0.70710678, 0.70710678, 0. , 0.)
s2.set_orientation(Eigen::Quaterniond(1, 1, 0, 0));
```

The state variables are also grouped in `pose` (`position` and `orientation`), `twist` (`linear_velocity` and
`angular_velocity`), `accelerations` (`linear_acceleration` and `angular_acceleration`) and `wrench` (`force` and `torque`).
Getter and setters are also implemented to do those bulk operations.

```cpp
s2.set_twist(Eigen::VectorXd::Random(6));
```

Note that for `pose`, it will be a `7d` vector (3 for `position` and 4 for `orientation`):

```cpp
s2.set_pose(Eigen::VectorXd::Random(7));
```

### State operations

Basic operations between frames such as addition, subtractions, scaling are defined, and applied on all the state variables.
It is very important to note that those operations are only valid if both states are expressed in the same reference frame.

```cpp
StateRepresentation::CartesianState s1("a"); // reference frame is world by default
StateRepresentation::CartesianState s2("b");
double lamda = 0.5

// for those operation to be valid both s1 and s2
// should be expressed in the same reference frame
StateRepresentation::CartesianState ssum = s1 + s2;
StateRepresentation::CartesianState sdiff = s1 - s2;
StateRepresentation::CartesianState sscaled = lambda * s1;
```

Also, despite common mathematical interpretation, the `+` operator is **not commutative** due to the orientation part of the state.
The addition `s1 + s2` corresponds to the transformation from the `world` frame to frame `a` followed by the transformation
from `world` frame to frame `b`.
If both frames have the same orientation then the `+` operator is commutative.

```cpp
StateRepresentation::CartesianState s1("a");
StateRepresentation::CartesianState s2("b");
s1.set_orientation(Eigen::Quaterniond(0,1,0,0));
s2.set_orientation(Eigen::Quaterniond::UnitRandom());

StateRepresentation::CartesianState ssum1 = s1 + s2;
StateRepresentation::CartesianState ssum2 = s2 + s1;

ssum1 != ssum2;
```

### Changing of reference frame

One of the most useful operation is the multiplication between two states that corresponds to a changing of reference frame:

```cpp
StateRepresentation::CartesianState wSa("a"); // reference frame is world by default
StateRepresentation::CartesianState aSb("b", "a");

// for this operation to be valid aSb should be expressed in a (wSa)
// the result is b expressed in world
StateRepresentation::CartesianState wSb = wSa * aSb;
```

Not only does that apply a changing of reference frame but it also express all the state variables of `aSb`
in the desired reference frame (here `world`), taking into account the dynamic of the frame `wSa`,
i.e. if `wSa` has a `twist` or `acceleration` it will affect the state variables of `wSb`.

### Specific state variables

Full `CartesianState` can be difficult to handle as they contain all the dynamics of the frame when, sometime,
you just want to express a `pose` without `twist`, `accelerations` or `wrench`.
Therefore, extra classes representing only those specific state variables have been defined, `CartesianPose`,
`CartesianTwist` and `CartesianWrench`.
Effectively, they all extend from `CartesianState` hence they can be intertwined as will.

```cpp
StateRepresentation::CartesianPose wPa("a");
StateRepresentation::CartesianState aSb("b", "a");

// the result is state b expressed in world
StateRepresentation::CartesianState wSa = wPa + aSb;
```

```cpp
StateRepresentation::CartesianPose wPa("a");
StateRepresentation::CartesianTwist aVb("b", "a");

// the result is twist b expressed in world
StateRepresentation::CartesianTwist wVa = wPa + aVb;
```

### Conversion between state variables

The distinction with those specific extra variables allows to define some extra conversion operations.
Therefore, dividing a `CartesianPose` by a time (`std::chrono_literals`) returns a `CartesianTwist`:

```cpp
using namespace std::chrono_literals;
auto period = 1h;

StateRepresentation::CartesianPose wPa("a", Eigen::Vector3d(1,0,0));
// the result is a twist of 1m/h in x direction converted in m/s
StateRepresentation::CartesianTwist wVa = wPa / period;
```

Conversely, multiplying a `CartesianTwist` (by default expressed internally in `m/s` and `rad/s`) to a `CartesianPose` is simply multiplying it by a time period:

```cpp
using namespace std::chrono_literals;
auto period = 10s;

StateRepresentation::CartesianTwist wVa("a", Eigen::Vector3d(1,0,0));
StateRepresentation::CartesianPose wPa = period * wVa; // note that wVa * period is also implemented
```

## Joint state

TODO