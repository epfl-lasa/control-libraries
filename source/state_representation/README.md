# state_representation

This library provides a set of classes to represent **states** in **cartesian** or **joint** spaces, **parameters**, 
or **geometrical shapes** that can be used as obstacles.
Those are a set of helpers functions to handle common concepts in robotics such as transformations between frames and 
the link between them and the robot state.
This description covers most of the functionalities starting from the spatial transformations.

## Table of contents:
* [Cartesian state](#cartesian-state)
  * [Cartesian state operations](#cartesian-state-operations)
  * [Changing of reference frame](#changing-of-reference-frame)
  * [Specific state variables](#specific-state-variables)
  * [Conversion between Cartesian state variables](#conversion-between-cartesian-state-variables)
* [Joint state](#joint-state)
  * [Joint state operations](#joint-state-operations)
  * [Conversion between joint state variables](#conversion-between-joint-state-variables)
* [The Jacobian matrix](#the-jacobian-matrix)
  * [Conversion between JointVelocities and CartesianTwist](#conversion-between-jointvelocities-and-cartesiantwist)
  * [Conversion between JointTorques and CartesianWrench](#conversion-between-jointtorques-and-cartesianwrench)
  * [Matrix multiplication](#matrix-multiplication)

## Cartesian state

A `CartesianState` represents the transformations between frames in space as well as their dynamic properties
(velocities, accelerations and forces).
It comprises the name of the frame it is associated to and is expressed in a reference frame (by default `world`).
A state contains all the variables that define its dynamic properties, i.e `position`, `orientation`, `linear_velocity`,
`angular_velocity`, `linear_acceleration`, `angular_acceleration`, `force` and `torque`.
All those state variables use `Eigen::Vector3d` internally, except for the orientation that is `Eigen::Quaterniond` based.
All getters and setters are implemented.

```cpp
state_representation::CartesianState s1("a"); // frame a expressed in world (default)
state_representation::CartesianState s2("b", "a"); // frame b expressed in a

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

### Cartesian state operations

Basic operations between frames such as addition, subtractions, scaling are defined, and applied on all the state variables.
It is very important to note that those operations are only valid if both states are expressed in the same reference frame.

```cpp
state_representation::CartesianState s1("a"); // reference frame is world by default
state_representation::CartesianState s2("b");
double lamda = 0.5

// for those operation to be valid both s1 and s2
// should be expressed in the same reference frame
state_representation::CartesianState ssum = s1 + s2;
state_representation::CartesianState sdiff = s1 - s2;
state_representation::CartesianState sscaled = lambda * s1;
```

Also, despite common mathematical interpretation, the `+` operator is **not commutative** due to the orientation part of the state.
The addition `s1 + s2` corresponds to the transformation from the `world` frame to frame `a` followed by the transformation
from `world` frame to frame `b`.
If both frames have the same orientation then the `+` operator is commutative.

```cpp
state_representation::CartesianState s1("a");
state_representation::CartesianState s2("b");
s1.set_orientation(Eigen::Quaterniond(0, 1, 0, 0));
s2.set_orientation(Eigen::Quaterniond::UnitRandom());

state_representation::CartesianState ssum1 = s1 + s2;
state_representation::CartesianState ssum2 = s2 + s1;

ssum1 != ssum2;
```

### Changing of reference frame

One of the most useful operation is the multiplication between two states that corresponds to a changing of reference frame:

```cpp
state_representation::CartesianState wSa("a"); // reference frame is world by default
state_representation::CartesianState aSb("b", "a");

// for this operation to be valid aSb should be expressed in a (wSa)
// the result is b expressed in world
state_representation::CartesianState wSb = wSa * aSb;
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
state_representation::CartesianPose wPa("a");
state_representation::CartesianState aSb("b", "a");

// the result is state b expressed in world
state_representation::CartesianState wSa = wPa + aSb;
```

```cpp
state_representation::CartesianPose wPa("a");
state_representation::CartesianTwist aVb("b", "a");

// the result is twist b expressed in world
state_representation::CartesianTwist wVa = wPa + aVb;
```

### Conversion between Cartesian state variables

The distinction with those specific extra variables allows to define some extra conversion operations.
Therefore, dividing a `CartesianPose` by a time (`std::chrono_literals`) returns a `CartesianTwist`:

```cpp
using namespace std::chrono_literals;
auto period = 1h;

state_representation::CartesianPose wPa("a", Eigen::Vector3d(1, 0, 0));
// the result is a twist of 1m/h in x direction converted in m/s
state_representation::CartesianTwist wVa = wPa / period;
```

Conversely, multiplying a `CartesianTwist` (by default expressed internally in `m/s` and `rad/s`) to a `CartesianPose`
is simply multiplying it by a time period:

```cpp
using namespace std::chrono_literals;
auto period = 10s;

state_representation::CartesianTwist wVa("a", Eigen::Vector3d(1, 0, 0));
state_representation::CartesianPose wPa = period * wVa; // note that wVa * period is also implemented
```

### Cartesian state distance and norms

As a `CartesianState` represents a spatial transformation, distance between states and norms computations have been
implemented. The distance functions is represented as the sum of the distance over all the state variables:

```cpp
using namespace StateRepresentation;
CartesianState cs1 = CartesianState::Random("test");
CartesianState cs2 = CartesianState::Random("test");

double d = cs1.dist(cs2);
// alternatively one can use the friend type notation
d = dist(cs1, cs2)
```

By default, the distance is computed over all the state variables.
One can specify the state variable to consider using the `CartesianStateVariable` enumeration:

```cpp
// only the distance in position
double d_pos = cs1.dist(cs2, CartesianStateVariable::POSITION);
// distance over the wrench
double d_wrench = cs1.dist(cs2, CartesianStateVariable::WRENCH);
```

The norm of a state is using a similar API and returns the norms of each state variable:

```cpp
using namespace StateRepresentation;
CartesianState cs = CartesianState::Random("test");
// default is norm over all the state variables, hence vector is of size 8
std::vector<double> norms = cs.norms();
// for only norm over the pose you need to specify it
std::vector<double> pose_norms = cs.norms(CartesianStateVariable::POSE);
```

`normalize`, inplace normalization, and the copy normalization, `normalized`, are also implemented:

```cpp
using namespace StateRepresentation;
CartesianState cs = CartesianState::Random("test");
// inplace, default is normalization over all the state variables
cs.normalize()
// copied normalized state, with only linear velocity normalized
CartesianState csn = cs.normalized(CartesianStateVariable::LINEAR_VELOCITY)
```

## Joint state

`JointState` follows the same logic as `CartesianState` but for representing robot states.
Similarly to the `CartesianState` the class `JointState`, `JointPositions`, `JointVelocities` and `JointTorques` have been developed.
The API follows exactly the same logic with similar operations implemented.

A `JointState` is defined by the name of the corresponding robot and the name of each joints.

```cpp
// create a state for myrobot with 3 joints
state_representation::JointState js("myrobot", std::vector<string>({"joint0", "joint1", "joint2"}));
```

Note that if the joints of the robot are named `{"joint0", "joint1", ..., "jointN"}` as above,
you can also use the constructor that takes the number of joints as input which will name them accordingly:

```cpp
// create a state for myrobot with 3 joints named {"joint0", "joint1", "joint3"}
state_representation::JointState js("myrobot", 3);
```

All the getters and setters for the `positions`, `velocities`, `accelerations` and `torques` are defined for both
`Eigen::VectorXd` and `std::vector<double>`:

```cpp
js.set_positions(Eigen::Vector3d(.5, 1., 0.));
js.set_positions(std::vector<double>{.5, 1., 0.});
```

Note that when using those setters, the size of the input vector should correspond to the number of joints of the state:

```cpp
js.set_positions(Eigen::Vector4d::Random()); // will throw an IncompatibleSizeException
```

### Joint state operations

Basic operations such as addition, subtraction and scaling have been implemented:

```cpp
state_representation::JointState js1("myrobot", 3);
state_representation::JointState js2("myrobot", 3);
double lambda = 0.5;

// for those operation to be valid both js1 and js2
// should correspond to the same robot and have the
// same number of joints
state_representation::JointState jssum = js1 + js2;
state_representation::JointState jsdiff = js1 - js2;
state_representation::JointState jsscaled = lambda * js1;
```

Multiplication of joint states doesn't have a physical meaning and is, therefore, not implemented.

### Conversion between joint state variables

Similarly to `CartesianState`, the conversion between `JointPositions` and `JointVelocities`
happens through operations with `std::chrono_literals`.

```cpp
using namespace std::chrono_literals;
auto period = 1h;

// create a state for myrobot with 3 joints named {"joint0", "joint1", "joint3"}
// and provide the position values
state_representation::JointPositions jp("myrobot", Eigen::Vector3d(1, 0, 0));

// result are velocities of 1 rad/h for joint0 expressed in rad/s
state_representation::JointVelocities jv = jp / period;
```

```cpp
using namespace std::chrono_literals;
auto period = 10s;

// create a state for myrobot with 3 joints named {"joint0", "joint1", "joint3"}
// and provide the velocities values
state_representation::JointVelocities wVa("a", Eigen::Vector3d(1, 0, 0));

state_representation::JointPositions jp = period * jv; // note that jv * period is also implemented
```

## The Jacobian matrix

The `Jacobian` matrix of a robot ensures the conversion between both `CartesianState` and `JointState`.
Similarly to the `JointState`, a `Jacobian` is associated to a robot and defined by the robot and the number of joints.

```cpp
// create a jacobian for myrobot with 3 joints
state_representation::Jacobian jac("myrobot", std::vector<string>({"joint0", "joint1", "joint2"}));
```

The API is the same as the `JointState`, hence the constructor can also accept the number of joints to initialize the
joint names vector.

```cpp
// create a jacobian for myrobot with 3 joints named {"joint0", "joint1", "joint3"}
state_representation::Jacobian jac("myrobot", 3);
```

The `Jacobian` is simply a `6 x N` matrix where `N` is the number of joints.
Therefore, the data can be set from an `Eigen::MatrixXd` of correct dimensions.

```cpp
jac.set_data(Eigen::MatrixXd::Random(6, 3)); // throw an IncompatibleSizeException if the size is not correct
```

All the functionalities of the `Jacobian` have been implemented such as `transpose`, `inverse` or `pseudoinverse` functions.

```cpp
/// returns the 3 x 6 transposed matrix
state_representation::Jacobian jacT = jac.transpose();
// will throw an error as a 6 x 3 matrix is not invertible
state_representation::Jacobian jacInv = jac.inverse();
// compute the pseudoinverse without the need of being invertible
state_representation::Jacobian jacPinv = jac.pseudoinverse();
```

Those operations are very useful to convert `JointState` from `CartesianState` and vice versa.

### Conversion between JointVelocities and CartesianTwist

The simplest conversion is to transform a `JointVelocities` into a `CartesiantTwist` by multiplication with the `Jacobian`

```cpp
state_representation::JointVelocities jv("myrobot", 3);
state_representation::CartesianTwist eef_twist = jac * js;
```

The opposite transformation, from `CartesianTwist` to `JointVelocities` requires the multiplication with the `inverse`
(or `pseudoinverse`).

```cpp
state_representation::CartesianTwist eef_twist("eef")
state_representation::JointVelocities jv = jac.pseudoinverse() * eef_twist;
```

Note that the `inverse` or `pseudoinverse` functions are computationally expensive and the `solve` function that relies
on the solving of the system `Ax = b` using `Eigen` has been implemented.

```cpp
state_representation::CartesianTwist eef_twist("eef")
// faster than doing jac.pseudoinverse() * eef_twist
state_representation::JointVelocities jv = jac.solve(eef_twist);
```

### Conversion between JointTorques and CartesianWrench

The other conversion that is implemented is the transformation from `CartesianWrench` to `JointTorques`, this one using
the `transpose`.

```cpp
state_representation::CartesianWrench eef_wrench("eef")
// faster than doing jac.pseudoinverse() * eef_twist
state_representation::JointTorques jt = jac.transpose() * eef_wrench;
```

### Matrix multiplication

The `Jacobian` object contains an underlying `Eigen::MatrixXd` which can be retrieved using the `Jacobian::data()` method.
Direct multiplication of the `Jacobian` object with another `Eigen::MatrixXd` has also been implemented and returns the
`Eigen::MatrixXd` product.

```cpp
state_representation::Jacobian jac("myrobot", 3, Eigen::MatrixXd::Random(6, 3));
Eigen::MatrixXd mat = jac.data();
Eigen::MatrixXd res = jac * Eigen::MatrixXd(3, 4); // equivalent to  jac.data() * Eigen::MatrixXd(3, 4);
```
