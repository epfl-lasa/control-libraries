# state_representation

This library provides a set of classes to represent states in cartesian, joint and dual quaternion spaces. A state is made of the name of the frame it is associated to and it is expressed in a reference frame (by default `world`). Basic operations such as addition, multiplication, scaling, ... are defined (if they have a physical meaning).

```cpp
StateRepresentation::CartesianPose p1("a"); // reference frame is world by default
StateRepresentation::CartesianPose p2("a");

// for this operation to be valid both p1 and p2 should be associated to the same frame (here a) and expressed in the same reference frame
StateRepresentation::CartesianPose psum = p1 + p2;
```

States in each spaces can represent pose (position and orientation), twist (linear velocity and angular velocity), acceleration (linear and angular) and wrench (force and torque). Operations representing transformations are implemented. For example the multiplication between two poses allows to express the resulting pose in a different reference frame.

```cpp
StateRepresentation::CartesianPose p1("a");
StateRepresentation::CartesianPose p2("b", "a");

// for this operation to be valid p2 reference frame should be equal to p1 name. The resultant will be "b" expressed in world
StateRepresentation::CartesianPose pres = p1 * p2;

std::cout << pres.get_name() << std::endl;
$ b

std::cout << pres.get_reference_frame() << std::endl;
$ world
```

Joint space also includes additional verification such as joint ordering and names to ensure valid operations.

### Compile cpp library (for testing)
```bash
cd ~/modulo_lib/state_representation/
mkdir build && cd build
cmake -Druntests=ON
make
sudo make install
```

### Run Test Library (CPP)
Run extensive tests
```bash
ctest --verbose
```

Run specific test (here Cartesian).
List other executables (in the build folder)
```bash
./runTestCartesianState
```
