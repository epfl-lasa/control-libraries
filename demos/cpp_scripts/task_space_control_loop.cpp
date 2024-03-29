#include <chrono>
#include <thread>
#include <dynamical_systems/DynamicalSystemFactory.hpp>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianTwist.hpp>

using namespace dynamical_systems;
using namespace state_representation;
using namespace std::chrono_literals;

CartesianPose control_loop_step(
    const CartesianPose& pose, const std::shared_ptr<IDynamicalSystem<CartesianState>>& ds,
    const std::chrono::nanoseconds& dt
) {
  // get the twist evaluated at current pose
  CartesianTwist desired_twist = ds->evaluate(pose);
  // integrate the twist and add it to the current pose
  CartesianPose new_pose = dt * desired_twist + pose;
  // print the new pose
  std::cout << new_pose << std::endl;
  // return it
  return new_pose;
}

void control_loop(const std::chrono::nanoseconds& dt, double tolerance) {
  // set a desired target and a point attractor ds toward the target
  CartesianPose target = CartesianPose::Random("frame");
  auto ds = CartesianDynamicalSystemFactory::create_dynamical_system(DYNAMICAL_SYSTEM_TYPE::POINT_ATTRACTOR);
  ds->set_parameter(make_shared_parameter("attractor", target));
  // set a starting pose
  CartesianPose current_pose = CartesianPose::Random("frame");
  // loop until target is reached
  double distance;
  do {
    current_pose = control_loop_step(current_pose, ds, dt);
    distance = dist(current_pose, target, CartesianStateVariable::POSE);
    std::cout << "distance to attractor: " << std::to_string(distance) << std::endl;
    std::cout << "-----------" << std::endl;
    std::this_thread::sleep_for(dt);
  } while (distance > tolerance);

  std::cout << "##### TARGET #####" << std::endl;
  std::cout << target << std::endl;
  std::cout << "##### CURRENT POSE #####" << std::endl;
  std::cout << current_pose << std::endl;
}

int main(int, char**) {
  auto dt = 10ms;
  control_loop(dt, 1e-3);
}