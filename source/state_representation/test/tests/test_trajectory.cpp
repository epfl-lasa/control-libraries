#include <gtest/gtest.h>
#include <fstream>
#include <unistd.h>
#include "state_representation/trajectories/Trajectory.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/joint/JointState.hpp"

TEST(TrajectoryTest, CreateTrajectory) {
  state_representation::Trajectory<state_representation::JointState> trajectory;
  std::deque<state_representation::JointState> points = trajectory.get_points();
  std::deque<std::chrono::nanoseconds> times = trajectory.get_times();
  EXPECT_TRUE(points.empty());
  EXPECT_TRUE(times.empty());
}

TEST(TrajectoryTest, AddPoint) {
  state_representation::Trajectory<state_representation::JointState> trajectory;
  state_representation::JointState point("robot", 1);

  std::deque<state_representation::JointState> points = trajectory.get_points();
  std::deque<std::chrono::nanoseconds> times = trajectory.get_times();

  unsigned int prev_size_points = points.size();
  unsigned int prev_size_times = times.size();

  std::chrono::nanoseconds period(100);
  Eigen::ArrayXd positions(1);
  positions << 0.2;
  point.set_positions(positions);
  trajectory.add_point(point, period);

  points = trajectory.get_points();
  times = trajectory.get_times();

  unsigned int new_size_points = points.size();
  unsigned int new_size_times = times.size();

  EXPECT_TRUE(new_size_points == prev_size_points + 1);
  EXPECT_TRUE(new_size_times == prev_size_times + 1);
}

TEST(TrajectoryTest, ClearPoint) {
  state_representation::Trajectory<state_representation::JointState> trajectory;
  state_representation::JointState point("robot", 1);

  std::chrono::nanoseconds period(100);
  Eigen::ArrayXd positions(1);
  positions << 0.2;
  point.set_positions(positions);
  trajectory.add_point(point, period);

  std::deque<state_representation::JointState> points = trajectory.get_points();
  std::deque<std::chrono::nanoseconds> times = trajectory.get_times();

  unsigned int size_points = points.size();
  unsigned int size_times = times.size();

  EXPECT_TRUE(size_points == 1);
  EXPECT_TRUE(size_times == 1);

  trajectory.clear();
  points = trajectory.get_points();
  times = trajectory.get_times();

  EXPECT_TRUE(points.empty());
  EXPECT_TRUE(times.empty());
}

TEST(TrajectoryTest, OverloadIndex) {
  state_representation::Trajectory<state_representation::JointState> trajectory;
  state_representation::JointState point("robot", 1);

  std::chrono::nanoseconds period(100);
  Eigen::ArrayXd positions(1);
  positions << 0.2;
  point.set_positions(positions);
  trajectory.add_point(point, period);
  positions << 0.7;
  point.set_positions(positions);
  trajectory.add_point(point, period);

  std::pair<state_representation::JointState, std::chrono::nanoseconds> point0 = trajectory[0];
  std::pair<state_representation::JointState, std::chrono::nanoseconds> point1 = trajectory[1];

  EXPECT_TRUE(point0.first.get_positions()[0] == 0.2);
  EXPECT_TRUE(point1.first.get_positions()[0] == 0.7);
  EXPECT_TRUE(point0.second == period);
  EXPECT_TRUE(point1.second == 2 * period);
}

// TEST(TrajectoryTest, InsertPoint)
// {
// 	state_representation::Trajectory<state_representation::JointState> trajectory;
// 	state_representation::JointState point("robot", 1);

// 	std::chrono::nanoseconds period(100);
// 	Eigen::ArrayXd positions(1);
// 	positions << 0.2;
// 	point.set_positions(positions);
// 	trajectory.add_point(point, period);
// 	positions << 0.7;
// 	point.set_positions(positions);
// 	trajectory.add_point(point, period);

// 	std::pair<state_representation::JointState, std::chrono::nanoseconds> last_point = trajectory[1];
// 	std::deque<state_representation::JointState> points = trajectory.get_points();
// 	std::deque<std::chrono::nanoseconds> times = trajectory.get_times();

// 	EXPECT_TRUE(points.size() == 2);
// 	EXPECT_TRUE(times.size() == 2);
// 	EXPECT_TRUE(last_point.first.get_positions()[0] == 0.7);
// 	EXPECT_TRUE(last_point.second == 2*period);

// 	positions << 0.8;
// 	point.set_positions(positions);
// 	trajectory.insert_point(point, period, 1);

// 	std::pair<state_representation::JointState, std::chrono::nanoseconds> inserted_point = trajectory[1];
// 	last_point = trajectory[2];
// 	points = trajectory.get_points();
// 	times = trajectory.get_times();

// 	EXPECT_TRUE(points.size() == 3);
// 	EXPECT_TRUE(times.size() == 3);
// 	EXPECT_TRUE(inserted_point.first.get_positions()[0] == 0.8);
// 	EXPECT_TRUE(inserted_point.second == 2*period);
// 	EXPECT_TRUE(last_point.first.get_positions()[0] == 0.7);
// 	EXPECT_TRUE(last_point.second == 3*period);
// }
