#include "state_representation/Robot/JointState.hpp"
#include "state_representation/Robot/JointPositions.hpp"
#include "state_representation/Robot/JointTorques.hpp"
#include <gtest/gtest.h>
#include <fstream>
#include <unistd.h>


TEST(SetPositonsWithModulo, PositiveNos)
{
	Eigen::VectorXd positions(4);
	positions << 1,2,3,4;
	
	StateRepresentation::JointState j1("test_robot", 4);
	j1.set_positions(positions);

	std::cerr << j1 << std::endl;
	
	for(unsigned int i=0; i<j1.get_size(); ++i) EXPECT_TRUE(-M_PI < j1.get_positions()(i) && j1.get_positions()(i) < M_PI);
}

TEST(SetPositonsWithModuloAndNegativeNumbers, PositiveNos)
{
	Eigen::VectorXd positions(4);
	positions << -1,-2,-3,-4;
	
	StateRepresentation::JointState j1("test_robot", 4);
	j1.set_positions(positions);

	std::cerr << j1 << std::endl;

	for(unsigned int i=0; i<j1.get_size(); ++i) EXPECT_TRUE(-M_PI < j1.get_positions()(i) && j1.get_positions()(i) < M_PI);
	EXPECT_TRUE(j1.get_positions()(0) < 0 && j1.get_positions()(3) > 0);
}

TEST(AddTwoState, PositiveNos)
{
	Eigen::VectorXd pos1 = Eigen::VectorXd::Random(4);
	Eigen::VectorXd pos2 = Eigen::VectorXd::Random(4);

	StateRepresentation::JointState j1("test_robot", 4);
	j1.set_positions(pos1);

	StateRepresentation::JointState j2("test_robot", 4);
	j2.set_positions(pos2);

	StateRepresentation::JointState jsum = j1 + j2;
	for(unsigned int i=0; i<j1.get_size(); ++i) EXPECT_TRUE(jsum.get_positions()(i) == atan2(sin(j1.get_positions()(i) + j2.get_positions()(i)), cos(j1.get_positions()(i) + j2.get_positions()(i))));
}

TEST(MultiplyByScalar, PositiveNos)
{
	Eigen::VectorXd pos1 = Eigen::VectorXd::Random(4);

	StateRepresentation::JointState j1("test_robot", 4);
	j1.set_positions(pos1);

	StateRepresentation::JointState jsum = 0.5 * j1;
	std::cerr << jsum << std::endl;
	
	for(unsigned int i=0; i<j1.get_size(); ++i) EXPECT_TRUE(jsum.get_positions()(i) == atan2(sin(0.5 * j1.get_positions()(i)), cos(0.5 * j1.get_positions()(i))));
}

TEST(MultiplyByArray, PositiveNos)
{
	Eigen::VectorXd pos1 = Eigen::VectorXd::Random(4);
	Eigen::MatrixXd gain = Eigen::VectorXd::Random(4).asDiagonal();

	StateRepresentation::JointState j1("test_robot", 4);
	j1.set_positions(pos1);

	StateRepresentation::JointState jsum = gain * j1;
	std::cerr << jsum << std::endl;

	for(unsigned int i=0; i<j1.get_size(); ++i) EXPECT_TRUE(jsum.get_positions()(i) == atan2(sin(gain(i) * j1.get_positions()(i)), cos(gain(i) * j1.get_positions()(i))));
}

TEST(TestVelocitiesOperatorsWithEigen, PositiveNos)
{
	Eigen::Matrix<double, 6, 1> vec = Eigen::Matrix<double, 6, 1>::Random();
	StateRepresentation::JointVelocities velocities("test", 6);
	velocities = vec;
	EXPECT_TRUE((vec-velocities.get_velocities()).norm() < 1e-4);
	velocities = vec.array();
	EXPECT_TRUE((vec-velocities.get_velocities()).norm() < 1e-4);

	velocities += vec;
	EXPECT_TRUE((2 * vec-velocities.get_velocities()).norm() < 1e-4);

	Eigen::Matrix<double, 6, 1> arr = Eigen::Matrix<double, 6, 1>::Random();
	velocities = velocities + arr;

	EXPECT_TRUE(((2 * vec + arr)-velocities.get_velocities()).norm() < 1e-4);

	velocities = arr + velocities;
	EXPECT_TRUE(((2 * vec + 2 * arr)-velocities.get_velocities()).norm() < 1e-4);

	velocities -= arr;
	EXPECT_TRUE(((2 * vec + arr)-velocities.get_velocities()).norm() < 1e-4);

	velocities = velocities - vec;
	EXPECT_TRUE(((vec + arr)-velocities.get_velocities()).norm() < 1e-4);

	velocities = vec - velocities;
	EXPECT_TRUE((arr+velocities.get_velocities()).norm() < 1e-4);
}

TEST(TestTorquesOperatorsWithEigen, PositiveNos)
{
	Eigen::Matrix<double, 6, 1> vec = Eigen::Matrix<double, 6, 1>::Random();
	StateRepresentation::JointTorques torques("test", 6);
	torques = vec;
	EXPECT_TRUE((vec-torques.get_torques()).norm() < 1e-4);
	torques = vec.array();
	EXPECT_TRUE((vec-torques.get_torques()).norm() < 1e-4);

	torques += vec;
	EXPECT_TRUE((2 * vec-torques.get_torques()).norm() < 1e-4);

	Eigen::Matrix<double, 6, 1> arr = Eigen::Matrix<double, 6, 1>::Random();
	torques = torques + arr;

	EXPECT_TRUE(((2 * vec + arr)-torques.get_torques()).norm() < 1e-4);

	torques = arr + torques;
	EXPECT_TRUE(((2 * vec + 2 * arr)-torques.get_torques()).norm() < 1e-4);

	torques -= arr;
	EXPECT_TRUE(((2 * vec + arr)-torques.get_torques()).norm() < 1e-4);

	torques = torques - vec;
	EXPECT_TRUE(((vec + arr)-torques.get_torques()).norm() < 1e-4);

	torques = vec - torques;
	EXPECT_TRUE((arr+torques.get_torques()).norm() < 1e-4);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}