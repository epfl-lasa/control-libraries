#include "dynamical_systems/Linear.hpp"
#include <vector>
#include <gtest/gtest.h>
#include <unistd.h>


TEST(EvaluateDynamicalSystemPositionOnly, PositiveNos)
{
	StateRepresentation::CartesianPose current_pose("robot", 10 * Eigen::Vector3d::Random());
	StateRepresentation::CartesianPose target_pose("robot", 10 * Eigen::Vector3d::Random());
	DynamicalSystems::Linear<StateRepresentation::CartesianState> linearDS(target_pose);

	unsigned int nb_steps = 100;
	double dt = 0.1;

	for(unsigned int i=0; i<nb_steps; ++i)
	{
		StateRepresentation::CartesianTwist twist = linearDS.evaluate(current_pose);
		current_pose += dt * twist;
	}

	std::cout << current_pose << std::endl;
	std::cout << target_pose << std::endl;
	std::cout << abs(current_pose.get_orientation().dot(target_pose.get_orientation())) << std::endl;

	for(int i=0; i<3; ++i) EXPECT_NEAR(current_pose.get_position()(i), target_pose.get_position()(i), 0.001);
	EXPECT_TRUE(abs(current_pose.get_orientation().dot(target_pose.get_orientation())) > 1-10E-4);	

}

TEST(EvaluateDynamicalSystemOrientationOnly, PositiveNos)
{
	srand (time(NULL));

	StateRepresentation::CartesianPose current_pose("robot", Eigen::Vector3d(0,0,0));
	Eigen::Array4d orientation = Eigen::Array4d::Random();
	StateRepresentation::CartesianPose target_pose("robot", Eigen::Vector3d(0,0,0), Eigen::Quaterniond(orientation(0), orientation(1), orientation(2), orientation(3)));

	DynamicalSystems::Linear<StateRepresentation::CartesianState> linearDS(target_pose);

	unsigned int nb_steps = 100;
	double dt = 0.1;

	for(unsigned int i=0; i<nb_steps; ++i)
	{
		StateRepresentation::CartesianTwist twist = linearDS.evaluate(current_pose);
		current_pose = dt * twist + current_pose;
	}

	std::cout << current_pose << std::endl;
	std::cout << target_pose << std::endl;
	std::cout << abs(current_pose.get_orientation().dot(target_pose.get_orientation())) << std::endl;

	for(int i=0; i<3; ++i) EXPECT_NEAR(current_pose.get_position()(i), target_pose.get_position()(i), 0.01);
	EXPECT_TRUE(abs(current_pose.get_orientation().dot(target_pose.get_orientation())) > 1-10E-4);
}

TEST(EvaluateDynamicalSystem, PositiveNos)
{
	srand (time(NULL));

	StateRepresentation::CartesianPose current_pose("robot", 10 * Eigen::Vector3d::Random());
	Eigen::Array4d orientation = Eigen::Array4d::Random();
	StateRepresentation::CartesianPose target_pose("robot", 10 * Eigen::Vector3d::Random(), Eigen::Quaterniond(orientation(0), orientation(1), orientation(2), orientation(3)));

	DynamicalSystems::Linear<StateRepresentation::CartesianState> linearDS(target_pose);

	unsigned int nb_steps = 500;
	double dt = 0.1;

	for(unsigned int i=0; i<nb_steps; ++i)
	{
		StateRepresentation::CartesianTwist twist = linearDS.evaluate(current_pose);
		current_pose = dt * twist + current_pose;
	}

	std::cout << current_pose << std::endl;
	std::cout << target_pose << std::endl;
	std::cout << abs(current_pose.get_orientation().dot(target_pose.get_orientation())) << std::endl;

	for(int i=0; i<3; ++i) EXPECT_NEAR(current_pose.get_position()(i), target_pose.get_position()(i), 0.01);
	EXPECT_TRUE(abs(current_pose.get_orientation().dot(target_pose.get_orientation())) > 1-10E-4);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}