#include "dynamical_systems/Circular.hpp"
#include <gtest/gtest.h>
#include <unistd.h>


TEST(EvaluateDynamicalSystemPositionOnly, PositiveNos)
{
	StateRepresentation::CartesianPose current_pose("robot", 10 * Eigen::Vector3d::Random());
	StateRepresentation::CartesianPose center = StateRepresentation::CartesianPose::Identity("robot");
	DynamicalSystems::Circular circularDS(center);
	double radius = 10;
	circularDS.set_radius(radius);

	unsigned int nb_steps = 100;
	double dt = 0.1;

	for(unsigned int i=0; i<nb_steps; ++i)
	{
		StateRepresentation::CartesianTwist twist = circularDS.evaluate(current_pose);
		current_pose += dt * twist;
	}

	//for(int i=0; i<3; ++i) ASSERT_NEAR(current_pose.get_position()(i), target_pose.get_position()(i), 0.001);
	//ASSERT_NEAR(current_pose.get_orientation().w(), target_pose.get_orientation().w(), 0.001);
	//for(int i=0; i<3; ++i) ASSERT_NEAR(current_pose.get_orientation().vec()(i), target_pose.get_orientation().vec()(i), 0.001);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}