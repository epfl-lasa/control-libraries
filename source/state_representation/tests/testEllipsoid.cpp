#include "state_representation/Geometry/Ellipsoid.hpp"
#include <gtest/gtest.h>


using namespace state_representation;

TEST(Sampling, PositiveNos)
{
	Ellipsoid ellipse("test");
	
	// simplest case circle centered of radius 1
	auto points = ellipse.sample_from_parameterization(100);
	for (const auto& p : points)
	{
		double x = p.get_position()(0);
		double y = p.get_position()(1);
		double d = x*x + y*y - 1;
		EXPECT_TRUE(abs(d) < 1e-3);
	}

	// change the axis lenghts
	ellipse.set_axis_lengths({3., 0.5, 0.});
	points = ellipse.sample_from_parameterization(100);
	for (const auto& p : points)
	{
		double x = p.get_position()(0);
		double y = p.get_position()(1);
		double d = x*x / 9 + y*y / 0.25 - 1;
		EXPECT_TRUE(abs(d) < 1e-3); 
	}
}

TEST(EllipsoidFitting, PositiveNos)
{
	Ellipsoid ellipse("test");

	// sample from the parameterization
	auto points = ellipse.sample_from_parameterization(100);

	// try to fit a new ellipse on the points
	Ellipsoid fitted_ellipse = Ellipsoid::fit("fitted_test", points);

	// equality test
	double epsilon = 0.2;
	EXPECT_TRUE(abs(ellipse.get_center_position()(0) - fitted_ellipse.get_center_position()(0)) < epsilon);
	EXPECT_TRUE(abs(ellipse.get_center_position()(1) - fitted_ellipse.get_center_position()(1)) < epsilon);
	EXPECT_TRUE(abs(ellipse.get_axis_length(0) - fitted_ellipse.get_axis_length(0)) < epsilon);
	EXPECT_TRUE(abs(ellipse.get_axis_length(1) - fitted_ellipse.get_axis_length(1)) < epsilon);

	// more complex
	ellipse.set_center_position(Eigen::Vector3d(-1, 2.5, 0));
	ellipse.set_center_orientation(Eigen::Quaterniond(Eigen::AngleAxisd(0.56, Eigen::Vector3d::UnitZ())));
	ellipse.set_axis_lengths({3, 1});

	points = ellipse.sample_from_parameterization(100);
	fitted_ellipse = Ellipsoid::fit("fitted_test", points);

	// equality test
	EXPECT_TRUE(abs(ellipse.get_center_position()(0) - fitted_ellipse.get_center_position()(0)) < epsilon);
	EXPECT_TRUE(abs(ellipse.get_center_position()(1) - fitted_ellipse.get_center_position()(1)) < epsilon);
	EXPECT_TRUE(abs(ellipse.get_axis_length(0) - fitted_ellipse.get_axis_length(0)) < epsilon);
	EXPECT_TRUE(abs(ellipse.get_axis_length(1) - fitted_ellipse.get_axis_length(1)) < epsilon);

	// random parameters
	for (unsigned int i=0; i<100; ++i)
	{
		Eigen::Vector3d position = Eigen::Vector3d::Random();
		position(2) = 0.;
		ellipse.set_center_position(position);
		ellipse.set_center_orientation(Eigen::Quaterniond(Eigen::AngleAxisd((rand() % 10)/10., Eigen::Vector3d::UnitZ())));

		double r1 = (rand() % 100)/10.;
		double r2 = r1 / (rand() % 10 + 1);
		ellipse.set_axis_lengths({r1, r2});

		points = ellipse.sample_from_parameterization(100);
		fitted_ellipse = Ellipsoid::fit("fitted_test", points);

		// equality test
		EXPECT_TRUE(abs(ellipse.get_center_position()(0) - fitted_ellipse.get_center_position()(0)) < epsilon);
		EXPECT_TRUE(abs(ellipse.get_center_position()(1) - fitted_ellipse.get_center_position()(1)) < epsilon);
		EXPECT_TRUE(abs(ellipse.get_axis_length(0) - fitted_ellipse.get_axis_length(0)) < epsilon);
		EXPECT_TRUE(abs(ellipse.get_axis_length(1) - fitted_ellipse.get_axis_length(1)) < epsilon);
	}
}

int main(int argc, char **argv) 
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
