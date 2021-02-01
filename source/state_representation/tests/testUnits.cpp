#include <gtest/gtest.h>
#include <fstream>
#include "state_representation/Units/Distance.hpp"
#include "state_representation/Units/Angle.hpp"
#include "state_representation/Units/Velocity.hpp"

using namespace StateRepresentation::Units;
using namespace StateRepresentation::Units::literals;

TEST(CreateDistances, PositiveNos)
{
	Distance d1 = 1.0_dm;
	Distance d2 = 0.1_m;
	
	EXPECT_TRUE(d1 == d2);
	EXPECT_FALSE(d1 != d2);

	Distance d3 = 500.0_cm;
	EXPECT_TRUE(d1 < d3);
	EXPECT_TRUE(d3 > d1);

	Distance d4 = 5000.01_mm;
	EXPECT_FALSE(d3 < d4);
	EXPECT_TRUE(d3 <= d4);
}

TEST(DistanceOperations, PositiveNos)
{
	Distance d1 = 1.0_dm;
	Distance d2 = 0.1_m;

	Distance d3 = 0.2_m;
	Distance d4 = 0.0_m;

	Distance d5 = 0.4_m;

	EXPECT_TRUE((d1 + d2) == d3);
	EXPECT_TRUE((d1 - d2) == d4);

	EXPECT_TRUE(2*(d1 + d2) == d5);
	EXPECT_TRUE(abs(d5 / d3 - 2.0) < 1e-4);
}

TEST(CreateLinearVelocity, PositiveNos)
{
	LinearVelocity v1 = 1.0_m_s;
	EXPECT_TRUE(abs(v1.get_value() - 1.0) < 1e-4);
}

TEST(CreateAngles, PositiveNos)
{
	Angle a1 = M_PI;
	Angle a2 = 180.0_deg;
	EXPECT_TRUE(a1 == a2);

	Angle a3 = -M_PI / 2;
	Angle a4 = -90.0_deg;
	EXPECT_TRUE(a3 == a4);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}