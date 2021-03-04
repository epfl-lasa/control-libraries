#include "state_representation/Parameters/Event.hpp"
#include "state_representation/Parameters/Parameter.hpp"
#include "state_representation/Space/Cartesian/CartesianPose.hpp"
#include "state_representation/Space/Cartesian/CartesianState.hpp"
#include <gtest/gtest.h>

TEST(Conversion, PositiveNos) {
  using namespace state_representation;
  Parameter<CartesianPose> test1("test", CartesianPose::Random("test"));
  Parameter<CartesianState> test2(test1);
  EXPECT_EQ(test2.get_type(), StateType::PARAMETER_CARTESIANPOSE);

  std::shared_ptr<Parameter<CartesianState>> test3 = std::make_shared<Parameter<CartesianState>>(Parameter<CartesianPose>("test", CartesianPose::Random("test")));
  EXPECT_EQ(test3->get_type(), StateType::PARAMETER_CARTESIANPOSE);
}

TEST(Event, PositiveNos) {
  using namespace state_representation;
  Event e("test");
  EXPECT_FALSE(e.get_value());
  EXPECT_FALSE(e.get_previous_value());

  e.set_value(true);

  EXPECT_TRUE(e.get_value());
  EXPECT_TRUE(e.get_previous_value());

  EXPECT_TRUE(e.read_value());// reading once will prevent from reading true again
  EXPECT_FALSE(e.get_value());
  EXPECT_FALSE(e.read_value());

  e.set_value(true);// will not produce true as it was already true before
  EXPECT_FALSE(e.get_value());

  e.set_value(false);// stay false
  EXPECT_FALSE(e.get_value());

  e.set_value(true);// finally true
  EXPECT_TRUE(e.get_value());

  // test the conversion from shared_ptr
  std::shared_ptr<Parameter<bool>> ptr_e = std::make_shared<Event>("test2");
  ptr_e->set_value(true);
  EXPECT_TRUE(ptr_e->get_value());
  EXPECT_TRUE(std::static_pointer_cast<Event>(ptr_e)->read_value());
  EXPECT_FALSE(ptr_e->get_value());

  ptr_e->set_value(true);
  EXPECT_FALSE(ptr_e->get_value());

  ptr_e->set_value(false);
  ptr_e->set_value(true);
  EXPECT_TRUE(ptr_e->get_value());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}