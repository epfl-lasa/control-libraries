#include "state_representation/parameters/Event.hpp"
#include "state_representation/parameters/Parameter.hpp"
#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"
#include <gtest/gtest.h>

using namespace state_representation;

TEST(ParameterTest, Conversion) {
  Parameter<int> int_param("test");
  EXPECT_TRUE(int_param.is_empty());
  EXPECT_EQ(int_param.get_type(), StateType::PARAMETER_INT);
  int_param.set_value(2);
  EXPECT_EQ(int_param.get_value(), 2);

  std::vector<int> values{1, 2, 3};
  Parameter<std::vector<int>> int_array_param("test", values);
  EXPECT_FALSE(int_array_param.is_empty());
  EXPECT_EQ(int_array_param.get_type(), StateType::PARAMETER_INT_ARRAY);
  for (std::size_t i = 0; i < values.size(); ++i) {
    EXPECT_EQ(int_array_param.get_value().at(i), values.at(i));
  }
//  int_array_param.set_value({1, 2});

  Parameter<CartesianPose> test1("test", CartesianPose::Random("test"));
  Parameter<CartesianState> test2(test1);
  EXPECT_EQ(test2.get_type(), StateType::PARAMETER_CARTESIANPOSE);

  std::shared_ptr<Parameter<CartesianState>> test3 =
      std::make_shared<Parameter<CartesianState>>(Parameter<CartesianPose>("test", CartesianPose::Random("test")));
  EXPECT_EQ(test3->get_type(), StateType::PARAMETER_CARTESIANPOSE);
}

TEST(ParameterTest, Event) {
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
