#include "state_representation/parameters/Event.hpp"
#include "state_representation/parameters/Parameter.hpp"
#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"

#include "state_representation/exceptions/InvalidParameterCastException.hpp"
#include "state_representation/exceptions/InvalidPointerException.hpp"

#include <gtest/gtest.h>

using namespace state_representation;

TEST(ParameterTest, Conversion) {
  Parameter<int> int_param("test");
  EXPECT_TRUE(int_param.is_empty());
  EXPECT_EQ(int_param.get_type(), StateType::PARAMETER);
  EXPECT_EQ(int_param.get_parameter_type(), ParameterType::INT);
  EXPECT_EQ(int_param.get_parameter_state_type(), StateType::NONE);
  int_param.set_value(2);
  EXPECT_EQ(int_param.get_value(), 2);
  EXPECT_EQ(typeid(int_param.get_value<double>()).name(), typeid(2.0).name());
  Parameter<double> double_param("double");
  EXPECT_NO_THROW(double_param = int_param);
  EXPECT_EQ(double_param.get_type(), StateType::PARAMETER);
  EXPECT_EQ(double_param.get_parameter_type(), ParameterType::DOUBLE);
  EXPECT_EQ(double_param.get_parameter_state_type(), StateType::NONE);
  EXPECT_EQ(double_param.get_name(), int_param.get_name());
  EXPECT_EQ(double_param.get_value(), 2.0);

  std::vector<int> values{1, 2, 3};
  Parameter<std::vector<int>> int_array_param("test", values);
  EXPECT_FALSE(int_array_param.is_empty());
  EXPECT_EQ(int_array_param.get_type(), StateType::PARAMETER);
  EXPECT_EQ(int_array_param.get_parameter_type(), ParameterType::INT_ARRAY);
  EXPECT_EQ(int_array_param.get_parameter_state_type(), StateType::NONE);
  for (std::size_t i = 0; i < values.size(); ++i) {
    EXPECT_EQ(int_array_param.get_value().at(i), values.at(i));
  }

  Parameter<CartesianPose> test1("test", CartesianPose::Random("test"));
  EXPECT_EQ(test1.get_parameter_state_type(), StateType::CARTESIAN_POSE);
  Parameter<CartesianState> test2(test1);
  EXPECT_EQ(test2.get_type(), StateType::PARAMETER);
  EXPECT_EQ(test2.get_parameter_type(), ParameterType::STATE);
  EXPECT_EQ(test2.get_parameter_state_type(), StateType::CARTESIAN_STATE);

  std::shared_ptr<Parameter<CartesianState>> test3 =
      std::make_shared<Parameter<CartesianState>>(Parameter<CartesianPose>("test", CartesianPose::Random("test")));
  EXPECT_EQ(test3->get_type(), StateType::PARAMETER);
  EXPECT_EQ(test3->get_parameter_type(), ParameterType::STATE);
  EXPECT_EQ(test3->get_parameter_state_type(), StateType::CARTESIAN_STATE);
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

TEST(ParameterTest, MakeShared) {
  auto pose = CartesianPose::Random("A", "B");
  auto param = make_shared_parameter("name", pose);
  EXPECT_EQ(param->get_name(), "name");
  EXPECT_EQ(param->get_type(), StateType::PARAMETER);
  EXPECT_EQ(param->get_parameter_type(), ParameterType::STATE);
  EXPECT_EQ(param->get_parameter_state_type(), StateType::CARTESIAN_POSE);
  EXPECT_EQ(param->is_empty(), false);
  EXPECT_EQ(param->get_value().get_name(), "A");
  EXPECT_EQ(param->get_value().get_reference_frame(), "B");
  EXPECT_TRUE(param->get_value().data().isApprox(pose.data()));
}

TEST(ParameterTest, ParameterThroughInterface) {
  auto pose = CartesianPose::Random("A", "B");
  std::shared_ptr<ParameterInterface> param_interface = make_shared_parameter("name", pose);

  auto param = param_interface->get_parameter<CartesianPose>();
  EXPECT_EQ(param->get_name(), "name");
  EXPECT_EQ(param->get_type(), StateType::PARAMETER);
  EXPECT_EQ(param->get_parameter_type(), ParameterType::STATE);
  EXPECT_EQ(param->get_parameter_state_type(), StateType::CARTESIAN_POSE);
  EXPECT_EQ(param->is_empty(), false);
  EXPECT_EQ(param->get_value().get_name(), "A");
  EXPECT_EQ(param->get_value().get_reference_frame(), "B");
  EXPECT_TRUE(param->get_value().data().isApprox(pose.data()));

  auto param_value = param_interface->get_parameter_value<CartesianPose>();
  EXPECT_EQ(param_value.get_type(), StateType::CARTESIAN_POSE);
  EXPECT_EQ(param_value.get_name(), "A");
  EXPECT_EQ(param_value.get_reference_frame(), "B");
  EXPECT_TRUE(param_value.data().isApprox(pose.data()));

  auto pose2 = CartesianPose::Random("C", "D");
  param_interface->set_parameter_value(pose2);
  param_value = param_interface->get_parameter_value<CartesianPose>();
  EXPECT_EQ(param_value.get_name(), "C");
  EXPECT_EQ(param_value.get_reference_frame(), "D");
  EXPECT_TRUE(param_value.data().isApprox(pose2.data()));
}

TEST(ParameterTest, ParameterInterfaceBadPointer) {
  ParameterInterface parameter_interface("name", ParameterType::INT);

  // by default (validate_pointer = true), throw when the ParameterInterface instance is not managed by any pointer
  EXPECT_THROW(parameter_interface.get_parameter<int>(), exceptions::InvalidPointerException);
  EXPECT_THROW(parameter_interface.get_parameter<int>(true), exceptions::InvalidPointerException);

  // using validate_pointer = false catches the exception but returns a null pointer
  EXPECT_NO_THROW(parameter_interface.get_parameter<int>(false));
  EXPECT_EQ(parameter_interface.get_parameter<int>(false), nullptr);
}

TEST(ParameterTest, ParameterInterfaceNullCast) {
  auto parameter_interface_ptr = std::make_shared<ParameterInterface>("name", ParameterType::INT);
  std::shared_ptr<Parameter<int>> parameter;

  // by default (validate_pointer = true), throw when the pointer does not address a Parameter instance
  EXPECT_THROW(parameter_interface_ptr->get_parameter<int>(), exceptions::InvalidParameterCastException);
  EXPECT_THROW(parameter_interface_ptr->get_parameter<int>(true), exceptions::InvalidParameterCastException);

  // using validate_pointer = false catches the exception but returns a null pointer
  EXPECT_NO_THROW(parameter = parameter_interface_ptr->get_parameter<int>(false));
  EXPECT_EQ(parameter, nullptr);
}

TEST(ParameterTest, ParameterInterfaceWrongTypeCast) {
  std::shared_ptr<ParameterInterface> parameter_interface_ptr = make_shared_parameter<int>("name", 1);

  std::shared_ptr<Parameter<int>> parameter_int;
  EXPECT_NO_THROW(parameter_int = parameter_interface_ptr->get_parameter<int>());
  EXPECT_NO_THROW(parameter_interface_ptr->get_parameter<int>(true));
  EXPECT_NO_THROW(parameter_interface_ptr->get_parameter<int>(false));
  EXPECT_NE(parameter_int, nullptr);
  EXPECT_EQ(parameter_int->get_value(), 1);

  std::shared_ptr<Parameter<std::string>> parameter_string;
  EXPECT_THROW(parameter_interface_ptr->get_parameter<std::string>(), exceptions::InvalidParameterCastException);
  EXPECT_THROW(parameter_interface_ptr->get_parameter<std::string>(true), exceptions::InvalidParameterCastException);
  EXPECT_NO_THROW(parameter_string = parameter_interface_ptr->get_parameter<std::string>(false));
  EXPECT_EQ(parameter_string, nullptr);

  EXPECT_NO_THROW(parameter_interface_ptr->get_parameter_value<int>());
  EXPECT_THROW(parameter_interface_ptr->get_parameter_value<std::string>(), exceptions::InvalidParameterCastException);
}