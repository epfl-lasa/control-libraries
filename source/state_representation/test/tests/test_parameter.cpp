#include "state_representation/geometry/Ellipsoid.hpp"
#include "state_representation/parameters/Event.hpp"
#include "state_representation/parameters/Parameter.hpp"
#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/space/joint/JointPositions.hpp"

#include "state_representation/exceptions/InvalidParameterCastException.hpp"
#include "state_representation/exceptions/InvalidPointerException.hpp"

#include <gtest/gtest.h>

using namespace state_representation;

template<typename T> using ParamT = std::vector<std::tuple<T, ParameterType, StateType>>;

static std::tuple<ParamT<bool>,
                  ParamT<std::vector<bool>>,
                  ParamT<int>,
                  ParamT<std::vector<int>>,
                  ParamT<double>,
                  ParamT<std::vector<double>>,
                  ParamT<std::string>,
                  ParamT<std::vector<std::string>>,
                  ParamT<CartesianState>,
                  ParamT<CartesianPose>,
                  ParamT<JointState>,
                  ParamT<JointPositions>,
                  ParamT<Ellipsoid>,
                  ParamT<Eigen::VectorXd>,
                  ParamT<Eigen::MatrixXd>>
    parameter_test_cases{{std::make_tuple(false, ParameterType::BOOL, StateType::NONE)}, {
    std::make_tuple(std::vector<bool>({true, false, true}), ParameterType::BOOL_ARRAY, StateType::NONE)
}, {std::make_tuple(1, ParameterType::INT, StateType::NONE)}, {
                             std::make_tuple(std::vector<int>({1, 2, 3}), ParameterType::INT_ARRAY, StateType::NONE)
                         }, {std::make_tuple(1.0, ParameterType::DOUBLE, StateType::NONE)}, {
                             std::make_tuple(
                                 std::vector<double>({1.0, 2.0, 3.0}), ParameterType::DOUBLE_ARRAY, StateType::NONE
                             )
                         }, {
                             std::make_tuple("test", ParameterType::STRING, StateType::NONE)
                         }, {
                             std::make_tuple(
                                 std::vector<std::string>({"1", "2", "3"}), ParameterType::STRING_ARRAY, StateType::NONE
                             )
                         }, {
                             std::make_tuple(
                                 CartesianState::Random("test", "base"), ParameterType::STATE,
                                 StateType::CARTESIAN_STATE
                             )
                         }, {
                             std::make_tuple(
                                 CartesianPose::Random("test", "base"), ParameterType::STATE, StateType::CARTESIAN_POSE
                             )
                         }, {
                             std::make_tuple(
                                 JointState::Random("test", 3), ParameterType::STATE, StateType::JOINT_STATE
                             )
                         }, {
                             std::make_tuple(
                                 JointPositions::Random("test", 3), ParameterType::STATE, StateType::JOINT_POSITIONS
                             )
                         }, {
                             std::make_tuple(Ellipsoid("test"), ParameterType::STATE, StateType::GEOMETRY_ELLIPSOID)
                         }, {
                             std::make_tuple(Eigen::VectorXd::Random(2), ParameterType::VECTOR, StateType::NONE)
                         }, {
                             std::make_tuple(Eigen::MatrixXd::Random(2, 2), ParameterType::MATRIX, StateType::NONE)
                         }};

template<typename T>
void expect_values_equal(const T& value_1, const T& value_2) {
  EXPECT_EQ(value_1, value_2);
}

template<>
void expect_values_equal(const CartesianState& value_1, const CartesianState& value_2) {
  EXPECT_EQ(value_1.get_name(), value_2.get_name());
  EXPECT_EQ(value_1.get_reference_frame(), value_2.get_reference_frame());
  EXPECT_TRUE(value_1.data().isApprox(value_2.data()));
}

template<>
void expect_values_equal(const CartesianPose& value_1, const CartesianPose& value_2) {
  EXPECT_EQ(value_1.get_name(), value_2.get_name());
  EXPECT_EQ(value_1.get_reference_frame(), value_2.get_reference_frame());
  EXPECT_TRUE(value_1.data().isApprox(value_2.data()));
}

template<>
void expect_values_equal(const JointState& value_1, const JointState& value_2) {
  EXPECT_EQ(value_1.get_name(), value_2.get_name());
  ASSERT_EQ(value_1.get_size(), value_2.get_size());
  for (std::size_t i = 0; i < value_1.get_size(); ++i) {
    EXPECT_EQ(value_1.get_names().at(i), value_2.get_names().at(i));
  }
  EXPECT_TRUE(value_1.data().isApprox(value_2.data()));
}

template<>
void expect_values_equal(const JointPositions& value_1, const JointPositions& value_2) {
  EXPECT_EQ(value_1.get_name(), value_2.get_name());
  ASSERT_EQ(value_1.get_size(), value_2.get_size());
  for (std::size_t i = 0; i < value_1.get_size(); ++i) {
    EXPECT_EQ(value_1.get_names().at(i), value_2.get_names().at(i));
  }
  EXPECT_TRUE(value_1.data().isApprox(value_2.data()));
}

template<>
void expect_values_equal(const Ellipsoid& value_1, const Ellipsoid& value_2) {
  EXPECT_EQ(value_1.get_name(), value_2.get_name());
  ASSERT_EQ(value_1.get_axis_lengths().size(), value_2.get_axis_lengths().size());
  for (std::size_t i = 0; i < value_1.get_axis_lengths().size(); ++i) {
    EXPECT_EQ(value_1.get_axis_length(i), value_1.get_axis_length(i));
  }
  EXPECT_EQ(value_1.get_center_pose().get_name(), value_2.get_center_pose().get_name());
  EXPECT_EQ(value_1.get_center_pose().get_reference_frame(), value_2.get_center_pose().get_reference_frame());
  EXPECT_TRUE(value_1.get_center_pose().data().isApprox(value_2.get_center_pose().data()));
}

template<typename T>
class ParameterTest : public testing::Test {
public:
  ParameterTest() : test_cases_{std::get<ParamT<T>>(parameter_test_cases)} {}
protected:
  ParamT<T> test_cases_;
};
TYPED_TEST_SUITE_P(ParameterTest);

TYPED_TEST_P(ParameterTest, Construction) {
  for (auto const& test_case: this->test_cases_) {
    Parameter<TypeParam> param("test");
    EXPECT_EQ(param.get_name(), "test");
    EXPECT_EQ(param.get_parameter_type(), std::get<1>(test_case));
    EXPECT_EQ(param.get_parameter_state_type(), std::get<2>(test_case));
    EXPECT_TRUE(param.is_empty());
    expect_values_equal(param.get_value(), TypeParam());
    ParameterInterface param_interface(param);
    EXPECT_EQ(param_interface.get_name(), param.get_name());
    EXPECT_EQ(param_interface.get_type(), StateType::PARAMETER);
    EXPECT_EQ(param_interface.get_parameter_type(), param.get_parameter_type());
    EXPECT_EQ(param_interface.get_parameter_state_type(), param.get_parameter_state_type());
    param.set_value(std::get<0>(test_case));
    EXPECT_FALSE(param.is_empty());
    expect_values_equal(param.get_value(), std::get<0>(test_case));
  }
}

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

TYPED_TEST_P(ParameterTest, MakeShared) {
  for (auto const& test_case: this->test_cases_) {
    auto param = make_shared_parameter("test", std::get<0>(test_case));
    EXPECT_EQ(param->get_name(), "test");
    EXPECT_EQ(param->get_type(), StateType::PARAMETER);
    EXPECT_FALSE(param->is_empty());
    EXPECT_EQ(param->get_parameter_type(), std::get<1>(test_case));
    EXPECT_EQ(param->get_parameter_state_type(), std::get<2>(test_case));
    expect_values_equal(param->get_value(), std::get<0>(test_case));
  }
}

TYPED_TEST_P(ParameterTest, ParameterThroughInterface) {
  for (auto const& test_case: this->test_cases_) {
    std::shared_ptr<ParameterInterface> param_interface = make_shared_parameter("test", std::get<0>(test_case));

    auto param = param_interface->get_parameter<TypeParam>();
    EXPECT_EQ(param->get_name(), "test");
    EXPECT_EQ(param->get_type(), StateType::PARAMETER);
    EXPECT_FALSE(param->is_empty());
    EXPECT_EQ(param->get_parameter_type(), std::get<1>(test_case));
    EXPECT_EQ(param->get_parameter_state_type(), std::get<2>(test_case));
    expect_values_equal(param->get_value(), std::get<0>(test_case));

    auto param_value = param_interface->get_parameter_value<TypeParam>();
    expect_values_equal(param_value, std::get<0>(test_case));

    param_interface->set_parameter_value(TypeParam());
    param_value = param_interface->get_parameter_value<TypeParam>();
    expect_values_equal(param_value, TypeParam());
  }
}

TYPED_TEST_P(ParameterTest, ParameterInterfaceBadPointer) {
  for (auto const& test_case: this->test_cases_) {
    ParameterInterface parameter_interface("test", std::get<1>(test_case));

    // by default (validate_pointer = true), throw when the ParameterInterface instance is not managed by any pointer
    EXPECT_THROW(parameter_interface.get_parameter<TypeParam>(), exceptions::InvalidPointerException);
    EXPECT_THROW(parameter_interface.get_parameter<TypeParam>(true), exceptions::InvalidPointerException);

    // using validate_pointer = false catches the exception but returns a null pointer
    EXPECT_NO_THROW(parameter_interface.get_parameter<TypeParam>(false));
    EXPECT_EQ(parameter_interface.get_parameter<TypeParam>(false), nullptr);
  }
}

TYPED_TEST_P(ParameterTest, ParameterInterfaceNullCast) {
  for (auto const& test_case: this->test_cases_) {
    auto parameter_interface_ptr = std::make_shared<ParameterInterface>("test", std::get<1>(test_case));
    std::shared_ptr<Parameter<TypeParam>> parameter;

    // by default (validate_pointer = true), throw when the pointer does not address a Parameter instance
    EXPECT_THROW(parameter_interface_ptr->template get_parameter<TypeParam>(),
                 exceptions::InvalidParameterCastException);
    EXPECT_THROW(parameter_interface_ptr->template get_parameter<TypeParam>(true),
                 exceptions::InvalidParameterCastException);

    // using validate_pointer = false catches the exception but returns a null pointer
    EXPECT_NO_THROW(parameter = parameter_interface_ptr->template get_parameter<TypeParam>(false));
    EXPECT_EQ(parameter, nullptr);
  }
}

TYPED_TEST_P(ParameterTest, ParameterInterfaceWrongTypeCast) {
  for (auto const& test_case: this->test_cases_) {
    std::shared_ptr<ParameterInterface>
        parameter_interface_ptr = make_shared_parameter<TypeParam>("test", std::get<0>(test_case));

    std::shared_ptr<Parameter<TypeParam>> parameter;
    EXPECT_NO_THROW(parameter = parameter_interface_ptr->get_parameter<TypeParam>());
    EXPECT_NO_THROW(parameter_interface_ptr->get_parameter<TypeParam>(true));
    EXPECT_NO_THROW(parameter_interface_ptr->get_parameter<TypeParam>(false));
    EXPECT_NE(parameter, nullptr);
    expect_values_equal(parameter->get_value(), std::get<0>(test_case));

    if (std::get<1>(test_case) == ParameterType::STRING) {
      std::shared_ptr<Parameter<int>> parameter_int;
      EXPECT_THROW(parameter_interface_ptr->get_parameter<int>(), exceptions::InvalidParameterCastException);
      EXPECT_THROW(parameter_interface_ptr->get_parameter<int>(true), exceptions::InvalidParameterCastException);
      EXPECT_NO_THROW(parameter_int = parameter_interface_ptr->get_parameter<int>(false));
      EXPECT_EQ(parameter_int, nullptr);

      EXPECT_NO_THROW(parameter_interface_ptr->get_parameter_value<TypeParam>());
      EXPECT_THROW(parameter_interface_ptr->get_parameter_value<int>(), exceptions::InvalidParameterCastException);
    } else {
      std::shared_ptr<Parameter<std::string>> parameter_string;
      EXPECT_THROW(parameter_interface_ptr->get_parameter<std::string>(), exceptions::InvalidParameterCastException);
      EXPECT_THROW(parameter_interface_ptr->get_parameter<std::string>(true),
                   exceptions::InvalidParameterCastException);
      EXPECT_NO_THROW(parameter_string = parameter_interface_ptr->get_parameter<std::string>(false));
      EXPECT_EQ(parameter_string, nullptr);

      EXPECT_NO_THROW(parameter_interface_ptr->get_parameter_value<TypeParam>());
      EXPECT_THROW(parameter_interface_ptr->get_parameter_value<std::string>(),
                   exceptions::InvalidParameterCastException);
    }
  }
}

REGISTER_TYPED_TEST_SUITE_P(ParameterTest, Construction, MakeShared, ParameterThroughInterface,
                            ParameterInterfaceBadPointer, ParameterInterfaceNullCast, ParameterInterfaceWrongTypeCast);

using ParameterTestTypes = testing::Types<bool,
                                          std::vector<bool>,
                                          int,
                                          std::vector<int>,
                                          double,
                                          std::vector<double>,
                                          std::string,
                                          std::vector<std::string>,
                                          CartesianState,
                                          CartesianPose,
                                          JointPositions,
                                          JointState,
                                          Ellipsoid,
                                          Eigen::VectorXd,
                                          Eigen::MatrixXd>;
INSTANTIATE_TYPED_TEST_SUITE_P(TestPrefix, ParameterTest, ParameterTestTypes);