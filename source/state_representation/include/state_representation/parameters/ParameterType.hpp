#pragma once

namespace state_representation {

/**
 * @enum ParameterType
 * @brief The parameter value types.
 */
enum class ParameterType {
  BOOL,
  BOOL_ARRAY,
  INT,
  INT_ARRAY,
  DOUBLE,
  DOUBLE_ARRAY,
  STRING,
  STRING_ARRAY,
  STATE,
  VECTOR,
  MATRIX
};
}