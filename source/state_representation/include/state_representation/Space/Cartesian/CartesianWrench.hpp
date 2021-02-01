/**
 * @author Baptiste Busch
 * @date 2019/09/13
 */

#pragma once

#include "state_representation/Space/Cartesian/CartesianState.hpp"

namespace StateRepresentation 
{
	/**
	 * @class CartesianWrench
	 * @brief Class to define wrench in cartesian space as 3D force and torque vectors
	 */
	class CartesianWrench: public CartesianState
	{
	public:
		/**
		 * Empty constructor
		 */
		explicit CartesianWrench();

		/**
	 	 * @brief Empty constructor for a CartesianWrench
	     */
		explicit CartesianWrench(const std::string& name, const std::string& reference="world");

		/**
	 	 * @brief Copy constructor
	     */
		CartesianWrench(const CartesianWrench& wrench);

		/**
	 	 * @brief Copy constructor from a CartesianState
	     */
		CartesianWrench(const CartesianState& state);
		
		/**
	 	 * @brief Construct a CartesianWrench from a force given as a vector of coordinates.
	     */
		explicit CartesianWrench(const std::string& name, const Eigen::Vector3d& force, const std::string& reference="world");

		/**
	 	 * @brief Construct a CartesianWrench from a force given as a vector of coordinates and a quaternion.
	     */
		explicit CartesianWrench(const std::string& name, const Eigen::Vector3d& force, const Eigen::Vector3d& torque, const std::string& reference="world");

		/**
	 	 * @brief Construct a CartesianWrench from a single 6d wrench vector
	     */
		explicit CartesianWrench(const std::string& name, const Eigen::Matrix<double, 6, 1>& wrench, const std::string& reference="world");

		/**
		 * @brief Copy assignement operator that have to be defined to the custom assignement operator
		 * @param pose the pose with value to assign
		 * @return reference to the current pose with new values
		 */
		CartesianWrench& operator=(const CartesianWrench& pose);

		/**
		 * @brief Set the values of the 6D wrench from a 6D Eigen Vector
		 * @param wrench the wrench as an Eigen Vector
		 */
		CartesianWrench& operator=(const Eigen::Matrix<double, 6, 1>& wrench);

		/**
	 	 * @brief Overload the += operator with a 6D Eigen Vector
	 	 * @param vector Eigen Vector to add
	 	 * @return the CartesianWrench added the vector given in argument
	     */
		CartesianWrench& operator+=(const Eigen::Matrix<double, 6, 1>& vector);

		/**
	 	 * @brief Overload the += operator
	 	 * @param wrench CartesianWrench to add
	 	 * @return the current CartesianWrench added the CartesianWrench given in argument
	     */
		CartesianWrench& operator+=(const CartesianWrench& wrench);

		/**
	 	 * @brief Overload the + operator with a 6D Eigen Vector
	 	 * @param vector Eigen Vector to add
	 	 * @return the CartesianWrench added the vector given in argument
	     */
		const CartesianWrench operator+(const Eigen::Matrix<double, 6, 1>& vector) const;

		/**
	 	 * @brief Overload the + operator
	 	 * @param wrench CartesianWrench to add
	 	 * @return the current CartesianWrench added the CartesianWrench given in argument
	     */
		const CartesianWrench operator+(const CartesianWrench& wrench) const;

		/**
	 	 * @brief Overload the -= operator with a 6D Eigen Vector
	 	 * @param vector Eigen Vector to substract
	 	 * @return the CartesianWrench substracted the vector given in argument
	     */
		CartesianWrench& operator-=(const Eigen::Matrix<double, 6, 1>& vector);

		/**
	 	 * @brief Overload the -= operator
	 	 * @param wrench CartesianWrench to substract
	 	 * @return the current CartesianWrench minus the CartesianWrench given in argument
	     */
		CartesianWrench& operator-=(const CartesianWrench& wrench);

		/**
	 	 * @brief Overload the - operator with a 6D Eigen Vector
	 	 * @param vector Eigen Vector to substract
	 	 * @return the CartesianWrench substracted the vector given in argument
	     */
		const CartesianWrench operator-(const Eigen::Matrix<double, 6, 1>& vector) const;

		/**
	 	 * @brief Overload the - operator
	 	 * @param wrench CartesianWrench to substract
	 	 * @return the current CartesianWrench minus the CartesianWrench given in argument
	     */
		const CartesianWrench operator-(const CartesianWrench& wrench) const;

		/**
	 	 * @brief Overload the = operator from a CartesianState
	 	 * @param state CartesianState to get the wrench from
	     */
		void operator=(const CartesianState& state);

		/**
	 	 * @brief Overload the *= operator with a scalar
	 	 * @param lambda the scalar to multiply with
	 	 * @return the CartesianWrench multiply by lambda
	     */
		CartesianWrench& operator*=(double lambda);

		/**
	 	 * @brief Overload the * operator with a scalar
	 	 * @param lambda the scalar to multiply with
	 	 * @return the CartesianWrench multiply by lambda
	     */
		const CartesianWrench operator*(double lambda) const;

		/**
		 * @brief Clamp inplace the magnitude of the force to the values in argument
		 * @param max_force the maximum magnitude of the force
		 * @param max_torque the maximum magnitude of the torque
		 * @param noise_ratio if provided, this value will be used to apply a deadzone under which
		 * the signal will be set to 0
		 */
		void clamp(double max_force, double max_torque, double noise_ratio=0);

		/**
		 * @brief Return the clamped velocity
		 * @param max_force the maximum magnitude of the force
		 * @param max_torque the maximum magnitude of the torque
		 * @param noise_ratio if provided, this value will be used to apply a deadzone under which
		 * the signal will be set to 0
		 * @return the clamped velocity
		 */
		const CartesianWrench clamped(double max_force, double max_torque, double noise_ratio=0) const;

		/**
		 * @brief Return a copy of the CartesianWrench
		 * @return the copy
		 */
		const CartesianWrench copy() const;

		/**
		 * @brief Return the value of the 6D wrench as Eigen array
		 * @retrun the Eigen array representing the wrench
		 */
		const Eigen::Array<double, 6, 1> array() const;

		/**
	 	 * @brief Overload the ostream operator for printing
	 	 * @param os the ostream to happend the string representing the CartesianWrench to
	 	 * @param CartesianWrench the CartesianWrench to print
	 	 * @return the appended ostream
	     */
		friend std::ostream& operator<<(std::ostream& os, const CartesianWrench& wrench);

		/**
	 	 * @brief Overload the + operator with a 6D Eigen Vector
	 	 * @param vector Eigen Vector to add
	 	 * @param wrench CartesianWrench to add
	 	 * @return the Eigen Vector plus the CartesianWrench represented as a CartesianWrench
	     */
		friend const CartesianWrench operator+(const Eigen::Matrix<double, 6, 1>& vector, const CartesianWrench& wrench);

		/**
	 	 * @brief Overload the - operator with a 6D Eigen Vector
	 	 * @param vector Eigen Vector
	 	 * @param wrench CartesianWrench to substract
	 	 * @return the Eigen Vector minus the CartesianWrench represented as a CartesianWrench
	     */
		friend const CartesianWrench operator-(const Eigen::Matrix<double, 6, 1>& vector, const CartesianWrench& wrench);

		/**
	 	 * @brief Overload the * operator with a scalar
	 	 * @param lambda the scalar to multiply with
	 	 * @return the CartesianWrench provided multiply by lambda
	     */
		friend const CartesianWrench operator*(double lambda, const CartesianWrench& wrench);
	};

	inline CartesianWrench& CartesianWrench::operator=(const CartesianWrench& wrench)
	{
		CartesianState::operator=(wrench);
		return (*this);
	}
}
