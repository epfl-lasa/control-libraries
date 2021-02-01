/**
 * @author Baptiste Busch
 * @date 2019/09/13
 */

#pragma once

#include "state_representation/Robot/JointState.hpp"

namespace StateRepresentation 
{
	/**
	 * @class JointTorques
	 * @brief Class to define torques of the joints
	 */
	class JointTorques: public JointState
	{
	public:
		/**
		 * Empty constructor
		 */
		explicit JointTorques();

		/**
	 	 * @brief Constructor with name and number of joints provided
	 	 * @brief name the name of the state
	 	 * @brief nb_joints the number of joints for initialization
	     */
		explicit JointTorques(const std::string& robot_name, unsigned int nb_joints=0);

		/**
	 	 * @brief Constructor with name and list of joint names provided
	 	 * @brief name the name of the state
	 	 * @brief joint_names list of joint names
	     */
		explicit JointTorques(const std::string& robot_name, const std::vector<std::string>& joint_names);

		/**
	 	 * @brief Constructor with name and torques values provided
	 	 * @brief name the name of the state
	 	 * @brief torques the vector of torques
	     */
		explicit JointTorques(const std::string& robot_name, const Eigen::VectorXd& torques);

		/**
	 	 * @brief Constructor with name, a list of joint names  and torques values provided
	 	 * @brief name the name of the state
	 	 * @brief joint_names list of joint names
	 	 * @brief torques the vector of torques
	     */
		explicit JointTorques(const std::string& robot_name, const std::vector<std::string>& joint_names, const Eigen::VectorXd& torques);

		/**
	 	 * @brief Copy constructor
	     */
		JointTorques(const JointTorques& torques);

		/**
	 	 * @brief Copy constructor from a JointState
	     */
		JointTorques(const JointState& state);

		/**
		 * @brief Copy assignement operator that have to be defined to the custom assignement operator
		 * @param state the state with value to assign
		 * @return reference to the current state with new values
		 */
		JointTorques& operator=(const JointTorques& state);

		/**
		 * @brief Set the values of the  torques from an Eigen Vector
		 * @param torques the torques as an Eigen Vector
		 */
		JointTorques& operator=(const Eigen::VectorXd& torques);

		/**
	 	 * @brief Overload the += operator with an Eigen Vector
	 	 * @param vector Eigen Vector to add
	 	 * @return the JointTorques added the vector given in argument
	     */
		JointTorques& operator+=(const Eigen::VectorXd& vector);

		/**
	 	 * @brief Overload the += operator
	 	 * @param torques JointTorques to add
	 	 * @return the current JointTorques added the JointTorques given in argument
	     */
		JointTorques& operator+=(const JointTorques& torques);

		/**
	 	 * @brief Overload the + operator with a  Eigen Vector
	 	 * @param vector Eigen Vector to add
	 	 * @return the JointTorques added the vector given in argument
	     */
		const JointTorques operator+(const Eigen::VectorXd& vector) const;

		/**
	 	 * @brief Overload the + operator
	 	 * @param torques JointTorques to add
	 	 * @return the current JointTorques added the JointTorques given in argument
	     */
		const JointTorques operator+(const JointTorques& torques) const;

		/**
	 	 * @brief Overload the -= operator with a  Eigen Vector
	 	 * @param vector Eigen Vector to substract
	 	 * @return the JointTorques substracted the vector given in argument
	     */
		JointTorques& operator-=(const Eigen::VectorXd& vector);

		/**
	 	 * @brief Overload the -= operator
	 	 * @param torques JointTorques to substract
	 	 * @return the current JointTorques substracted the JointTorques given in argument
	     */
		JointTorques& operator-=(const JointTorques& torques);

		/**
	 	 * @brief Overload the - operator with an Eigen Vector
	 	 * @param vector Eigen Vector to substract
	 	 * @return the JointTorques substracted the vector given in argument
	     */
		const JointTorques operator-(const Eigen::VectorXd& vector) const;

		/**
	 	 * @brief Overload the - operator
	 	 * @param torques JointTorques to substract
	 	 * @return the current JointTorques substracted the JointTorques given in argument
	     */
		const JointTorques operator-(const JointTorques& torques) const;

		/**
		 * @brief Return a copy of the JointTorques
		 * @return the copy
		 */
		const JointTorques copy() const;

		/**
		 * @brief Return the value of the torques vector as Eigen array
		 * @retrun the Eigen array representing the torques
		 */
		const Eigen::ArrayXd array() const;

		/**
		 * @brief Clamp inplace the magnitude of the velocity to the values in argument
		 * @param max_absolute the maximum magnitude of torque for each joint 
		 * @param noise_ratio if provided, this value will be used to apply a deadzone under which
		 * the torque will be set to 0
		 */
		void clamp(const Eigen::ArrayXd& max_absolute, const Eigen::ArrayXd& noise_ratio);

		/**
		 * @brief Clamp inplace the magnitude of the velocity to the values in argument
		 * @param max_absolute the maximum magnitude of torque for each joint 
		 * @param noise_ratio if provided, this value will be used to apply a deadzone under which
		 * the torque will be set to 0
		 * @return the clamped JointTorques
		 */
		const JointTorques clamped(const Eigen::ArrayXd& max_absolute, const Eigen::ArrayXd& noise_ratio) const;

		/**
	 	 * @brief Overload the ostream operator for printing
	 	 * @param os the ostream to append the string representing the state
	 	 * @param state the state to print
	 	 * @return the appended ostream
	     */
		friend std::ostream& operator<<(std::ostream& os, const JointTorques& torques);

		/**
	 	 * @brief Overload the + operator with an Eigen Vector
	 	 * @param vector Eigen Vector to add
	 	 * @param torques JointTorques to add
	 	 * @return the Eigen Vector plus the JointTorques represented as a JointTorques
	     */
		friend const JointTorques operator+(const Eigen::VectorXd& vector, const JointTorques& torques);

		/**
	 	 * @brief Overload the - operator with a  Eigen Vector
	 	 * @param vector Eigen Vector
	 	 * @param torques JointTorques to substract
	 	 * @return the Eigen Vector minus the JointTorques represented as a JointTorques
	     */
		friend const JointTorques operator-(const Eigen::VectorXd& vector, const JointTorques& torques);

		/**
	 	 * @brief Overload the * operator with a scalar
	 	 * @param lambda the scalar to multiply with
	 	 * @return the JointTorques provided multiply by lambda
	     */
		friend const JointTorques operator*(double lambda, const JointTorques& torques);

		/**
	 	 * @brief Overload the * operator with an array of gains
	 	 * @param lambda the array to multiply with
	 	 * @return the JointTorques provided multiply by lambda
	     */
		friend const JointTorques operator*(const Eigen::ArrayXd& lambda, const JointTorques& torques);

		/**
	 	 * @brief Overload the / operator with a scalar
	 	 * @param lambda the scalar to divide with
	 	 * @return the JointTorques provided divided by lambda
	     */
		friend const JointTorques operator/(const JointTorques& torques, double lambda);

		/**
	 	 * @brief Overload the / operator with an array of gains
	 	 * @param lambda the array to divide with
	 	 * @return the JointTorques provided divided by lambda
	     */
		friend const JointTorques operator/(const JointTorques& torques, const Eigen::ArrayXd& lambda);
	};

	inline JointTorques& JointTorques::operator=(const JointTorques& state)
	{
		JointState::operator=(state);
		return (*this);
	}
}
