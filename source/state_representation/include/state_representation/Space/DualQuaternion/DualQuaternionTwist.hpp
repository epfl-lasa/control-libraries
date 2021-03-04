/**
 * @class DualQuaternionTwist
 * @brief Class to represent a Twist in Dual Quaternion space
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#ifndef STATEREPRESENTATION_SPACE_DUALQUATERNION_DUALQUATERNIONTWIST_H_
#define STATEREPRESENTATION_SPACE_DUALQUATERNION_DUALQUATERNIONTWIST_H_

#include "state_representation/Space/DualQuaternion/DualQuaternionState.hpp"

namespace state_representation
{
	class DualQuaternionTwist: public DualQuaternionState
	{
	private:
		Eigen::Vector3d position; ///< position of the point
		Eigen::Vector3d linear_velocity; ///< linear_velocity of the point

	public:
		/**
	 	 * @brief Constructor with name and reference frame provided
	 	 * @brief name the name of the state
	 	 * @brief reference the name of the reference frame
	     */
		explicit DualQuaternionTwist(const std::string& name, const std::string& reference="world");

		/**
	 	 * @brief Copy constructor of a DualQuaternionTwist
	     */
		DualQuaternionTwist(const DualQuaternionTwist& state);

		/**
	 	 * @brief Construct a DualQuaternionTwist from a position given as a vector of coordinates, a linear and an angular velocities
	     */
		explicit DualQuaternionTwist(const std::string& name, const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& angular_velocity, const Eigen::Vector3d& position, const std::string& reference);

		/**
	 	 * @brief Getter of the position attribute
	     */
		const Eigen::Vector3d& get_position() const;

		/**
	 	 * @brief Getter of the linear_velocity attribute
	     */
		const Eigen::Vector3d& get_linear_velocity() const;

		/**
	 	 * @brief Getter of the angular_velocity attribute
	     */
		const Eigen::Vector3d get_angular_velocity() const;

		/**
	 	 * @brief Setter of the linear_velocity
	     */
		void set_linear_velocity(const Eigen::Vector3d& linear_velocity);

		/**
	 	 * @brief Setter of the angular_velocity
	     */
		void set_angular_velocity(const Eigen::Vector3d& angular_velocity);

		/**
	 	 * @brief Setter of the position
	     */
		void set_position(const Eigen::Vector3d& position);

		/**
		 * @brief Overload the = operator to create a twist from a DualQuaternionState. Note that the 
		 * linear velocity will be computed from the value of the dual and the current position. 
		 */
		void operator=(const DualQuaternionState &q);

		/**
	 	 * @brief Initialize the DualQuaternionPose to a zero value
	     */
		void initialize();

		/**
		 * @brief Return a copy of the DualQuaternionTwist
		 * @return the copy
		 */
		const DualQuaternionTwist copy() const;

		/**
	 	 * @brief Overload the ostream operator for printing
	 	 * @param os the ostream to happend the string representing the state to
	 	 * @param state the state to print
	 	 * @return the appended ostream
	     */
		friend std::ostream& operator<<(std::ostream& os, const DualQuaternionTwist& state);
	};

	inline const Eigen::Vector3d& DualQuaternionTwist::get_position() const
	{
		return this->position;
	}

	inline const Eigen::Vector3d& DualQuaternionTwist::get_linear_velocity() const
	{
		return this->linear_velocity;
	}

	inline const Eigen::Vector3d DualQuaternionTwist::get_angular_velocity() const
	{
		return this->get_primary().vec();
	}

	inline void DualQuaternionTwist::set_linear_velocity(const Eigen::Vector3d& linear_velocity)
	{
		this->linear_velocity = linear_velocity;
		Eigen::Vector3d temp = this->get_linear_velocity() + this->get_position().cross(this->get_angular_velocity());
		this->set_dual(Eigen::Quaterniond(0, temp(0), temp(1), temp(2)));
	}

	inline void DualQuaternionTwist::set_angular_velocity(const Eigen::Vector3d& angular_velocity)
	{
		this->set_primary(Eigen::Quaterniond(0, angular_velocity(0), angular_velocity(1), angular_velocity(2)));
		Eigen::Vector3d temp = this->get_linear_velocity() + this->get_position().cross(this->get_angular_velocity());
		this->set_dual(Eigen::Quaterniond(0, temp(0), temp(1), temp(2)));
	}

	inline void DualQuaternionTwist::set_position(const Eigen::Vector3d& position)
	{
		this->position = position;
		Eigen::Vector3d temp = this->get_linear_velocity() + this->get_position().cross(this->get_angular_velocity());
		this->set_dual(Eigen::Quaterniond(0, temp(0), temp(1), temp(2)));
	}
}

#endif
