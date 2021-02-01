/**
 * @class DualQuaternionPose
 * @brief Class to represent a Pose in Dual Quaternion space
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#ifndef STATEREPRESENTATION_SPACE_DUALQUATERNION_DUALQUATERNIONPOSE_H_
#define STATEREPRESENTATION_SPACE_DUALQUATERNION_DUALQUATERNIONPOSE_H_

#include "state_representation/Space/DualQuaternion/DualQuaternionState.hpp"

namespace StateRepresentation 
{
	class DualQuaternionPose: public DualQuaternionState
	{
	private:
		Eigen::Vector3d position; ///< position of the point

	public:
		/**
	 	 * @brief Constructor with name and reference frame provided
	 	 * @brief name the name of the state
	 	 * @brief reference the name of the reference frame
	     */
		explicit DualQuaternionPose(const std::string& name, const std::string& reference="world");

		/**
	 	 * @brief Copy constructor of a DualQuaternionPose
	     */
		DualQuaternionPose(const DualQuaternionPose& state);

		/**
	 	 * @brief Copy constructor of a DualQuaternionPose from a DualQuaternionState
	     */
		DualQuaternionPose(const DualQuaternionState& state);

		/**
	 	 * @brief Construct a DualQuaternionPose from a position given as a vector of coordinates and a quaternion.
	     */
		explicit DualQuaternionPose(const std::string& name, const Eigen::Vector3d& position, const Eigen::Quaterniond& rotation, const std::string& reference="world");

		/**
	 	 * @brief Getter of the position attribute
	     */
		const Eigen::Vector3d get_position() const;

		/**
	 	 * @brief Getter of the orientation from the primary
	     */
		const Eigen::Quaterniond& get_orientation() const;

		/**
	 	 * @brief Setter of the orientation
	     */
		void set_orientation(const Eigen::Quaterniond& orientation);

		/**
	 	 * @brief Setter of the psotion from a vector
	     */
		void set_position(const Eigen::Vector3d& position);

		/**
	 	 * @brief Setter of the position from a dual
	     */
		void set_position(const Eigen::Quaterniond& dual);

		/**
		 * @brief compute the conjugate of the current DualQuaternionPose
		 * @return the conjugate
		 */
		const DualQuaternionPose conjugate() const;

		/**
		 * @brief compute the inverse of the current DualQuaternionPose
		 * @return the inverse
		 */
		const DualQuaternionPose inverse() const;

		/**
	 	 * @brief Overload the *= operator
	 	 * @param q DualQuaternion to multiply with
	 	 * @return the current DualQuaternion multiply by the DualQuaternion given in argument
	     */
		DualQuaternionPose& operator*=(const DualQuaternionPose& q);

		/**
	 	 * @brief Overload the * operator
	 	 * @param p DualQuaternionState to multiply with
	 	 * @return the current DualQuaternionState multiply by the DualQuaternionState given in argument
	     */
		const DualQuaternionPose operator*(const DualQuaternionPose& p) const;

		/**
	 	 * @brief Overload the *= operator
	 	 * @param q DualQuaternion to multiply with
	 	 * @return the current DualQuaternion multiply by the DualQuaternion given in argument
	     */
		DualQuaternionPose& operator*=(const DualQuaternionState& s);

		/**
	 	 * @brief Overload the * operator
	 	 * @param p DualQuaternionState to multiply with
	 	 * @return the current DualQuaternionState multiply by the DualQuaternionState given in argument
	     */
		const DualQuaternionPose operator*(const DualQuaternionState& s) const;

		/**
	 	 * @brief Overload the = operator with a DualQuaternionState
	 	 * @param s DualQuaternion to copy values from
	     */
		void operator=(const DualQuaternionState& s);

		/**
	 	 * @brief Initialize the DualQuaternionPose to a zero value
	     */
		void initialize();

		/**
		 * @brief Return a copy of the DualQuaternionPose
		 * @return the copy
		 */
		const DualQuaternionPose copy() const;

		/**
		 * @brief Calculate the log of a dual quaternion
		 * @param state the dual quaternion to calcualte the log on
		 * @return the log of the dual quaternion
		 */
		friend const DualQuaternionState log(const DualQuaternionPose& state);

		/**
	 	 * @brief Overload the ostream operator for printing
	 	 * @param os the ostream to happend the string representing the state to
	 	 * @param state the state to print
	 	 * @return the appended ostream
	     */
		friend std::ostream& operator<<(std::ostream& os, const DualQuaternionPose& state);
	};

	inline const Eigen::Vector3d DualQuaternionPose::get_position() const
	{
		return 2 * (this->get_dual() * this->get_primary().conjugate()).vec();
	}

	inline const Eigen::Quaterniond& DualQuaternionPose::get_orientation() const
	{
		return this->get_primary();
	}

	inline void DualQuaternionPose::set_orientation(const Eigen::Quaterniond& orientation)
	{
		Eigen::Quaterniond temp = orientation;
		if(orientation.norm() - 1 < 1e-4) temp.normalize();
		if(orientation.w() < 0) temp = Eigen::Quaterniond(-orientation.w(), -orientation.x(), -orientation.y(), -orientation.z());
		this->set_primary(temp);
		this->set_dual(Eigen::Quaterniond(0.5 * (Eigen::Quaterniond(0, this->position(0), this->position(1), this->position(2)) * this->get_primary()).coeffs()));
	}

	inline void DualQuaternionPose::set_position(const Eigen::Vector3d& position)
	{
		this->position = position;
		this->set_dual(Eigen::Quaterniond(0.5 * (Eigen::Quaterniond(0, this->position(0), this->position(1), this->position(2)) * this->get_primary()).coeffs()));
	}

	inline void DualQuaternionPose::set_position(const Eigen::Quaterniond& dual)
	{
        // WARNING - position is updated, but not dual. This might lead to conficts / different result between position and get_position
		this->position = 2 * (dual * this->get_primary().conjugate()).vec();
        this->set_dual(dual);
	}
}

#endif
