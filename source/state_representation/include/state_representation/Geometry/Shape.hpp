/**
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#pragma once

#include <eigen3/Eigen/Core>
#include "state_representation/State.hpp"
#include "state_representation/Space/Cartesian/CartesianState.hpp"
#include "state_representation/Space/Cartesian/CartesianPose.hpp"
#include "state_representation/Space/Cartesian/CartesianTwist.hpp"
#include "state_representation/Exceptions/IncompatibleStatesException.hpp"
#include "state_representation/Exceptions/IncompatibleReferenceFramesException.hpp"

namespace StateRepresentation
{
	/**
	 * @class Shape
	 */
	class Shape: public State 
	{
	private:
		CartesianState center_state_; ///< pose and potentially velocities and accelerations of the shape if moving

	public:
		/**
		 * @brief Constructor with name but empty state
		 * @param type the type of shape as a StateType
		 * @param name name of the shape
		 * @param reference_frame the reference frame in which the state is expressed
		 */
		explicit Shape(const StateType& type, const std::string& name, const std::string& reference_frame="world");

		/**
		 * @brief Copy constructor from another shape
		 * @param shape the shape to copy
		 */
		explicit Shape(const Shape& shape);

		/**
		 * @brief Copy assignement operator that have to be defined to the custom assignement operator
		 * @param state the state with value to assign
		 * @return reference to the current state with new values
		 */
		Shape& operator=(const Shape& state);

		/**
		 * @brief Getter of the state
		 * @return the state
		 */
		const CartesianState& get_center_state() const;

		/**
		 * @brief Getter of the pose from the state
		 * @return the pose of the shape
		 */
		const CartesianPose& get_center_pose() const;

		/**
		 * @brief Getter of the position from the state
		 * @return the position of the shape
		 */
		const Eigen::Vector3d get_center_position() const;

		/**
		 * @brief Getter of the orientation from the state
		 * @return the orientation of the shape
		 */
		const Eigen::Quaterniond get_center_orientation() const;

		/**
		 * @brief Getter of the twist from the state
		 * @return the twist of the shape
		 */
		const CartesianTwist& get_center_twist() const;

		/**
		 * @brief Setter of the state
		 * @param state the new state
		 */
		void set_center_state(const CartesianState& state);

		/**
		 * @brief Setter of the pose
		 * @param pose the new pose
		 */
		void set_center_pose(const CartesianPose& pose);

		/**
		 * @brief Setter of the position
		 * @param position the new position
		 */
		void set_center_position(const Eigen::Vector3d& position);

		/**
		 * @brief Setter of the pose
		 * @param pose the new pose
		 */
		void set_center_orientation(const Eigen::Quaterniond& orientation);

		/**
	 	 * @brief Overload the ostream operator for printing
	 	 * @param os the ostream to happend the string representing the Shape to
	 	 * @param shape the Shape to print
	 	 * @return the appended ostream
	     */
		friend std::ostream& operator<<(std::ostream& os, const Shape& shape);
	};

	inline Shape& Shape::operator=(const Shape& state)
	{
		State::operator=(state);
		this->center_state_ = state.center_state_;
		return (*this);
	}

	inline const CartesianState& Shape::get_center_state() const
	{
		return this->center_state_;
	}

	inline const CartesianPose& Shape::get_center_pose() const
	{
		return static_cast<const CartesianPose&>(this->center_state_);
	}

	inline const Eigen::Vector3d Shape::get_center_position() const
	{
		return this->center_state_.get_position();
	}

	inline const Eigen::Quaterniond Shape::get_center_orientation() const
	{
		return this->center_state_.get_orientation();
	}

	inline const CartesianTwist& Shape::get_center_twist() const
	{
		return static_cast<const CartesianTwist&>(this->center_state_);
	}

	inline void Shape::set_center_state(const CartesianState& state)
	{
		this->center_state_ = state;
	}

	inline void Shape::set_center_pose(const CartesianPose& pose)
	{
		if (this->center_state_.get_reference_frame() != pose.get_reference_frame())
		{
			throw Exceptions::IncompatibleReferenceFramesException("The shape state and the given pose are not expressed in the same reference frame");
		}
		this->center_state_.set_pose(pose.get_position(), pose.get_orientation());	
	}

	inline void Shape::set_center_position(const Eigen::Vector3d& position)
	{
		this->center_state_.set_position(position);	
	}

	inline void Shape::set_center_orientation(const Eigen::Quaterniond& orientation)
	{
		this->center_state_.set_orientation(orientation);
	}
}