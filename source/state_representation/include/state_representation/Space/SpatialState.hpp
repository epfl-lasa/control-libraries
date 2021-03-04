#pragma once

#include "state_representation/State.hpp"

namespace state_representation
{
	class SpatialState : public State
	{
	private:
		std::string reference_frame; ///< name of the reference frame

	public:
		/**
	 	 * @brief Empty constructor only specifying the type
	     */
		explicit SpatialState(const StateType& type);
		
		/**
	 	 * @brief Constructor with name and reference frame specification
	 	 * @param type the type of SpatialState (Cartesian or DualQuaternion)
	 	 * @param name the name of the State
	 	 * @param reference_frame the reference frame in which the state is expressed, by default world
	 	 * @param empty specify if the state is initialized as empty, default true
	     */
		explicit SpatialState(const StateType& type, const std::string& name, const std::string& reference_frame="world", const bool& empty=true);

		/**
	 	 * @brief Copy constructor from another SpatialState
	     */
		SpatialState(const SpatialState& state);

		/**
		 * @brief Copy assignement operator that have to be defined to the custom assignement operator
		 * @param state the state with value to assign
		 * @return reference to the current state with new values
		 */
		SpatialState& operator=(const SpatialState& state);

		/**
	 	 * @brief Getter of the reference frame as const reference
	     */
		const std::string get_reference_frame() const;

		/**
	 	 * @brief Setter of the reference frame
	     */
		virtual void set_reference_frame(const std::string& reference);

		/**
	 	 * @brief Check if the state is compatible for operations with the state given as argument
	 	 * @param state the state to check compatibility with
	     */
		virtual bool is_compatible(const State& state) const;
	};

	inline SpatialState& SpatialState::operator=(const SpatialState& state)
	{
		State::operator=(state);
		this->set_reference_frame(state.get_reference_frame());
		return (*this);
	}

	inline const std::string SpatialState::get_reference_frame() const
	{ 
		return this->reference_frame;
	}

	inline void SpatialState::set_reference_frame(const std::string& reference)
	{
		this->reference_frame = reference;
	}

	inline bool SpatialState::is_compatible(const State& state) const
	{
		bool compatible = (this->get_name() == state.get_name()) && (this->reference_frame == static_cast<const SpatialState&>(state).reference_frame);
		return compatible;
	}
}
