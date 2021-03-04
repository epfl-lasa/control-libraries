#pragma once

#include <chrono>
#include <deque>
#include "state_representation/State.hpp"

namespace state_representation
{
	template <class StateT>
	class Trajectory : public State
	{
	private:
		std::deque<StateT> points_;
		std::deque<std::chrono::nanoseconds> times_;
		std::string reference_frame_; ///< name of the reference frame
		std::vector<std::string> joint_names_; ///< names of the joints

	public:

		/**
	 	 * @brief Empty constructor
	     */
		explicit Trajectory();

		/**
	 	 * @brief Constructor with name and reference frame provided
	 	 * @brief name the name of the state
	     */
		explicit Trajectory(const std::string& name);

		/**
	 	 * @brief Getter of the reference frame as const reference
	     */
		const std::string get_reference_frame() const;

		/**
	 	 * @brief Setter of the reference frame
	     */
		virtual void set_reference_frame(const std::string& reference_frame);

		/**
	 	 * @brief Getter of the names attribute
	     */
		const std::vector<std::string>& get_joint_names() const;

		/**
	 	 * @brief Setter of the names attribute from the number of joints
	     */
		void set_joint_names(unsigned int nb_joints);

		/**
	 	 * @brief Setter of the names attribute from the joints names
	     */
		void set_joint_names(const std::vector<std::string>& joint_names);

		/**
		 * @brief Initialize trajectory
		 */
		void initialize();

		/**
		 * @brief Add new point and corresponding time to trajectory
		 */
		template <typename DurationT>
		void add_point(const StateT& new_point, const std::chrono::duration<int64_t, DurationT>& new_time);

		/**
		 * @brief Insert new point and corresponding time to trajectory between two already existing points
		 */
		template <typename DurationT>
		void insert_point(const StateT& new_point, const std::chrono::duration<int64_t, DurationT>& new_time, int pos);

		/**
		 * @brief Delete last point and corresponding time from trajectory
		 */
		void delete_point();

		/**
		 * @brief Clear trajectory
		 */
		void clear();

		/**
		 * @brief Get attribute list of trajectory points
		 */
		const std::deque<StateT>& get_points() const;

		/**
		 * @brief Get the trajectory point at given index
		 * @param index the index
		 */
		const StateT& get_point(unsigned int index) const;

		/**
		 * @brief Get the trajectory point at given index
		 * @param index the index
		 */
		StateT& get_point(unsigned int index);

		/**
		 * @brief Get attribute list of trajectory times
		 */
		const std::deque<std::chrono::nanoseconds>& get_times() const;

		/**
		 * @brief Get attribute number of point in trajectory
		 */
		int get_size() const;

		/**
		 * @brief Operator overload for returning a single trajectory point and corresponding time
		 */
		const std::pair<StateT, std::chrono::nanoseconds> operator[](unsigned int idx) const;

		/**
		 * @brief Operator overload for returning a single trajectory point and corresponding time
		 */
		std::pair<StateT, std::chrono::nanoseconds> operator[](unsigned int idx);

	};
	
	template <class StateT> 
	Trajectory<StateT>::Trajectory():
	State(StateType::TRAJECTORY)
	{
		this->initialize();
	}

	template <class StateT> 
	Trajectory<StateT>::Trajectory(const std::string& name):
	State(StateType::TRAJECTORY, name),
	reference_frame_("")
	{
		this->initialize();
	}

	template <class StateT>
	inline const std::string Trajectory<StateT>::get_reference_frame() const
	{ 
		return this->reference_frame_;
	}

	template <class StateT>
	inline void Trajectory<StateT>::set_reference_frame(const std::string& reference_frame)
	{
		this->reference_frame_ = reference_frame;
	}

	template <class StateT>
	inline const std::vector<std::string>& Trajectory<StateT>::get_joint_names() const
	{
		return this->joint_names_;
	}

	template <class StateT>
	inline void Trajectory<StateT>::set_joint_names(unsigned int nb_joints)
	{
		this->joint_names_.resize(nb_joints);
		for(unsigned int i=0; i<nb_joints; i++)
		{
			this->joint_names_[i] = "joint_" + std::to_string(i+1);
		}
	}

	template <class StateT>
	inline void Trajectory<StateT>::set_joint_names(const std::vector<std::string>& joint_names)
	{
		this->joint_names_ = joint_names;
	}

	template <class StateT>
	void Trajectory<StateT>::initialize()
	{
		this->State::initialize();
		this->points_.clear();
		this->times_.clear();
	}

	template <class StateT> template <typename DurationT>
	void Trajectory<StateT>::add_point(const StateT& new_point, const std::chrono::duration<int64_t, DurationT>& new_time)
	{
		this->set_filled();
		this->points_.push_back(new_point);

		if(!this->times_.empty())
		{
			auto const previous_time = this->times_.back();
			this->times_.push_back(previous_time + new_time);
		}
		else
		{
			this->times_.push_back(new_time);
		}
	}

	template <class StateT> template <typename DurationT>
	void Trajectory<StateT>::insert_point(const StateT& new_point, const std::chrono::duration<int64_t, DurationT>& new_time, int pos)
	{
		this->set_filled();

		auto it_points = this->points_.begin();
		auto it_times = this->times_.begin();
		std::advance(it_points, pos);
		std::advance(it_times, pos);
		
		this->points_.insert(it_points, new_point);

		auto previous_time = this->times_[pos-1];
		this->times_.insert(it_times, previous_time + new_time);

		for(unsigned int i = pos+1; i <= this->points_.size(); i++)
		{
			this->times_[i] += new_time;
		}
	}

	template <class StateT>
	void Trajectory<StateT>::delete_point()
	{
		this->set_filled();
		if(!this->points_.empty())
		{
			this->points_.pop_back();
		}
		if(!this->times_.empty())
		{
			this->times_.pop_back();
		}
	}

	template <class StateT>
	void Trajectory<StateT>::clear()
	{
		this->points_.clear();
		this->times_.clear();
	}

	template <class StateT> 
	inline const std::deque<StateT>& Trajectory<StateT>::get_points() const
	{
		return this->points_;
	}

	template <class StateT> 
	const StateT& Trajectory<StateT>::get_point(unsigned int index) const
	{
		return this->points_[index];
	}

	template <class StateT> 
	StateT& Trajectory<StateT>::get_point(unsigned int index)
	{
		return this->points_[index];
	}

	template <class StateT> 
	inline const std::deque<std::chrono::nanoseconds>& Trajectory<StateT>::get_times() const 
	{
		return this->times_;
	}

	template <class StateT> 
	int Trajectory<StateT>::get_size() const
	{
		return this->points_.size();
	}

	template <class StateT>
	const std::pair<StateT, std::chrono::nanoseconds> Trajectory<StateT>::operator[](unsigned int idx) const
	{
		return std::make_pair(this->points_[idx], this->times_[idx]);
	}

	template <class StateT>
	std::pair<StateT, std::chrono::nanoseconds> Trajectory<StateT>::operator[](unsigned int idx)
	{
		this->set_filled();
		return std::make_pair(this->points_[idx], this->times_[idx]);
	}
}
