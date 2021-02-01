#include "dynamical_systems/Linear.hpp"

namespace DynamicalSystems
{
	template<>
	inline void Linear<StateRepresentation::CartesianState>::set_gain(double iso_gain)
	{
		this->gain_->set_value(iso_gain * Eigen::MatrixXd::Identity(6, 6));
	}


	template<>
	inline void Linear<StateRepresentation::JointState>::set_gain(double iso_gain)
	{
		int nb_joints = this->get_attractor().get_size();
		this->gain_->set_value(iso_gain * Eigen::MatrixXd::Identity(nb_joints, nb_joints));
	}

	template<>
	inline void Linear<StateRepresentation::CartesianState>::set_gain(const std::vector<double>& diagonal_coefficients)
	{
		if(diagonal_coefficients.size() != 6)
		{
			throw Exceptions::IncompatibleSizeException("The provided diagonal coefficients do not correspond to the expected size of 6 elements");
		}
		Eigen::VectorXd diagonal = Eigen::VectorXd::Map(diagonal_coefficients.data(), 6);
		this->gain_->set_value(diagonal.asDiagonal());
	}

	template<>
	inline void Linear<StateRepresentation::JointState>::set_gain(const std::vector<double>& diagonal_coefficients)
	{
		size_t nb_joints = this->get_attractor().get_size();
		if(diagonal_coefficients.size() != nb_joints)
		{
			throw Exceptions::IncompatibleSizeException("The provided diagonal coefficients do not correspond to the expected size of " + std::to_string(nb_joints) + " elements");
		}
		Eigen::VectorXd diagonal = Eigen::VectorXd::Map(diagonal_coefficients.data(), nb_joints);
		this->gain_->set_value(diagonal.asDiagonal());
	}

	template<>
	inline void Linear<StateRepresentation::CartesianState>::set_gain(const Eigen::MatrixXd& gain_matrix)
	{
		if(gain_matrix.rows() != 6 && gain_matrix.cols() != 6)
		{
			throw Exceptions::IncompatibleSizeException("The provided gain matrix do not have the expected size of 6x6 elements");
		}
		this->gain_->set_value(gain_matrix);
	}

	template<>
	inline void Linear<StateRepresentation::JointState>::set_gain(const Eigen::MatrixXd& gain_matrix)
	{
		int nb_joints = this->get_attractor().get_size();
		if(gain_matrix.rows() != nb_joints && gain_matrix.cols() != nb_joints)
		{
			throw Exceptions::IncompatibleSizeException("The provided gain matrix do not have the expected size of " + std::to_string(nb_joints) + "x" + std::to_string(nb_joints) + " elements");
		}
		this->gain_->set_value(gain_matrix);
	}

	template<>
	Linear<StateRepresentation::CartesianState>::Linear(const StateRepresentation::CartesianState& attractor, double iso_gain):
	DynamicalSystem<StateRepresentation::CartesianState>(StateRepresentation::CartesianPose::Identity(attractor.get_reference_frame())),
	attractor_(std::make_shared<StateRepresentation::Parameter<StateRepresentation::CartesianState>>(StateRepresentation::Parameter<StateRepresentation::CartesianPose>("attractor", attractor))),
	gain_(std::make_shared<StateRepresentation::Parameter<Eigen::MatrixXd>>("gain"))
	{
		this->set_gain(iso_gain);
	}

	template<>
	Linear<StateRepresentation::JointState>::Linear(const StateRepresentation::JointState& attractor, double iso_gain):
	DynamicalSystem<StateRepresentation::JointState>(),
	attractor_(std::make_shared<StateRepresentation::Parameter<StateRepresentation::JointState>>(StateRepresentation::Parameter<StateRepresentation::JointPositions>("attractor", attractor))),
	gain_(std::make_shared<StateRepresentation::Parameter<Eigen::MatrixXd>>("gain"))
	{
		this->set_gain(iso_gain);
	}

	template<>
	Linear<StateRepresentation::CartesianState>::Linear(const StateRepresentation::CartesianState& attractor, const std::vector<double>& diagonal_coefficients):
	DynamicalSystem<StateRepresentation::CartesianState>(StateRepresentation::CartesianPose::Identity(attractor.get_reference_frame())),
	attractor_(std::make_shared<StateRepresentation::Parameter<StateRepresentation::CartesianState>>(StateRepresentation::Parameter<StateRepresentation::CartesianPose>("attractor", attractor))),
	gain_(std::make_shared<StateRepresentation::Parameter<Eigen::MatrixXd>>("gain"))
	{
		this->set_gain(diagonal_coefficients);
	}

	template<>
	Linear<StateRepresentation::JointState>::Linear(const StateRepresentation::JointState& attractor, const std::vector<double>& diagonal_coefficients):
	DynamicalSystem<StateRepresentation::JointState>(),
	attractor_(std::make_shared<StateRepresentation::Parameter<StateRepresentation::JointState>>(StateRepresentation::Parameter<StateRepresentation::JointPositions>("attractor", attractor))),
	gain_(std::make_shared<StateRepresentation::Parameter<Eigen::MatrixXd>>("gain"))
	{
		this->set_gain(diagonal_coefficients);
	}

	template<>
	Linear<StateRepresentation::CartesianState>::Linear(const StateRepresentation::CartesianState& attractor, const Eigen::MatrixXd& gain_matrix):
	DynamicalSystem<StateRepresentation::CartesianState>(StateRepresentation::CartesianPose::Identity(attractor.get_reference_frame())),
	attractor_(std::make_shared<StateRepresentation::Parameter<StateRepresentation::CartesianState>>(StateRepresentation::Parameter<StateRepresentation::CartesianPose>("attractor", attractor))),
	gain_(std::make_shared<StateRepresentation::Parameter<Eigen::MatrixXd>>("gain"))
	{
		this->set_gain(gain_matrix);
	}

	template<>
	Linear<StateRepresentation::JointState>::Linear(const StateRepresentation::JointState& attractor, const Eigen::MatrixXd& gain_matrix):
	DynamicalSystem<StateRepresentation::JointState>(),
	attractor_(std::make_shared<StateRepresentation::Parameter<StateRepresentation::JointState>>(StateRepresentation::Parameter<StateRepresentation::JointPositions>("attractor", attractor))),
	gain_(std::make_shared<StateRepresentation::Parameter<Eigen::MatrixXd>>("gain"))
	{
		this->set_gain(gain_matrix);
	}

	template<>
	const StateRepresentation::CartesianState Linear<StateRepresentation::CartesianState>::compute_dynamics(const StateRepresentation::CartesianState& state) const
	{
		StateRepresentation::CartesianTwist twist = static_cast<const StateRepresentation::CartesianPose&>(this->get_attractor()) - static_cast<const StateRepresentation::CartesianPose&>(state);
		twist *= this->get_gain();
		return twist;
	}

	template<>
	const StateRepresentation::JointState Linear<StateRepresentation::JointState>::compute_dynamics(const StateRepresentation::JointState& state) const
	{
		StateRepresentation::JointState positions = - this->get_gain() * (state - this->get_attractor());
		StateRepresentation::JointState velocities(state.get_name(), state.get_names());
		velocities.set_velocities(positions.get_positions());
		return velocities;
	}
}