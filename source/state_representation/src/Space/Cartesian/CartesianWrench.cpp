#include "state_representation/Space/Cartesian/CartesianWrench.hpp"
#include "state_representation/Exceptions/IncompatibleStatesException.hpp"
#include "state_representation/Exceptions/IncompatibleReferenceFramesException.hpp"
#include "state_representation/Exceptions/EmptyStateException.hpp"

using namespace StateRepresentation::Exceptions;

namespace StateRepresentation 
{
	CartesianWrench::CartesianWrench()
	{}

	CartesianWrench::CartesianWrench(const std::string& name, const std::string& reference):
	CartesianState(name, reference)
	{}

	CartesianWrench::CartesianWrench(const std::string& name, const Eigen::Vector3d& force, const std::string& reference):
	CartesianState(name, reference)
	{
		this->set_force(force);
	}

	CartesianWrench::CartesianWrench(const std::string& name, const Eigen::Vector3d& force, const Eigen::Vector3d& torque, const std::string& reference):
	CartesianState(name, reference)
	{
		this->set_force(force);
		this->set_torque(torque);
	}

	CartesianWrench::CartesianWrench(const std::string& name, const Eigen::Matrix<double, 6, 1>& wrench, const std::string& reference):
	CartesianState(name, reference)
	{
		this->set_wrench(wrench);
	}

	CartesianWrench::CartesianWrench(const CartesianWrench& wrench):
	CartesianState(wrench)
	{}

	CartesianWrench::CartesianWrench(const CartesianState& state):
	CartesianState(state)
	{}

	CartesianWrench& CartesianWrench::operator=(const Eigen::Matrix<double, 6, 1>& wrench)
	{
		this->set_wrench(wrench);
		return (*this);
	}

	CartesianWrench& CartesianWrench::operator+=(const Eigen::Matrix<double, 6, 1>& vector)
	{
		if(this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
		this->set_wrench(this->get_wrench() + vector);
		return (*this);
	}

	CartesianWrench& CartesianWrench::operator+=(const CartesianWrench& wrench)
	{
		// sanity check
		if(this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
		if(wrench.is_empty()) throw EmptyStateException(wrench.get_name() + " state is empty");
		if(!(this->get_reference_frame() == wrench.get_reference_frame())) throw IncompatibleReferenceFramesException("The two states do not have the same reference frame");
		// operation
		this->set_force(this->get_force() + wrench.get_force());
		this->set_torque(this->get_torque() + wrench.get_torque());
		return (*this);
	}

	const CartesianWrench CartesianWrench::operator+(const Eigen::Matrix<double, 6, 1>& vector) const
	{
		CartesianWrench result(*this);
		result += vector;
		return result;
	}

	const CartesianWrench CartesianWrench::operator+(const CartesianWrench& wrench) const
	{
		CartesianWrench result(*this);
		result += wrench;
		return result;
	}

	CartesianWrench& CartesianWrench::operator-=(const Eigen::Matrix<double, 6, 1>& vector)
	{
		if(this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
		this->set_wrench(this->get_wrench() - vector);
		return (*this);
	}

	CartesianWrench& CartesianWrench::operator-=(const CartesianWrench& wrench)
	{
		// sanity check
		if(this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
		if(wrench.is_empty()) throw EmptyStateException(wrench.get_name() + " state is empty");
		if(!(this->get_reference_frame() == wrench.get_reference_frame())) throw IncompatibleReferenceFramesException("The two states do not have the same reference frame");
		// operation
		this->set_force(this->get_force() - wrench.get_force());
		this->set_torque(this->get_torque() - wrench.get_torque());
		return (*this);
	}

	const CartesianWrench CartesianWrench::operator-(const Eigen::Matrix<double, 6, 1>& vector) const
	{
		CartesianWrench result(*this);
		result -= vector;
		return result;
	}

	const CartesianWrench CartesianWrench::operator-(const CartesianWrench& wrench) const
	{
		CartesianWrench result(*this);
		result -= wrench;
		return result;
	}

	void CartesianWrench::operator=(const CartesianState& state)
	{
		// sanity check
		if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty");
		// operation
		this->set_name(state.get_name());
		this->set_reference_frame(state.get_reference_frame());
		this->set_force(state.get_force());
		this->set_torque(state.get_torque());
	}

	CartesianWrench& CartesianWrench::operator*=(double lambda)
	{
		// sanity check
		if(this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
		// operation
		this->set_force(lambda * this->get_force());
		this->set_torque(lambda * this->get_torque());
		return (*this);
	}

	const CartesianWrench CartesianWrench::operator*(double lambda) const
	{
		CartesianWrench result(*this);
		result *= lambda;
		return result;
	}

	void CartesianWrench::clamp(double max_force, double max_torque, double noise_ratio)
	{	
		if(noise_ratio != 0)
		{	
			// substract the noise ratio to both velocities
			this->set_force(this->get_force() - noise_ratio * this->get_force().normalized());
			this->set_torque(this->get_torque() - noise_ratio * this->get_torque().normalized());
			// apply a deadzone
			if(this->get_force().norm() < noise_ratio) this->set_force(Eigen::Vector3d::Zero());
			if(this->get_torque().norm() < noise_ratio) this->set_torque(Eigen::Vector3d::Zero()); 	
		} 
		// clamp the velocities to their maximum amplitude provided
		if(this->get_force().norm() > max_force) this->set_force(max_force * this->get_force().normalized());
		if(this->get_torque().norm() > max_torque) this->set_torque(max_torque * this->get_torque().normalized());
	}

	const CartesianWrench CartesianWrench::clamped(double max_force, double max_torque, double noise_ratio) const
	{
		CartesianWrench result(*this);
		result.clamp(max_force, max_torque, noise_ratio);
		return result;
	}

	const CartesianWrench CartesianWrench::copy() const
	{
		CartesianWrench result(*this);
		return result;
	}

	const Eigen::Array<double, 6, 1> CartesianWrench::array() const
	{
		return this->get_wrench().array();
	}

	std::ostream& operator<<(std::ostream& os, const CartesianWrench& wrench) 
	{
		if(wrench.is_empty())
		{
			os << "Empty CartesianWrench";
		}
		else
		{
			os << wrench.get_name() << " CartesianWrench expressed in " << wrench.get_reference_frame() << " frame" << std::endl;
	  		os << "force: (" << wrench.get_force()(0) << ", ";
	  		os << wrench.get_force()(1) << ", ";
	  		os << wrench.get_force()(2) << ")" << std::endl;
	  		os << "torque: (" << wrench.get_torque()(0) << ", ";
	  		os << wrench.get_torque()(1) << ", ";
	  		os << wrench.get_torque()(2) << ")";
	  	}
  		return os;
	}

	const CartesianWrench operator+(const Eigen::Matrix<double, 6, 1>& vector, const CartesianWrench& wrench)
	{
		return wrench + vector;
	}

	const CartesianWrench operator-(const Eigen::Matrix<double, 6, 1>& vector, const CartesianWrench& wrench)
	{
		return vector + (-1) * wrench;
	}

	const CartesianWrench operator*(double lambda, const CartesianWrench& wrench)
	{
		return wrench * lambda;
	}
}
