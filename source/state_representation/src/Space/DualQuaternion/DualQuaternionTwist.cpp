#include "state_representation/Space/DualQuaternion/DualQuaternionTwist.hpp"

namespace StateRepresentation 
{
	DualQuaternionTwist::DualQuaternionTwist(const std::string& name, const std::string& reference):
	DualQuaternionState(name, reference)
	{
		this->initialize();
	}

	DualQuaternionTwist::DualQuaternionTwist(const DualQuaternionTwist& state):
	DualQuaternionState(state), position(state.position), linear_velocity(state.linear_velocity)
	{}

	DualQuaternionTwist::DualQuaternionTwist(const std::string& name, const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& angular_velocity, const Eigen::Vector3d& position, const std::string& reference):
	DualQuaternionState(name, reference)
	{
		this->initialize();
		this->set_linear_velocity(linear_velocity);
		this->set_angular_velocity(angular_velocity);
		this->set_position(position);
	}

	void DualQuaternionTwist::operator=(const DualQuaternionState &q) 
	{
		this->set_primary(q.get_primary());
		this->set_dual(q.get_dual());
		this->linear_velocity = this->get_dual().vec() - this->get_position().cross(this->get_angular_velocity());
	}

	void DualQuaternionTwist::initialize()
	{
		this->DualQuaternionState::initialize();
		this->position = Eigen::Vector3d(0,0,0);
		this->linear_velocity = Eigen::Vector3d(0,0,0);
	}

	const DualQuaternionTwist DualQuaternionTwist::copy() const
	{
		DualQuaternionTwist result(*this);
		return result;
	}

	std::ostream& operator<<(std::ostream& os, const DualQuaternionTwist& state) 
	{ 
		if(state.is_empty())
		{
			os << "Empty DualQuaternionTwist";
		}
		else
		{
			os << state.get_name() << " DualQuaternionTwist state expressed in " << state.get_reference_frame() << " frame" << std::endl;
	  		os << "primary: (" <<state.get_primary().w() << ", ";
	  		os << state.get_primary().x() << ", ";
	  		os << state.get_primary().y() << ", ";
	  		os << state.get_primary().z() << ")";
	  		os << " <=> angular_velocity: (" << state.get_primary().x() << ", ";
	  		os << state.get_primary().y() << ", ";
	  		os << state.get_primary().z() << ")" << std::endl;
	  		os << "dual: (" <<state.get_dual().w() << ", ";
	  		os << state.get_dual().x() << ", ";
	  		os << state.get_dual().y() << ", ";
	  		os << state.get_dual().z() << ")";
	  		Eigen::Vector3d linear_velocity = state.get_linear_velocity();
	  		os << " => linear_velocity: (" << linear_velocity(0) << ", ";
	  		os << linear_velocity(1) << ", ";
	  		os << linear_velocity(2) << ")";
	  		Eigen::Vector3d position = state.get_position();
	  		os << " & position: (" << position(0) << ", ";
	  		os << position(1) << ", ";
	  		os << position(2) << ")";
	  	}
  		return os;
	}
}