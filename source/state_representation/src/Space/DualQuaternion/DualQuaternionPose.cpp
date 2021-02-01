#include "state_representation/Space/DualQuaternion/DualQuaternionPose.hpp"

namespace StateRepresentation 
{
	DualQuaternionPose::DualQuaternionPose(const std::string& name, const std::string& reference):
	DualQuaternionState(name, reference)
	{
		this->initialize();
	}

	DualQuaternionPose::DualQuaternionPose(const DualQuaternionPose& state):
	DualQuaternionState(state), position(state.position)
	{}

	DualQuaternionPose::DualQuaternionPose(const DualQuaternionState& state):
	DualQuaternionState(state)
	{
		this->set_position(this->get_dual());
	}

	DualQuaternionPose::DualQuaternionPose(const std::string& name, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, const std::string& reference):
	DualQuaternionState(name, reference)
	{
		this->initialize();
		this->set_position(position);
		this->set_orientation(orientation);
	}

	const DualQuaternionPose DualQuaternionPose::conjugate() const
	{
		return static_cast<const DualQuaternionState&>(*this).conjugate();
	}

	const DualQuaternionPose DualQuaternionPose::inverse() const
	{
		DualQuaternionPose result(*this);
		// inverse name and reference frame
		std::string ref = this->get_reference_frame();
		std::string name = this->get_name();
		result.set_reference_frame(name);
		result.set_name(ref);
		return result.conjugate();
	}

	DualQuaternionPose& DualQuaternionPose::operator*=(const DualQuaternionPose& q)
	{
		Eigen::Quaterniond primary = this->get_primary() * q.get_primary();
		Eigen::Quaterniond dual = Eigen::Quaterniond((this->get_primary() * q.get_dual()).coeffs() + (this->get_dual() * q.get_primary()).coeffs());
		this->set_primary(primary);
		this->set_dual(dual);
		this->set_position(this->get_dual());
		this->set_name(q.get_name());
		return (*this);
	}

    const DualQuaternionPose DualQuaternionPose::operator*(const DualQuaternionPose& p) const
	{
		DualQuaternionState result(*this);
		result *= p;
		return result;
	}

	DualQuaternionPose& DualQuaternionPose::operator*=(const DualQuaternionState& s)
	{
        // Why two functions DualQuaternionState/DualQuaternionPose and referenced via base-pointer?
		Eigen::Quaterniond primary = this->get_primary() * s.get_primary();
		Eigen::Quaterniond dual = Eigen::Quaterniond((this->get_primary() * s.get_dual()).coeffs() + (this->get_dual() * s.get_primary()).coeffs());
		this->set_primary(primary);
		this->set_dual(dual);
		this->set_position(this->get_dual());
		this->set_name(s.get_name());
		return (*this);
	}

	const DualQuaternionPose DualQuaternionPose::operator*(const DualQuaternionState& s) const
	{
		DualQuaternionState result(*this);
		result *= s;
		return result;
	}

	void DualQuaternionPose::operator=(const DualQuaternionState& s)
	{
		this->set_primary(s.get_primary());
		this->set_dual(s.get_dual());
		this->set_position(this->get_dual());
		this->set_name(s.get_name());
		this->set_reference_frame(s.get_reference_frame());
	}

	void DualQuaternionPose::initialize()
	{
		this->DualQuaternionState::initialize();
		this->position = Eigen::Vector3d(0,0,0);
	}

	const DualQuaternionPose DualQuaternionPose::copy() const
	{
		DualQuaternionPose result(*this);
		return result;
	}

	const DualQuaternionState log(const DualQuaternionPose& state)
	{
		DualQuaternionState result(state.get_name(), state.get_reference_frame());
		Eigen::AngleAxisd axis_angle(state.get_primary());
		Eigen::Vector3d rotation = (axis_angle.angle() * axis_angle.axis()) / 2;
		Eigen::Vector3d position = state.get_position() / 2;
		result.set_primary(Eigen::Quaterniond(0, rotation(0), rotation(1), rotation(2)));
		result.set_dual(Eigen::Quaterniond(0, position(0), position(1), position(2)));
		return result;
	}

	std::ostream& operator<<(std::ostream& os, const DualQuaternionPose& state) 
	{ 
		if(state.is_empty())
		{
			os << "Empty DualQuaternionPose";
		}
		else
		{
			os << state.get_name() << " DualQuaternionPose expressed in " << state.get_reference_frame() << " frame" << std::endl;
	  		os << "primary/orientation: (" <<state.get_primary().w() << ", ";
	  		os << state.get_primary().x() << ", ";
	  		os << state.get_primary().y() << ", ";
	  		os << state.get_primary().z() << ")";
	  		Eigen::AngleAxisd axis_angle(state.get_primary());
	  		os << " <=> theta: " << axis_angle.angle() << ", ";
	  		os << "axis: (" << axis_angle.axis()(0) << ", ";
	  		os << axis_angle.axis()(1) << ", ";
	  		os << axis_angle.axis()(2) << ")" << std::endl;
	  		os << "dual: (" <<state.get_dual().w() << ", ";
	  		os << state.get_dual().x() << ", ";
	  		os << state.get_dual().y() << ", ";
	  		os << state.get_dual().z() << ")";
	  		Eigen::Vector3d position = state.get_position();
	  		os << " => position: (" << position(0) << ", ";
	  		os << position(1) << ", ";
	  		os << position(2) << ")";
	  	}
  		return os;
	}
}
