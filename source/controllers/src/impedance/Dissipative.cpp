#include "controllers/impedance/Dissipative.hpp"

using namespace state_representation;

namespace controllers::impedance {
template<class S>
Dissipative<S>::Dissipative(const ComputationalSpaceType& computational_space):
    Dissipative<S>(computational_space, 6) {}

template Dissipative<CartesianState>::Dissipative(const ComputationalSpaceType&);

template<class S>
Dissipative<S>::Dissipative(unsigned int nb_dimensions):
    Dissipative<S>(ComputationalSpaceType::FULL, nb_dimensions) {}

template Dissipative<JointState>::Dissipative(unsigned int);
}// namespace controllers