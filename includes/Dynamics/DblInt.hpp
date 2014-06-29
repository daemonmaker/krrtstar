#ifndef __DBLINT_HPP__
#define __DBLINT_HPP__

#include <Dynamics/Dynamics.hpp>

/**
* This class is intended to be the basic template for double integrator models of any size.
* It is not intended to stand on it's own and therefore should not be directly instatiated.
*/
template <size_t _xDim, size_t _uDim>
class DblInt : public Dynamics<_xDim, _uDim> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
	DblInt(std::string name, const floatX control_penalty, const floatX poly_degree, const floatX radius_multiplier, const floatX radius)
		: Dynamics(name, control_penalty, poly_degree, radius_multiplier, radius)
	{}

};

#endif // __DBLINT_HPP__