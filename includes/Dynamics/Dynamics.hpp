#ifndef __DYNAMICS_HPP__
#define __DYNAMICS_HPP__

#include <stdint.h>
#include <string>
#include <utility>
#include <algorithm>
// Fix for stupid windows crap
#undef max

#include <bounds.h>
#include "matrix.h"

typedef double floatX;

template <size_t _xDim, size_t _uDim>
class Dynamics {
public:
	typedef Matrix<_xDim> state;
	typedef Matrix<_uDim> control;

	Dynamics(std::string name)
		: _name(name), x0(zeros<_xDim,1>())
	{}

	virtual inline double applyHeuristics(const state& x0, const state& x1) const {
		return 0.0;
	}

	virtual inline Matrix<_xDim> dynamics(floatX step, const Matrix<_xDim>& x, const Matrix<_uDim>& u) const = 0;

	virtual bool computeCostClosedForm(const state& x0, const state& x1, floatX radius, floatX& cost, floatX& tau, state& d_tau) = 0;
	virtual void setRadius(const floatX& num_states, floatX& radius) = 0;
	virtual void calc_backward_reachable_bounds(const state& state, const double& radius, BOUNDS& bounds) = 0;
	virtual void calc_forward_reachable_bounds(const state& state, const double& radius, BOUNDS& bounds) = 0;

protected:
	floatX computeHeuristic(const floatX x0, const floatX x1, const std::pair<floatX, floatX>& bounds) const {
		floatX tmp = x1 - x0;
		return std::max(tmp/bounds.first, tmp/bounds.second);
	}

private:
	std::string _name;

	Matrix<_xDim> x0;
};

#endif // __DYNAMICS_HPP__