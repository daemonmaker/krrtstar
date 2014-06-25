#ifndef __DBLINT2D_HPP__
#define __DBLINT2D_HPP__

#include <Dynamics/DblInt.hpp>

#define DBLINT2D_X_DIM 4
#define DBLINT2D_U_DIM 2

/**
* TOOD
* - Standardize function naming convention to use underscores
* - Move random state generation into the dynamics. Some worlds have complicated position bounds so randomly generating a state vector will need to be a function of the world (i.e. the world will randomly generate the position elements while the dynamics will randomly generate the rest).
*/

class DblInt2D : public DblInt<DBLINT2D_X_DIM, DBLINT2D_U_DIM> {
public:
	static const size_t X_DIM;
	static const size_t U_DIM;

	DblInt2D(const floatX control_penalty, const floatX poly_degree, const BOUNDS& world_bounds) : DblInt("DblInt2D", control_penalty, poly_degree) {
		_x_bounds.resize(DBLINT2D_X_DIM);
		_u_bounds.resize(DBLINT2D_U_DIM);

		// TODO The bounds are a function of the map
		_x_bounds[0] = std::make_pair(world_bounds[0].first, world_bounds[0].second);
		_x_bounds[1] = std::make_pair(world_bounds[1].first, world_bounds[1].second);
		_x_bounds[2] = std::make_pair(-10, 10);
		_x_bounds[3] = std::make_pair(-10, 10);

		_u_bounds[0] = std::make_pair(-10, 10);
		_u_bounds[1] = std::make_pair(-10, 10);

		_state_space_volume = 1;
		for (BOUNDS::iterator p = _x_bounds.begin(); p != _x_bounds.end(); p++) {
			_state_space_volume *= (p->second - p->first);
		}
	}

	virtual inline floatX applyHeuristics(const state& x0, const state& x1) const {
		floatX cost = 0.0;

		floatX cost1 = std::max(computeHeuristic(x0[0], x1[0], _x_bounds[2]), computeHeuristic(x0[1], x1[1], _x_bounds[3]));
		floatX cost2 = std::max(computeHeuristic(x0[2], x1[2], _u_bounds[0]), computeHeuristic(x0[3], x1[3], _u_bounds[1]));

		cost = std::max(cost1, cost2);

		return cost;
	}

	virtual inline Matrix<DBLINT2D_X_DIM> dynamics(floatX step, const Matrix<DBLINT2D_X_DIM>& x, const Matrix<DBLINT2D_U_DIM>& u) const {
		Matrix<DBLINT2D_X_DIM> xNew;

		xNew[0] = x[0] + x[2]*step + u[0]*step*step;
		xNew[1] = x[1] + x[3]*step + u[1]*step*step;
		xNew[2] = x[2] + u[0]*step;
		xNew[3] = x[3] + u[1]*step;

		return xNew;
	}

	virtual void _compute_cost_p(const Dynamics::state& x0, const Dynamics::state& x1, floatX *p) const {
		p[0] = 1;
		p[1] = 0;
		p[2] = -4.0*_control_penalty*x1[3]*x1[3]-4.0*_control_penalty*x0[3]*x0[3]-4.0*_control_penalty*x1[2]*x1[2]-4.0*_control_penalty*x1[3]*x0[3]-4.0*_control_penalty*x1[2]*x0[2]-4.0*_control_penalty*x0[2]*x0[2];
		p[3] = 24.0*_control_penalty*x1[1]*x0[3]+24.0*_control_penalty*x1[0]*x0[2]-24.0*_control_penalty*x0[0]*x0[2]-24.0*_control_penalty*x0[1]*x0[3]+24.0*_control_penalty*x1[2]*x1[0]-24.0*_control_penalty*x1[2]*x0[0]+24.0*_control_penalty*x1[3]*x1[1]-24.0*_control_penalty*x1[3]*x0[1];
		p[4] = -36.0*_control_penalty*x1[0]*x1[0]-36.0*_control_penalty*x0[0]*x0[0]-36.0*_control_penalty*x1[1]*x1[1]-36.0*_control_penalty*x0[1]*x0[1]+72.0*_control_penalty*x1[0]*x0[0]+72.0*_control_penalty*x1[1]*x0[1];
	}
	
	virtual void _compute_cost_d(const Dynamics::state& x0, const Dynamics::state& x1, Dynamics::state& x1diffbarx0, floatX realRoot, Dynamics::state& d) const {
		double t7 = realRoot*realRoot;
		double t10 = _control_penalty/t7/realRoot;
		double t14 = 1/t7*_control_penalty;
		double t26 = _control_penalty/realRoot;
		x1diffbarx0[0] = x1[0]-x0[0]-realRoot*x0[2];
		x1diffbarx0[1] = x1[1]-x0[1]-realRoot*x0[3];
		x1diffbarx0[2] = x1[2]-x0[2];
		x1diffbarx0[3] = x1[3]-x0[3];
		d[0] = 12.0*t10*x1diffbarx0[0]-6.0*t14*x1diffbarx0[2];
		d[1] = 12.0*t10*x1diffbarx0[1]-6.0*t14*x1diffbarx0[3];
		d[2] = -6.0*t14*x1diffbarx0[0]+4.0*t26*x1diffbarx0[2];
		d[3] = -6.0*t14*x1diffbarx0[1]+4.0*t26*x1diffbarx0[3];
	}

	virtual void setRadius(const floatX& num_states, floatX& radius) {
	}

	virtual void calc_backward_reachable_bounds(const state& state, const double& radius, BOUNDS& bounds) {
	}

	virtual void calc_forward_reachable_bounds(const state& state, const double& radius, BOUNDS& bounds) {
	}

	void set_x_bounds(const BOUNDS& x_bounds) {
		_x_bounds = x_bounds;
	}

	BOUNDS get_x_bounds() const {
		return _x_bounds;
	}

	void set_u_bounds(const BOUNDS& u_bounds) {
		_u_bounds = u_bounds;
	}

	BOUNDS get_u_bounds() const {
		return _u_bounds;
	}

	floatX get_state_space_volume() const {
		return _state_space_volume;
	}

private:
	BOUNDS _x_bounds;
	BOUNDS _u_bounds;
	floatX _state_space_volume;
};

#endif // __DBLINT2D_HPP__