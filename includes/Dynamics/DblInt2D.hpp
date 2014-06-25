#ifndef __DBLINT2D_HPP__
#define __DBLINT2D_HPP__

#include <Dynamics/DblInt.hpp>

#define DBLINT2D_X_DIM 4
#define DBLINT2D_U_DIM 2

class DblInt2D : public DblInt<DBLINT2D_X_DIM, DBLINT2D_U_DIM> {
public:
	static const size_t X_DIM;
	static const size_t U_DIM;

	DblInt2D(const BOUNDS& world_bounds) : DblInt("DblInt2D") {
		_x_bounds.resize(DBLINT2D_X_DIM);
		_u_bounds.resize(DBLINT2D_U_DIM);

		// TODO The bounds are a function of the map
		//_x_bounds[0] = std::make_pair(0, 100);
		//_x_bounds[1] = std::make_pair(0, 200); // 100);
		_x_bounds[0] = std::make_pair(world_bounds[0].first, world_bounds[0].second);
		_x_bounds[1] = std::make_pair(world_bounds[1].first, world_bounds[1].second);
		_x_bounds[2] = std::make_pair(-10, 10);
		_x_bounds[3] = std::make_pair(-10, 10);

		_u_bounds[0] = std::make_pair(-10, 10);
		_u_bounds[1] = std::make_pair(-10, 10);
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

	virtual bool computeCostClosedForm(const state& x0, const state& x1, floatX radius, floatX& cost, floatX& tau, state& d_tau) {
		return false;
	}

	virtual void setRadius(const floatX& num_states, floatX& radius) {
	}

	virtual void calc_backward_reachable_bounds(const state& state, const double& radius, BOUNDS& bounds) {
	}

	virtual void calc_forward_reachable_bounds(const state& state, const double& radius, BOUNDS& bounds) {
	}

private:
	BOUNDS _x_bounds;
	BOUNDS _u_bounds;
};

#endif // __DBLINT2D_HPP__