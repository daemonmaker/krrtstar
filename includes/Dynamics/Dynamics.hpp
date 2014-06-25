#ifndef __DYNAMICS_HPP__
#define __DYNAMICS_HPP__

#include <stdint.h>
#include <string>
#include <utility>
#include <complex>
#include <algorithm>
// Fix for stupid windows crap
#undef max

#include <bounds.h>
#include <rpoly.h>
#include "matrix.h"

typedef double floatX;

template <size_t _xDim, size_t _uDim>
class Dynamics {
public:
	typedef Matrix<_xDim> state;
	typedef Matrix<_uDim> control;

	Dynamics(std::string name, const floatX control_penalty, const floatX poly_degree)
		: _name(name), x0(zeros<_xDim,1>()), _control_penalty(control_penalty), _poly_degree(poly_degree)
	{
		_p = new floatX[_poly_degree + 1];
		_zeror = new floatX[_poly_degree];
		_zeroi = new floatX[_poly_degree];
	}

	~Dynamics() {
		delete[] _p, _zeror, _zeroi;
	}

	virtual inline double applyHeuristics(const state& x0, const state& x1) const {
		return 0.0;
	}

	virtual inline Matrix<_xDim> dynamics(floatX step, const Matrix<_xDim>& x, const Matrix<_uDim>& u) const = 0;

	virtual bool compute_cost_closed_form(const state& x0, const state& x1, const floatX& radius, floatX& cost, floatX& tau, state& d_tau) {
		if (applyHeuristics(x0, x1) > radius) return false;

#ifdef _DEBUG_COMPUTE_COST
		cout << "~~~~~~~~~~~~~~~~~~~~~~~ compute_cost_closed_form ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
#endif

		Dynamics::state x1diffbarx0;
		Dynamics::state d;

		_compute_cost_p(x0, x1, _p);

#ifdef _DEBUG_COMPUTE_COST
		cout << "polynomial: " << f << endl;
#endif

		// TODO DWEBB clean this up!
		std::vector<std::complex<double> > complexRoots;
		std::vector<double> realRoots;

		memset(_zeror, 0, sizeof(double)*_poly_degree);
		memset(_zeroi, 0, sizeof(double)*_poly_degree);
		int info[1000];
		int returned_roots = rpoly(_p, _poly_degree, _zeror, _zeroi, info);

		realRoots.clear();
		for (int i = 0; i < returned_roots; i++) {
			if (_zeroi[i] != 0) {
			} else if (_zeror[i] >= 0) {
				realRoots.push_back(_zeror[i]);
			}
		}

		bool result = false;
		double minTau = radius;
		double minCost = radius;
		Dynamics::state mind;
		mind.reset();

		for (size_t i = 0; i < realRoots.size();++i) {

			if (!((realRoots[i] > 0.0) && (realRoots[i] < minCost))) {
				continue;
			}

#ifdef _DEBUG_COMPUTE_COST
			cout << "LOOP ____________________________" << endl;
			cout << "\tx0: " << ~x0 << "\tx1: " << ~x1 << endl;
			cout << "\trealRoots[" << i << "]: " << setprecision(12) <<  realRoots[i] << "\tf(realRoots[" << i << "]: " << f(realRoots[i]) << endl;
#endif


			_compute_cost_d(x0, x1, x1diffbarx0, realRoots[i], d);

#ifdef _DEBUG_COMPUTE_COST
			cout << "\tradius: " << radius << endl;
			cout << "\tx1diffbarx0: " << ~x1diffbarx0;
			cout << "\td: " << ~d;
#endif

			double current = realRoots[i] + tr(~x1diffbarx0*d);
#ifdef _DEBUG_COMPUTE_COST
			std::cout << "\tcurrent: " << current << "\tminCost: " << minCost << std::endl;
#endif
			if ((realRoots[i] > 0.0) && (current < minCost)) {
				mind = d;
				minTau = realRoots[i];
				minCost = current;
				result = true;
			}
		}

		d_tau = mind;
		cost = minCost;
		tau = minTau;

		return result;
	}
	// TODO make these next two function protected
	virtual void _compute_cost_p(const state& x0, const state& x1, floatX *p) const = 0;
	virtual void _compute_cost_d(const state& x0, const state& x1, state& x1diffbarx0, floatX realRoots, state& d) const = 0;

	/*
	bool compute_cost_RK4(D& dynamics, const state& x0, const state& x1, double radius, double& cost, double& tau, state& d_tau) {
		if (dynamics.applyHeuristics(x0, x1) > radius) return false;

		Matrix<X_DIM, X_DIM> G = zeros<X_DIM, X_DIM>();
		state xbar = x0;
		cost = radius;
		double t = 0;

#ifndef FRONT_LOAD_RK4
		linear<X_DIM> diff;
		diff.A = &A;
		diff.c = &c;
		rk<X_DIM, X_DIM, lyapunov<X_DIM> > lyaprk4;

		lyapunov<X_DIM> lyap;
		lyap.A = &A;
		lyap.BRiBt = &BRiBt;
		rk<X_DIM, 1, linear<X_DIM> > linearrk4;

		//lyaprk<X_DIM, X_DIM> lyaprk4;
#endif

		while (t < cost) {
#ifdef INLINE_LYAP
			AG = A*G;
			k_1 = deltaT*(AG + ~AG + BRiBt);
			AG = A*(G + 0.5*k_1);
			k_2 = deltaT*(AG + ~AG + BRiBt);
			AG = A*(G + 0.5*k_2);
			k_3 = deltaT*(AG + ~AG + BRiBt);
			AG = A*(G + k_3);
			k_4 = deltaT*(AG + ~AG + BRiBt);

			G += ((k_1 + k_4) + 2.0*(k_2 + k_3))/6.0;
#else
			lyaprk4.rk4(G, 0, deltaT, lyap);
			//lyaprk4.rk4(G, deltaT);
#endif
			linearrk4.rk4(xbar, 0, deltaT, diff);

			t += deltaT;

			state d = (G%(x1 - xbar));
			double cost_t = t + tr(~(x1 - xbar)*d);
			if (cost_t > t && cost_t < cost) {
				cost = cost_t;
				tau = t;
				d_tau = d;
			}
		}

		bool result = false;
		if (cost < radius) result = true;

#ifdef CHECK_CLOSED_FORM
		double cf_cost;
		double cf_tau;
		state cf_dtau;
		bool cf_result = dynamics.computeCostClosedForm(dynamics, x0, x1, radius, cf_cost, cf_tau, cf_dtau);

		bool resultMatch = (cf_result == result);
		bool costMatch = (cf_cost == cost);
		bool tauMatch = (cf_tau == tau);
		bool dtauMatch = (cf_dtau == d_tau);
		cout << "Summary\n________________________\n\tx0: " << setprecision(10) << ~x0 << "\tx1: " << ~x1 << "\n\tResults (" << result << ", " << cf_result << "): " << resultMatch << "\n\tCosts (" << cost << ", " << cf_cost << "): " << costMatch << "\n\tTaus (" << tau << ", " << cf_tau << "): " << tauMatch << "\n\tD_taus (" << ~d_tau << ", " << ~cf_dtau << "): " << dtauMatch << endl;

		if (!resultMatch || !costMatch || !tauMatch || !dtauMatch) {
			int k;
			cin >> k;
		}
#endif

		return result;
	}
	*/

	virtual void setRadius(const floatX& num_states, floatX& radius) = 0;
	virtual void calc_backward_reachable_bounds(const state& state, const double& radius, BOUNDS& bounds) = 0;
	virtual void calc_forward_reachable_bounds(const state& state, const double& radius, BOUNDS& bounds) = 0;

protected:
	floatX _control_penalty;

	floatX computeHeuristic(const floatX x0, const floatX x1, const std::pair<floatX, floatX>& bounds) const {
		floatX tmp = x1 - x0;
		return std::max(tmp/bounds.first, tmp/bounds.second);
	}

private:
	std::string _name;

	Matrix<_xDim> x0;

	floatX _poly_degree;
	floatX* _p;
	floatX* _zeror;
	floatX* _zeroi;
};

#endif // __DYNAMICS_HPP__