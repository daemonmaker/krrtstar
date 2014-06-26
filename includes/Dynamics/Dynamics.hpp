#ifndef __DYNAMICS_HPP__
#define __DYNAMICS_HPP__

#define _USE_MATH_DEFINES

#include <stdint.h>
#include <string>
#include <vector>
#include <utility>
#include <complex>
#include <cmath>
#include <algorithm>
// Fix for stupid windows crap
#undef max
#undef min

#include <bounds.hpp>
#include <utilities.hpp>
#include <rpoly.h>
#include <configuration.h>
#include "matrix.h"

//#define _DEBUG_COMPUTE_COST

typedef double floatX;

template <size_t _xDim, size_t _uDim>
class Dynamics {
public:
	typedef Matrix<_xDim> state;
	typedef Matrix<_uDim> control;

	typedef std::pair<double, state> state_time_t;
	typedef std::vector< state_time_t > state_time_list_t;

	Dynamics(std::string name, const floatX control_penalty, const floatX poly_degree, const floatX radius_multiplier, const floatX radius)
		: _name(name), _im(0,1), _control_penalty(control_penalty), _poly_degree(poly_degree), _radius_multiplier(radius_multiplier), _radius(radius)
	{
		_p = new floatX[_poly_degree + 1];
		_zeror = new floatX[_poly_degree];
		_zeroi = new floatX[_poly_degree];

		_chi = new Matrix<2*_xDim>();

		calc_unit_sphere_volume();
	}

	~Dynamics() {
		delete[] _p, _zeror, _zeroi;
	}

	inline floatX get_unit_sphere_volume() const {
		return _unit_sphere_volume;
	}

	inline floatX get_state_space_volume() const {
		return _state_space_volume;
	}

	virtual inline void rand_state(state& v) = 0;

	virtual inline double applyHeuristics(const state& x0, const state& x1) const {
		return 0.0;
	}

	virtual inline Matrix<_xDim> dynamics(floatX step, const Matrix<_xDim>& x, const Matrix<_uDim>& u) const = 0;

	virtual inline bool compute_cost_closed_form(const state& x0, const state& x1, floatX& cost, floatX& tau, state& d_tau) {
		if (applyHeuristics(x0, x1) > _radius) return false;

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
		double minTau = _radius;
		double minCost = _radius;
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
			cout << "\tradius: " << _radius << endl;
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

	bool compute_cost_RK4(const state& x0, const state& x1, double& cost, double& tau, state& d_tau) {
		if (applyHeuristics(x0, x1) > _radius) return false;

		Matrix<X_DIM, X_DIM> G = zeros<X_DIM, X_DIM>();
		state xbar = x0;
		cost = _radius;
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
		if (cost < _radius) result = true;

#ifdef CHECK_CLOSED_FORM
		double cf_cost;
		double cf_tau;
		state cf_dtau;
		bool cf_result = computeCostClosedForm(x0, x1, cf_cost, cf_tau, cf_dtau);

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

	bool check_path_closed_form(const state& x0, const state& x1, const floatX tau, const state& d_tau, const bool plot, state_time_list_t* vis) {
		floatX t;
		floatX bound = (plot ? deltaT * 8 : deltaT);
		int numPoints = ceil(tau / bound);
		int step = 1;
		while (step < numPoints) step *= 2;

		for ( ; step > 1; step /= 2) {
			for (int i = step / 2; i < numPoints; i += step) {
				t = (tau*i)/numPoints;

				_check_path_chi(x0, x1, t, tau, d_tau);
				//double t1 = t-tau;
				//double t3 = tau*tau;
				//double t7 = t*t;
				//double t12 = 1/control_penalty;
				//double t13 = (t3*tau-3.0*t3*t+3.0*tau*t7-t7*t)*t12;
				//double t19 = (t3-2.0*tau*t+t7)*t12;
				//double t31 = -t1*t12;
				//chi[0] = x1[0]+t1*x1[2]+t13*d_tau[0]/6.0+t19*d_tau[2]/2.0;
				//chi[1] = x1[1]+t1*x1[3]+t13*d_tau[1]/6.0+t19*d_tau[3]/2.0;
				//chi[2] = x1[2]-t19*d_tau[0]/2.0-t31*d_tau[2];
				//chi[3] = x1[3]-t19*d_tau[1]/2.0-t31*d_tau[3];
				//chi[4] = d_tau[0];
				//chi[5] = d_tau[1];
				//chi[6] = -t1*d_tau[0]+d_tau[2];
				//chi[7] = -t1*d_tau[1]+d_tau[3];

				state x = _chi->subMatrix<_xDim,1>(0,0);
				control u = R%(~B*_chi->subMatrix<_xDim,1>(_xDim,0));

				if (!(plot || vis) && !check_state(x, u)) {
					return false;
				}

				if (plot || vis) {
					floatX x_coord = 0.0, y_coord = 0.0, z_coord = 0.0;

					//x_coord = x[0];
					//y_coord = 0;
					//z_coord = x[1];
					_check_path_coords(x, &x_coord, &y_coord, &z_coord);

					if (plot) {
						fwrite((const void *)&t, sizeof(double), 1, path_log);
						fwrite(x._elems, sizeof(double), (x.numRows())*(x.numColumns()), path_log);
						CAL_CreateSphere(solution_group, 2*NODE_SIZE, x_coord, y_coord, z_coord);
					}

					if (vis) {
						vis->push_back(make_pair(t, x));
					}
				}
			}
		}

		return true;
	}

	bool check_path_RK4(const state& x0, const state& x1, const double tau, const state& d_tau, const bool plot, state_time_list_t* vis) {
		Alpha = block(A, -BRiBt, zeros<X_DIM,X_DIM>(), -~A);

		chi.insert(0,0,x1);
		chi.insert(X_DIM,0,-d_tau);

		c0.insert(0,0,c);
		c0.insert(X_DIM,0, zeros<X_DIM>());

		int num_steps = (int) ceil(tau / deltaT);
		double new_deltaT = tau / num_steps;

	#ifndef FRONT_LOAD_RK4
		linear<2*X_DIM> back;
		back.A = &Alpha;
		back.c = &c0;

		rk<2*X_DIM, 1, linear<2*X_DIM> > linearrk;
	#endif

		for (int j = 0; j < num_steps; j++) {
			linearrk.rk4(chi, 0, -new_deltaT, back);
	#if (DYNAMICS == SINGLE_INTEGRATOR_2D) || (DYNAMICS == DOUBLE_INTEGRATOR_2D) || (DYNAMICS == NONHOLONOMIC)
			/*
			ostringstream os;
			os << num_steps << " " << j << "     " << chi[0] << " " << chi[1] << " " << chi[2] << endl;
			fputs(os.str().c_str(), path_log);
			fflush(path_log);
			*/
			//cout << num_steps << " " << j << "     " << chi[0] << " " << chi[1] << " " << chi[2] << endl;
	#else
			//cout << chi[0] << " " << chi[1] << " " << chi[2] << " " << chi[6] << " " << chi[7] << endl;
	#endif

			state x = chi.subMatrix<X_DIM>(0,0);
			control u = -R%(~B*chi.subMatrix<X_DIM>(X_DIM,0)); // This should be negative as it is

			//cout << "checkBounds(x, _x_bounds): " << checkBounds(x, _x_bounds)
			//	<< "\tcheckBounds(u, u_bounds): " << checkBounds(u, u_bounds) << endl;

			if (!(plot || vis) && !check_state(x, u)) {
				return false;
			}

			if (plot || vis) {
				double x_coord = 0.0, y_coord = 0.0, z_coord = 0.0;

	#if (DYNAMICS == QUADROTOR)
				x_coord = x[0];
				y_coord = x[1];
				z_coord = x[2];
	#else
				x_coord = x[0];
				y_coord = 0;
				z_coord = x[1];
	#endif

				if (plot) {
					double temp = deltaT*j;
					fwrite((const void *)&(temp), sizeof(double), 1, path_log);
					fwrite(x._elems, sizeof(double), (x.numRows())*(x.numColumns()), path_log);
					CAL_CreateSphere(solution_group, 2*NODE_SIZE, x_coord, y_coord, z_coord);
				}

				if (vis) {
					vis->push_back(make_pair(deltaT*j, x));
				}
			}
		}
		return true;
	}

	bool check_state(const state& x, const control& u) {
		return check_bounds(x, _x_bounds) && check_bounds(u, _u_bounds) && collision_free(x);
	}

	virtual inline void calc_backward_reachable_bounds(const state& state, BOUNDS& bounds) = 0;
	virtual inline void calc_forward_reachable_bounds(const state& state, BOUNDS& bounds) = 0;
	
	virtual inline void set_x_bounds(const BOUNDS& x_bounds) {
		_x_bounds = x_bounds;
	}

	virtual inline BOUNDS get_x_bounds() const {
		return _x_bounds;
	}

	virtual inline void set_u_bounds(const BOUNDS& u_bounds) {
		_u_bounds = u_bounds;
	}

	virtual inline BOUNDS get_u_bounds() const {
		return _u_bounds;
	}

	virtual inline void set_radius(const floatX& num_states) = 0;

	inline floatX get_radius() const {
		return _radius;
	}

protected:
	const std::complex<double> _im;

	floatX _state_space_volume;
	floatX _unit_sphere_volume;

	floatX _control_penalty;

	floatX _radius_multiplier;

	floatX _radius;

	BOUNDS _x_bounds;
	BOUNDS _u_bounds;

	Matrix<2*_xDim>* _chi;

	/**
	 * Calculates the volume of a unit sphere.
	 *
	 * Verified with table on http://en.wikipedia.org/wiki/Unit_sphere#General_area_and_volume_formulas
	 * Reproduced here
	 * n 	V_n (volume)
	 * 0 	(1/0!)\pi^0 	1.000
	 * 1 	(2^1/1!!)\pi^0 	2.000
	 * 2 	(1/1!)\pi^1 = \pi 	3.142
	 * 3 	(2^2/3!!)\pi^1 = (4/3)\pi 	4.189
	 * 4 	(1/2!)\pi^2 = (1/2)\pi^2 	4.935
	 * 5 	(2^3/5!!)\pi^2 = (8/15)\pi^2 	5.264
	 * 6 	(1/3!)\pi^3 = (1/6)\pi^3 	5.168
	 * 7 	(2^4/7!!) \pi^3 = (16/105)\pi^3 	4.725
	 * 8 	(1/4!)\pi^4 = (1/24)\pi^4 	4.059
	 * 9 	(2^5/9!!) \pi^4 = (32/945)\pi^4 	3.299
	 * 10 	(1/5!)\pi^5 = (1/120)\pi^5 	2.550
	 */
	inline void calc_unit_sphere_volume() {
#if _xDim % 2 == 0
		int i = 4;
		int den = 2;
#else
		int i = 5;
		int den = 3;
#endif

#if ((_xDim % 2 == 0) && (_xDim > 2)) || ((_xDim % 2 != 0) && (_xDim > 3))
		for(; i <= _xDim; i+=2) {
			den *= i;
		}
#endif

		double num = (2*M_PI);
#if _xDim % 2 == 0
		num = pow(num, (double)_xDim/2.0);
#else
		num = 2.0*pow(num, (double)(_xDim-1)/2.0);
#endif

		_unit_sphere_volume = (num/den);
	}

	inline void calc_state_space_volume() {
		_state_space_volume = 1;
		for (BOUNDS::iterator p = _x_bounds.begin(); p != _x_bounds.end(); p++) {
			_state_space_volume *= (p->second - p->first);
		}
	}

	virtual inline void _compute_cost_p(const state& x0, const state& x1, floatX *p) const = 0;
	virtual inline void _compute_cost_d(const state& x0, const state& x1, state& x1diffbarx0, floatX realRoots, state& d) const = 0;

	virtual inline void _check_path_chi(const state& x0, const state& x1, const floatX t, const floatX tau, const state& d_tau) const = 0;
	virtual inline void _check_path_coords(const state& x, floatX* x_coord, floatX* y_coord, floatX* z_coord) const = 0;

	inline floatX computeHeuristic(const floatX x0, const floatX x1, const std::pair<floatX, floatX>& bounds) const {
		floatX tmp = x1 - x0;
		return std::max(tmp/bounds.first, tmp/bounds.second);
	}

private:
	std::string _name;

	floatX _poly_degree;
	floatX* _p;
	floatX* _zeror;
	floatX* _zeroi;
};

template <class D>
bool state_order(const typename D::state_time_t& a, const typename D::state_time_t& b) {
	return (a.first < b.first);
}

#endif // __DYNAMICS_HPP__