#ifndef __DBLINT2D_HPP__
#define __DBLINT2D_HPP__

#include <Dynamics/DblInt.hpp>

#define DBLINT2D_X_DIM 4
#define DBLINT2D_U_DIM 2

/**
* TOOD
* - Standardize function naming convention to use underscores
* - Move random state generation into the dynamics. Some worlds have complicated position bounds so randomly generating a state vector will need to be a function of the world (i.e. the world will randomly generate the position elements while the dynamics will randomly generate the rest).
* - Remove radius parameter from calc_*_reachable_bounds
* - Move into dynamics calc_*_reachable_bounds
* - Replace double with floatX throughout all dynamics files
*/

class DblInt2D : public DblInt<DBLINT2D_X_DIM, DBLINT2D_U_DIM> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	static const size_t X_DIM;
	static const size_t U_DIM;

	DblInt2D(const floatX control_penalty, const floatX poly_degree, const floatX radius_multiplier, const floatX radius, const BOUNDS& world_bounds) : DblInt("DblInt2D", control_penalty, poly_degree, radius_multiplier, radius) {
		_x_bounds.resize(DBLINT2D_X_DIM);
		_u_bounds.resize(DBLINT2D_U_DIM);

		// TODO The bounds are a function of the map
		_x_bounds[0] = std::make_pair(world_bounds[0].first, world_bounds[0].second);
		_x_bounds[1] = std::make_pair(world_bounds[1].first, world_bounds[1].second);
		_x_bounds[2] = std::make_pair(-10, 10);
		_x_bounds[3] = std::make_pair(-10, 10);

		_u_bounds[0] = std::make_pair(-10, 10);
		_u_bounds[1] = std::make_pair(-10, 10);

		calc_state_space_volume();
	}

	virtual inline void rand_state(Dynamics::state& v) {
		//int count = v.numRows();
		int count = v.rows();
		for (size_t i = 0; i < count; i++) {
			v[i] = rand_value(_x_bounds[i].first, _x_bounds[i].second);
		}
	}

	virtual inline floatX applyHeuristics(const state& x0, const state& x1) const {
		floatX cost = 0.0;

		floatX cost1 = std::max(computeHeuristic(x0[0], x1[0], _x_bounds[2]), computeHeuristic(x0[1], x1[1], _x_bounds[3]));
		floatX cost2 = std::max(computeHeuristic(x0[2], x1[2], _u_bounds[0]), computeHeuristic(x0[3], x1[3], _u_bounds[1]));

		cost = std::max(cost1, cost2);

		return cost;
	}

	//virtual inline Matrix<DBLINT2D_X_DIM> dynamics(floatX step, const Matrix<DBLINT2D_X_DIM>& x, const Matrix<DBLINT2D_U_DIM>& u) const {
	virtual inline Eigen::Matrix<double,DBLINT2D_X_DIM,1> dynamics(floatX step, const Eigen::Matrix<double,DBLINT2D_X_DIM,1>& x, const Eigen::Matrix<double,DBLINT2D_X_DIM,1>& u) const {
		//Matrix<DBLINT2D_X_DIM> xNew;
		Eigen::Matrix<double,DBLINT2D_X_DIM,1> xNew;

		xNew[0] = x[0] + x[2]*step + u[0]*step*step;
		xNew[1] = x[1] + x[3]*step + u[1]*step*step;
		xNew[2] = x[2] + u[0]*step;
		xNew[3] = x[3] + u[1]*step;

		return xNew;
	}

	virtual inline void _compute_cost_p(const Dynamics::state& x0, const Dynamics::state& x1, floatX *p) const {
		p[0] = 1;
		p[1] = 0;
		p[2] = -4.0*_control_penalty*x1[3]*x1[3]-4.0*_control_penalty*x0[3]*x0[3]-4.0*_control_penalty*x1[2]*x1[2]-4.0*_control_penalty*x1[3]*x0[3]-4.0*_control_penalty*x1[2]*x0[2]-4.0*_control_penalty*x0[2]*x0[2];
		p[3] = 24.0*_control_penalty*x1[1]*x0[3]+24.0*_control_penalty*x1[0]*x0[2]-24.0*_control_penalty*x0[0]*x0[2]-24.0*_control_penalty*x0[1]*x0[3]+24.0*_control_penalty*x1[2]*x1[0]-24.0*_control_penalty*x1[2]*x0[0]+24.0*_control_penalty*x1[3]*x1[1]-24.0*_control_penalty*x1[3]*x0[1];
		p[4] = -36.0*_control_penalty*x1[0]*x1[0]-36.0*_control_penalty*x0[0]*x0[0]-36.0*_control_penalty*x1[1]*x1[1]-36.0*_control_penalty*x0[1]*x0[1]+72.0*_control_penalty*x1[0]*x0[0]+72.0*_control_penalty*x1[1]*x0[1];
	}
	
	virtual inline void _compute_cost_d(const Dynamics::state& x0, const Dynamics::state& x1, Dynamics::state& x1diffbarx0, floatX realRoot, Dynamics::state& d) const {
		floatX t7 = realRoot*realRoot;
		floatX t10 = _control_penalty/t7/realRoot;
		floatX t14 = 1/t7*_control_penalty;
		floatX t26 = _control_penalty/realRoot;
		x1diffbarx0[0] = x1[0]-x0[0]-realRoot*x0[2];
		x1diffbarx0[1] = x1[1]-x0[1]-realRoot*x0[3];
		x1diffbarx0[2] = x1[2]-x0[2];
		x1diffbarx0[3] = x1[3]-x0[3];
		d[0] = 12.0*t10*x1diffbarx0[0]-6.0*t14*x1diffbarx0[2];
		d[1] = 12.0*t10*x1diffbarx0[1]-6.0*t14*x1diffbarx0[3];
		d[2] = -6.0*t14*x1diffbarx0[0]+4.0*t26*x1diffbarx0[2];
		d[3] = -6.0*t14*x1diffbarx0[1]+4.0*t26*x1diffbarx0[3];
	}

	virtual inline void _check_path_chi(const Dynamics::state& x0, const Dynamics::state& x1, const floatX t, const floatX tau, const Dynamics::state& d_tau) const {
		floatX t1 = t-tau;
		floatX t3 = tau*tau;
		floatX t7 = t*t;
		floatX t12 = 1/_control_penalty;
		floatX t13 = (t3*tau-3.0*t3*t+3.0*tau*t7-t7*t)*t12;
		floatX t19 = (t3-2.0*tau*t+t7)*t12;
		floatX t31 = -t1*t12;
		(*_chi)[0] = x1[0]+t1*x1[2]+t13*d_tau[0]/6.0+t19*d_tau[2]/2.0;
		(*_chi)[1] = x1[1]+t1*x1[3]+t13*d_tau[1]/6.0+t19*d_tau[3]/2.0;
		(*_chi)[2] = x1[2]-t19*d_tau[0]/2.0-t31*d_tau[2];
		(*_chi)[3] = x1[3]-t19*d_tau[1]/2.0-t31*d_tau[3];
		(*_chi)[4] = d_tau[0];
		(*_chi)[5] = d_tau[1];
		(*_chi)[6] = -t1*d_tau[0]+d_tau[2];
		(*_chi)[7] = -t1*d_tau[1]+d_tau[3];
	}

	virtual inline void _check_path_coords(const Dynamics::state& x, floatX* x_coord, floatX* y_coord, floatX* z_coord) const {
		(*x_coord) = x[0];
		(*y_coord) = 0;
		(*z_coord) = x[1];
	}

	virtual inline void calc_backward_reachable_bounds(const state& state, BOUNDS& bounds) {
		bounds.resize(X_DIM);

		floatX x0_0 = state[0];
		floatX x0_1 = state[1];
		floatX x0_2 = state[2];
		floatX x0_3 = state[3];

		// Calculate x1 bounds
		{
			std::complex<floatX> t1 = x0_2*x0_2;
			std::complex<floatX> t4 = _radius*_radius;
			std::complex<floatX> t6 = t1*t1;
			std::complex<floatX> t11 = t4*t4;
			std::complex<floatX> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
			std::complex<floatX> t18 = pow(-12.0*_radius*t1+t4*_radius+2.0*t15,0.3333333333333333);
			std::complex<floatX> t19 = t18/4.0;
			std::complex<floatX> t25 = 4.0*(t1/4.0-t4/16.0)/t18;
			std::complex<floatX> t26 = _radius/2.0;
			std::complex<floatX> t27 = t19-t25-t26;
			std::complex<floatX> t29 = t27*t27;
			std::complex<floatX> t34 = sqrt(-3.0*t29*t27*(t26+t19-t25));
			std::complex<floatX> t36_min = x0_0+t27*x0_2-t34/3.0;
			std::complex<floatX> t36_max = x0_0+t27*x0_2+t34/3.0;

			bounds[0].first = t36_min.real();
			bounds[0].second = t36_max.real();
		}

		{
			std::complex<floatX> t1 = x0_2*x0_2;
			std::complex<floatX> t4 = _radius*_radius;
			std::complex<floatX> t6 = t1*t1;
			std::complex<floatX> t11 = t4*t4;
			std::complex<floatX> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
			std::complex<floatX> t18 = pow(-12.0*_radius*t1+t4*_radius+2.0*t15,0.3333333333333333);
			std::complex<floatX> t19 = t18/8.0;
			std::complex<floatX> t24 = (t1/4.0-t4/16.0)/t18;
			std::complex<floatX> t25 = 2.0*t24;
			std::complex<floatX> t26 = _radius/2.0;
			std::complex<floatX> t27 = sqrt(3.0);
			std::complex<floatX> t32 = (1.0/2.0*_im)*t27*(t18/4.0+4.0*t24);
			std::complex<floatX> t33 = -t19+t25-t26+t32;
			std::complex<floatX> t35 = t33*t33;
			std::complex<floatX> t40 = sqrt(-3.0*t35*t33*(t26-t19+t25+t32));
			std::complex<floatX> t42_min = x0_0+t33*x0_2-t40/3.0;
			std::complex<floatX> t42_max = x0_0+t33*x0_2+t40/3.0;

			bounds[0].first = std::min(bounds[0].first, t42_min.real());
			bounds[0].second = std::max(bounds[0].second, t42_max.real());
		}

		{
			std::complex<floatX> t1 = x0_2*x0_2;
			std::complex<floatX> t4 = _radius*_radius;
			std::complex<floatX> t6 = t1*t1;
			std::complex<floatX> t11 = t4*t4;
			std::complex<floatX> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
			std::complex<floatX> t18 = pow(-12.0*_radius*t1+t4*_radius+2.0*t15,0.3333333333333333);
			std::complex<floatX> t19 = t18/8.0;
			std::complex<floatX> t24 = (t1/4.0-t4/16.0)/t18;
			std::complex<floatX> t25 = 2.0*t24;
			std::complex<floatX> t26 = _radius/2.0;
			std::complex<floatX> t27 = sqrt(3.0);
			std::complex<floatX> t32 = (-1.0/2.0*_im)*t27*(t18/4.0+4.0*t24);
			std::complex<floatX> t33 = -t19+t25-t26+t32;
			std::complex<floatX> t35 = t33*t33;
			std::complex<floatX> t40 = sqrt(-3.0*t35*t33*(t26-t19+t25+t32));
			std::complex<floatX> t42_min = x0_0+t33*x0_2-t40/3.0;
			std::complex<floatX> t42_max = x0_0+t33*x0_2+t40/3.0;

			bounds[0].first = std::min(bounds[0].first, t42_min.real());
			bounds[0].second = std::max(bounds[0].second, t42_max.real());
		}

		// Calculate x2 bounds
		{
			std::complex<floatX> t1 = x0_3*x0_3;
			std::complex<floatX> t4 = _radius*_radius;
			std::complex<floatX> t6 = t1*t1;
			std::complex<floatX> t11 = t4*t4;
			std::complex<floatX> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
			std::complex<floatX> t18 = pow(-12.0*_radius*t1+t4*_radius+2.0*t15,0.3333333333333333);
			std::complex<floatX> t19 = t18/4.0;
			std::complex<floatX> t25 = 4.0*(t1/4.0-t4/16.0)/t18;
			std::complex<floatX> t26 = _radius/2.0;
			std::complex<floatX> t27 = t19-t25-t26;
			std::complex<floatX> t29 = t27*t27;
			std::complex<floatX> t34 = sqrt(-3.0*t29*t27*(t26+t19-t25));
			std::complex<floatX> t36_min = x0_1+t27*x0_3-t34/3.0;
			std::complex<floatX> t36_max = x0_1+t27*x0_3+t34/3.0;

			bounds[1].first = t36_min.real();
			bounds[1].second = t36_max.real();
		}

		{
			std::complex<floatX> t1 = x0_3*x0_3;
			std::complex<floatX> t4 = _radius*_radius;
			std::complex<floatX> t6 = t1*t1;
			std::complex<floatX> t11 = t4*t4;
			std::complex<floatX> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
			std::complex<floatX> t18 = pow(-12.0*_radius*t1+t4*_radius+2.0*t15,0.3333333333333333);
			std::complex<floatX> t19 = t18/8.0;
			std::complex<floatX> t24 = (t1/4.0-t4/16.0)/t18;
			std::complex<floatX> t25 = 2.0*t24;
			std::complex<floatX> t26 = _radius/2.0;
			std::complex<floatX> t27 = sqrt(3.0);
			std::complex<floatX> t32 = (1.0/2.0*_im)*t27*(t18/4.0+4.0*t24);
			std::complex<floatX> t33 = -t19+t25-t26+t32;
			std::complex<floatX> t35 = t33*t33;
			std::complex<floatX> t40 = sqrt(-3.0*t35*t33*(t26-t19+t25+t32));
			std::complex<floatX> t42_min = x0_1+t33*x0_3-t40/3.0;
			std::complex<floatX> t42_max = x0_1+t33*x0_3+t40/3.0;

			bounds[1].first = std::min(bounds[1].first, t42_min.real());
			bounds[1].second = std::max(bounds[1].second, t42_max.real());
		}

		{
			std::complex<floatX> t1 = x0_3*x0_3;
			std::complex<floatX> t4 = _radius*_radius;
			std::complex<floatX> t6 = t1*t1;
			std::complex<floatX> t11 = t4*t4;
			std::complex<floatX> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
			std::complex<floatX> t18 = pow(-12.0*_radius*t1+t4*_radius+2.0*t15,0.3333333333333333);
			std::complex<floatX> t19 = t18/8.0;
			std::complex<floatX> t24 = (t1/4.0-t4/16.0)/t18;
			std::complex<floatX> t25 = 2.0*t24;
			std::complex<floatX> t26 = _radius/2.0;
			std::complex<floatX> t27 = sqrt(3.0);
			std::complex<floatX> t32 = (-1.0/2.0*_im)*t27*(t18/4.0+4.0*t24);
			std::complex<floatX> t33 = -t19+t25-t26+t32;
			std::complex<floatX> t35 = t33*t33;
			std::complex<floatX> t40 = sqrt(-3.0*t35*t33*(t26-t19+t25+t32));
			std::complex<floatX> t42_min = x0_1+t33*x0_3-t40/3.0;
			std::complex<floatX> t42_max = x0_1+t33*x0_3+t40/3.0;

			bounds[1].first = std::min(bounds[1].first, t42_min.real());
			bounds[1].second = std::max(bounds[1].second, t42_max.real());
		}

		// Calculate x3 bounds
		{
			floatX t1 = sqrt(4.0);
			floatX t2 = _radius*_radius;
			floatX t3 = sqrt(t2);
			floatX t6_min = x0_2-t1*t3/4.0;
			floatX t6_max = x0_2+t1*t3/4.0;

			bounds[2].first = t6_min;
			bounds[2].second = t6_max;
		}

		// Calculate x4 bounds
		{
			floatX t1 = sqrt(4.0);
			floatX t2 = _radius*_radius;
			floatX t3 = sqrt(t2);
			floatX t6_min = x0_3-t1*t3/4.0;
			floatX t6_max = x0_3+t1*t3/4.0;

			bounds[3].first = t6_min;
			bounds[3].second = t6_max;
		}
	}

	virtual inline void calc_forward_reachable_bounds(const state& state, BOUNDS& bounds) {
		bounds.resize(X_DIM);

		double x0_0 = state[0];
		double x0_1 = state[1];
		double x0_2 = state[2];
		double x0_3 = state[3];

		// Calculate x1 min
		{
			std::complex<double> t1 = x0_2*x0_2;
			std::complex<double> t4 = _radius*_radius;
			std::complex<double> t6 = t1*t1;
			std::complex<double> t11 = t4*t4;
			std::complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
			std::complex<double> t18 = pow(12.0*_radius*t1-t4*_radius+2.0*t15,0.3333333333333333);
			std::complex<double> t19 = t18/4.0;
			std::complex<double> t25 = 4.0*(t1/4.0-t4/16.0)/t18;
			std::complex<double> t26 = _radius/2.0;
			std::complex<double> t27 = t19-t25+t26;
			std::complex<double> t29 = sqrt(3.0);
			std::complex<double> t31 = t27*t27;
			std::complex<double> t34 = sqrt((t26-t19+t25)*t31*t27);
			std::complex<double> t37 = x0_0+t27*x0_2-t29*t34/3.0;

			bounds[0].first = t37.real();
		}

		{
			std::complex<double> t1 = x0_2*x0_2;
			std::complex<double> t4 = _radius*_radius;
			std::complex<double> t6 = t1*t1;
			std::complex<double> t11 = t4*t4;
			std::complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
			std::complex<double> t18 = pow(12.0*_radius*t1-t4*_radius+2.0*t15,0.3333333333333333);
			std::complex<double> t19 = t18/8.0;
			std::complex<double> t24 = (t1/4.0-t4/16.0)/t18;
			std::complex<double> t25 = 2.0*t24;
			std::complex<double> t26 = _radius/2.0;
			std::complex<double> t27 = sqrt(3.0);
			std::complex<double> t31 = t18/4.0+4.0*t24;
			std::complex<double> t33 = -t19+t25+t26+(1.0/2.0*_im)*t27*t31;
			std::complex<double> t38 = t33*t33;
			std::complex<double> t41 = sqrt((t26+t19-t25+(-1.0/2.0*_im)*t27*t31)*t38*t33);
			std::complex<double> t44 = x0_0+t33*x0_2-t27*t41/3.0;

			bounds[0].first = std::min(bounds[0].first, t44.real());
		}

		{
			std::complex<double> t1 = x0_2*x0_2;
			std::complex<double> t4 = _radius*_radius;
			std::complex<double> t6 = t1*t1;
			std::complex<double> t11 = t4*t4;
			std::complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
			std::complex<double> t18 = pow(12.0*_radius*t1-t4*_radius+2.0*t15,0.3333333333333333);
			std::complex<double> t19 = t18/8.0;
			std::complex<double> t24 = (t1/4.0-t4/16.0)/t18;
			std::complex<double> t25 = 2.0*t24;
			std::complex<double> t26 = _radius/2.0;
			std::complex<double> t27 = sqrt(3.0);
			std::complex<double> t31 = t18/4.0+4.0*t24;
			std::complex<double> t33 = -t19+t25+t26+(-1.0/2.0*_im)*t27*t31;
			std::complex<double> t38 = t33*t33;
			std::complex<double> t41 = sqrt((t26+t19-t25+(1.0/2.0*_im)*t27*t31)*t38*t33);
			std::complex<double> t44 = x0_0+t33*x0_2-t27*t41/3.0;

			bounds[0].first = std::min(bounds[0].first, t44.real());
		}

		// Calculate x1 max
		{
			std::complex<double> t1 = x0_2*x0_2;
			std::complex<double> t4 = _radius*_radius;
			std::complex<double> t6 = t1*t1;
			std::complex<double> t11 = t4*t4;
			std::complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
			std::complex<double> t18 = pow(12.0*_radius*t1-t4*_radius+2.0*t15,0.3333333333333333);
			std::complex<double> t19 = t18/4.0;
			std::complex<double> t25 = 4.0*(t1/4.0-t4/16.0)/t18;
			std::complex<double> t26 = _radius/2.0;
			std::complex<double> t27 = t19-t25+t26;
			std::complex<double> t29 = sqrt(3.0);
			std::complex<double> t31 = t27*t27;
			std::complex<double> t34 = sqrt((t26-t19+t25)*t31*t27);
			std::complex<double> t37 = x0_0+t27*x0_2+t29*t34/3.0;

			bounds[0].second = t37.real();
		}

		{
			std::complex<double> t1 = x0_2*x0_2;
			std::complex<double> t4 = _radius*_radius;
			std::complex<double> t6 = t1*t1;
			std::complex<double> t11 = t4*t4;
			std::complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
			std::complex<double> t18 = pow(12.0*_radius*t1-t4*_radius+2.0*t15,0.3333333333333333);
			std::complex<double> t19 = t18/8.0;
			std::complex<double> t24 = (t1/4.0-t4/16.0)/t18;
			std::complex<double> t25 = 2.0*t24;
			std::complex<double> t26 = _radius/2.0;
			std::complex<double> t27 = sqrt(3.0);
			std::complex<double> t31 = t18/4.0+4.0*t24;
			std::complex<double> t33 = -t19+t25+t26+(1.0/2.0*_im)*t27*t31;
			std::complex<double> t38 = t33*t33;
			std::complex<double> t41 = sqrt((t26+t19-t25+(-1.0/2.0*_im)*t27*t31)*t38*t33);
			std::complex<double> t44 = x0_0+t33*x0_2+t27*t41/3.0;

			bounds[0].second = std::max(bounds[0].second, t44.real());
		}

		{
			std::complex<double> t1 = x0_2*x0_2;
			std::complex<double> t4 = _radius*_radius;
			std::complex<double> t6 = t1*t1;
			std::complex<double> t11 = t4*t4;
			std::complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
			std::complex<double> t18 = pow(12.0*_radius*t1-t4*_radius+2.0*t15,0.3333333333333333);
			std::complex<double> t19 = t18/8.0;
			std::complex<double> t24 = (t1/4.0-t4/16.0)/t18;
			std::complex<double> t25 = 2.0*t24;
			std::complex<double> t26 = _radius/2.0;
			std::complex<double> t27 = sqrt(3.0);
			std::complex<double> t31 = t18/4.0+4.0*t24;
			std::complex<double> t33 = -t19+t25+t26+(-1.0/2.0*_im)*t27*t31;
			std::complex<double> t38 = t33*t33;
			std::complex<double> t41 = sqrt((t26+t19-t25+(1.0/2.0*_im)*t27*t31)*t38*t33);
			std::complex<double> t44 = x0_0+t33*x0_2+t27*t41/3.0;

			bounds[0].second = std::max(bounds[0].second, t44.real());
		}

		// Calculate x2 min
		{
			std::complex<double> t1 = x0_3*x0_3;
			std::complex<double> t4 = _radius*_radius;
			std::complex<double> t6 = t1*t1;
			std::complex<double> t11 = t4*t4;
			std::complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
			std::complex<double> t18 = pow(12.0*_radius*t1-t4*_radius+2.0*t15,0.3333333333333333);
			std::complex<double> t19 = t18/4.0;
			std::complex<double> t25 = 4.0*(t1/4.0-t4/16.0)/t18;
			std::complex<double> t26 = _radius/2.0;
			std::complex<double> t27 = t19-t25+t26;
			std::complex<double> t29 = sqrt(3.0);
			std::complex<double> t31 = t27*t27;
			std::complex<double> t34 = sqrt((t26-t19+t25)*t31*t27);
			std::complex<double> t37 = x0_1+t27*x0_3-t29*t34/3.0;

			bounds[1].first = t37.real();
		}

		{
			std::complex<double> t1 = x0_3*x0_3;
			std::complex<double> t4 = _radius*_radius;
			std::complex<double> t6 = t1*t1;
			std::complex<double> t11 = t4*t4;
			std::complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
			std::complex<double> t18 = pow(12.0*_radius*t1-t4*_radius+2.0*t15,0.3333333333333333);
			std::complex<double> t19 = t18/8.0;
			std::complex<double> t24 = (t1/4.0-t4/16.0)/t18;
			std::complex<double> t25 = 2.0*t24;
			std::complex<double> t26 = _radius/2.0;
			std::complex<double> t27 = sqrt(3.0);
			std::complex<double> t31 = t18/4.0+4.0*t24;
			std::complex<double> t33 = -t19+t25+t26+(1.0/2.0*_im)*t27*t31;
			std::complex<double> t38 = t33*t33;
			std::complex<double> t41 = sqrt((t26+t19-t25+(-1.0/2.0*_im)*t27*t31)*t38*t33);
			std::complex<double> t44 = x0_1+t33*x0_3-t27*t41/3.0;

			bounds[1].first = std::min(bounds[1].first, t44.real());
		}

		{
			std::complex<double> t1 = x0_3*x0_3;
			std::complex<double> t4 = _radius*_radius;
			std::complex<double> t6 = t1*t1;
			std::complex<double> t11 = t4*t4;
			std::complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
			std::complex<double> t18 = pow(12.0*_radius*t1-t4*_radius+2.0*t15,0.3333333333333333);
			std::complex<double> t19 = t18/8.0;
			std::complex<double> t24 = (t1/4.0-t4/16.0)/t18;
			std::complex<double> t25 = 2.0*t24;
			std::complex<double> t26 = _radius/2.0;
			std::complex<double> t27 = sqrt(3.0);
			std::complex<double> t31 = t18/4.0+4.0*t24;
			std::complex<double> t33 = -t19+t25+t26+(-1.0/2.0*_im)*t27*t31;
			std::complex<double> t38 = t33*t33;
			std::complex<double> t41 = sqrt((t26+t19-t25+(1.0/2.0*_im)*t27*t31)*t38*t33);
			std::complex<double> t44 = x0_1+t33*x0_3-t27*t41/3.0;

			bounds[1].first = std::min(bounds[1].first, t44.real());
		}

		// Calculate x2 max
		{
			std::complex<double> t1 = x0_3*x0_3;
			std::complex<double> t4 = _radius*_radius;
			std::complex<double> t6 = t1*t1;
			std::complex<double> t11 = t4*t4;
			std::complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
			std::complex<double> t18 = pow(12.0*_radius*t1-t4*_radius+2.0*t15,0.3333333333333333);
			std::complex<double> t19 = t18/4.0;
			std::complex<double> t25 = 4.0*(t1/4.0-t4/16.0)/t18;
			std::complex<double> t26 = _radius/2.0;
			std::complex<double> t27 = t19-t25+t26;
			std::complex<double> t29 = sqrt(3.0);
			std::complex<double> t31 = t27*t27;
			std::complex<double> t34 = sqrt((t26-t19+t25)*t31*t27);
			std::complex<double> t37 = x0_1+t27*x0_3+t29*t34/3.0;

			bounds[1].second = t37.real();
		}

		{
			std::complex<double> t1 = x0_3*x0_3;
			std::complex<double> t4 = _radius*_radius;
			std::complex<double> t6 = t1*t1;
			std::complex<double> t11 = t4*t4;
			std::complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
			std::complex<double> t18 = pow(12.0*_radius*t1-t4*_radius+2.0*t15,0.3333333333333333);
			std::complex<double> t19 = t18/8.0;
			std::complex<double> t24 = (t1/4.0-t4/16.0)/t18;
			std::complex<double> t25 = 2.0*t24;
			std::complex<double> t26 = _radius/2.0;
			std::complex<double> t27 = sqrt(3.0);
			std::complex<double> t31 = t18/4.0+4.0*t24;
			std::complex<double> t33 = -t19+t25+t26+(1.0/2.0*_im)*t27*t31;
			std::complex<double> t38 = t33*t33;
			std::complex<double> t41 = sqrt((t26+t19-t25+(-1.0/2.0*_im)*t27*t31)*t38*t33);
			std::complex<double> t44 = x0_1+t33*x0_3+t27*t41/3.0;

			bounds[1].second = std::max(bounds[1].second, t44.real());
		}

		{
			std::complex<double> t1 = x0_3*x0_3;
			std::complex<double> t4 = _radius*_radius;
			std::complex<double> t6 = t1*t1;
			std::complex<double> t11 = t4*t4;
			std::complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
			std::complex<double> t18 = pow(12.0*_radius*t1-t4*_radius+2.0*t15,0.3333333333333333);
			std::complex<double> t19 = t18/8.0;
			std::complex<double> t24 = (t1/4.0-t4/16.0)/t18;
			std::complex<double> t25 = 2.0*t24;
			std::complex<double> t26 = _radius/2.0;
			std::complex<double> t27 = sqrt(3.0);
			std::complex<double> t31 = t18/4.0+4.0*t24;
			std::complex<double> t33 = -t19+t25+t26+(-1.0/2.0*_im)*t27*t31;
			std::complex<double> t38 = t33*t33;
			std::complex<double> t41 = sqrt((t26+t19-t25+(1.0/2.0*_im)*t27*t31)*t38*t33);
			std::complex<double> t44 = x0_1+t33*x0_3+t27*t41/3.0;

			bounds[1].second = std::max(bounds[1].second, t44.real());
		}

		// Calculate min/max bounds for x3
		{
			double t1 = sqrt(4.0);
			double t2 = _radius*_radius;
			double t3 = sqrt(t2);

			bounds[2].first = x0_2-t1*t3/4.0;
			bounds[2].second = x0_2+t1*t3/4.0;
		}

		// Calculate min/max bounds for x4
		{
			double t1 = sqrt(4.0);
			double t2 = _radius*_radius;
			double t3 = sqrt(t2);

			bounds[3].first = x0_3-t1*t3/4.0;
			bounds[3].second = x0_3+t1*t3/4.0;
		}
	}

	virtual inline void set_radius(const floatX& num_states) {
		floatX t4 = _unit_sphere_volume*num_states;
		floatX t5 = pow(225.0,0.3333333333333333);
		floatX t6 = _state_space_volume*_state_space_volume;
		floatX t7 = log(num_states);
		floatX t8 = t7*t7;
		floatX t11 = pow(t6*t8*t4,0.3333333333333333);
		floatX t14 = sqrt(t4*t5*t11);
		floatX t16 = sqrt(t4*t14);
		floatX t18 = 3.0/_unit_sphere_volume/num_states*t16;
		_radius = t18*_radius_multiplier;
	}
};

#endif // __DBLINT2D_HPP__