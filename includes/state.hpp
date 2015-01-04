#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <boost/function.hpp>

#ifndef __STATE_HPP__
#define __STATE_HPP__

typedef Eigen::Matrix<double,X_DIM,1> state;

inline double rand_value(double a, double b) {
	return ((double) (rand()*(RAND_MAX+1) + rand()) / (RAND_MAX*(RAND_MAX + 2))) * (b - a) + a;
}

class StateSpace
{
public:
	StateSpace(boost::function<void (state&)> position_generator, const BOUNDS& bounds)
		: position_generator(position_generator), bounds(bounds)
	{}

	virtual void randState(state& s) {
		this->_randState(s.rows(), s);
	}

protected:
	BOUNDS bounds;
	boost::function<void (state&)> position_generator;

	void _randState(int count, state& s) {
		(this->position_generator)(s);

		for (int i = POSITION_DIM; i < count; ++i) {
			s[i] = rand_value(this->bounds[i].first, this->bounds[i].second);
		}
	}
};

class QuadrotorStateSpace
	: public StateSpace
{
public:
	QuadrotorStateSpace(boost::function<void (state&)> position_generator, const BOUNDS& bounds)
		: StateSpace(position_generator, bounds)
	{}

	virtual void randState(state& s) {
		s = Eigen::MatrixXd::Zero(s.rows(),s.cols());
		this->_randState(6, s);
	}
};

class NonholonomicStateSpace
	: public StateSpace
{
public:
	NonholonomicStateSpace(boost::function<void (state&)> position_generator, const BOUNDS& bounds)
		: StateSpace(position_generator, bounds)
	{}

	virtual void randState(state& s) {
		this->_randState(s.rows(), s);
		s[4] = 0;
	}
};

#endif // __STATE_HPP__