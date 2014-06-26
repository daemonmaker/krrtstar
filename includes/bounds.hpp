#ifndef __BOUNDS_H__
#define __BOUNDS_H__

#include <utility>
#include <vector>

#include "matrix.h"

typedef std::pair<double, double> BOUND;
typedef std::vector< BOUND > BOUNDS;

template<typename vec>
bool check_bounds(const vec& v, const BOUNDS& v_bounds) {
	for (size_t i = 0; i < v.numRows(); i++) {
		if ((v[i] < v_bounds[i].first) || (v[i] > v_bounds[i].second)) {
			return false;
		}
	}
	return true;
}

#endif // __BOUNDS_H__