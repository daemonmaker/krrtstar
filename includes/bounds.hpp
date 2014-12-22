#include <utility>
#include <vector>

typedef std::pair<double, double> BOUND;
typedef std::vector< BOUND > BOUNDS;

template<typename vec>
bool checkBounds(const vec& v, const BOUNDS& v_bounds) {
//return true; // TODO REMOVE
	for (size_t i = 0; i < v.rows(); i++) {
		if ((v[i] < v_bounds[i].first) || (v[i] > v_bounds[i].second)) {
			return false;
		}
	}
	return true;
}
