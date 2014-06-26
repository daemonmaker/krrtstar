#ifndef __UTILITIES_HPP__
#define __UTILITIES_HPP__

#include <iostream>

inline double rand_value(double a, double b) {
	return ((double) (rand()*(RAND_MAX+1) + rand()) / (RAND_MAX*(RAND_MAX + 2))) * (b - a) + a;
}

inline char _getchar() {
	char k;
	std::cin >> k;
	return k;
}

#endif // __UTILITIES_HPP__