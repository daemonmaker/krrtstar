#ifndef __EIGENM_H__
#define __EIGENM_H__

#include <Eigen/Dense>

template <size_t _numRows1, size_t _numRows2, size_t _numCols1, size_t _numCols2>
inline Eigen::Matrix<double, _numRows1+_numRows2, _numCols1+_numCols2> block(const Eigen::Matrix<double,_numRows1,_numCols1> A, const Eigen::Matrix<double,_numRows1,_numCols2> B,
																			 const Eigen::Matrix<double,_numRows2,_numCols1> C, const Eigen::Matrix<double,_numRows2,_numCols2> D) {
	Eigen::Matrix<double,_numRows1+_numRows2,_numCols1+_numCols2> m;
	m.block<_numRows1,_numCols1>(0,0) = A;			m.block<_numRows1,_numCols2>(0,_numCols1) = B;
	m.block<_numRows2,_numCols1>(_numRows1,0) = C;	m.block<_numRows2,_numCols2>(_numRows1,_numCols1) = D;
	return m;
}

// P%Q solves PX = Q for X
template <size_t _size, size_t _numColumns>
Eigen::Matrix<double,_size,_numColumns> operator%(const Eigen::Matrix<double,_size,_size>&p, const Eigen::Matrix<double,_size,_numColumns>&q) {
	Eigen::Matrix<double,_size,_numColumns> x = P.fullPivLu().solve(q);
	return x;
}

#endif