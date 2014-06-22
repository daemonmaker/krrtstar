#ifndef __MATRIX_H__
#define __MATRIX_H__

#define _USE_MATH_DEFINES

#include <cstdlib>
#include <fstream>

#include <cmath>
#include <float.h>
#include <iostream>
#include <assert.h>
#include <limits>
#include <iomanip>

template <size_t _numRows, size_t _numColumns = 1> class Matrix {

private:

  //double *_elems;

public:
	double _elems[_numRows * _numColumns];
  /*Matrix() { _elems = new double[_numRows * _numColumns]; }
  Matrix( const Matrix<_numRows, _numColumns>& q ) {
    _elems = new double[_numRows * _numColumns];
    for (size_t i = 0; i < _numRows * _numColumns; ++i) {
      _elems[i] = q._elems[i]; 
    }
  }
  
  ~Matrix() { delete[] _elems; }
  
  Matrix<_numRows, _numColumns>& operator = (const Matrix<_numRows, _numColumns>& q) {
    for (size_t i = 0; i < _numRows * _numColumns; ++i) {
      _elems[i] = q._elems[i]; 
    }
    return (*this);
  }*/

  // Retrieval
  inline size_t numRows() const { 
    return _numRows;
  }
  inline size_t numColumns() const { 
    return _numColumns;
  }
  
  // Subscript operator
  inline double& operator () (size_t row, size_t column) {
    assert(row < _numRows && column < _numColumns);
    return _elems[row * _numColumns + column]; 
  }
  inline double  operator () (size_t row, size_t column) const {
    assert(row < _numRows && column < _numColumns);
    return _elems[row * _numColumns + column]; 
  }

  inline double& operator [] (size_t elt) {
    assert(elt < _numRows * _numColumns);
    return _elems[elt]; 
  }
  inline double  operator [] (size_t elt) const {
    assert(elt < _numRows * _numColumns);
    return _elems[elt]; 
  }

  // Reset to zeros
  inline void reset() { 
    for (size_t i = 0; i < _numRows * _numColumns; ++i) {
      _elems[i] = double(0);
    }
  }

  // Submatrix
  template <size_t numRows, size_t numColumns>
  inline Matrix<numRows, numColumns> subMatrix(size_t row, size_t column) const {
    assert(row + numRows <= _numRows && column + numColumns <= _numColumns);
    Matrix<numRows, numColumns> m;
    for (size_t i = 0; i < numRows; ++i) {
      for (size_t j = 0; j < numColumns; ++j) {
        m(i, j) = (*this)(row + i, column + j);
      }
    }
    return m;
  }

  template <size_t __numRows>
  inline Matrix<__numRows> subMatrix(size_t row, size_t column) const {
    assert(row + __numRows <= _numRows && column < _numColumns);
    Matrix<__numRows> m;
    for (size_t i = 0; i < __numRows; ++i) {
      m[i] = (*this)(row + i, column);
    }
    return m;
  }

  inline Matrix<_numRows> column(size_t columnNr) const {
    assert(columnNr < _numColumns);
    Matrix<_numRows> m;
    for (size_t i = 0; i < _numRows; ++i) {
      m[i] = (*this)(i, columnNr);
    }
    return m;
  }

  inline Matrix<1, _numColumns> row(size_t rowNr) const {
    assert(rowNr < _numRows);
    Matrix<1, _numColumns> m;
    for (size_t i = 0; i < _numColumns; ++i) {
      m[i] = (*this)(rowNr, i);
    }
    return m;
  }

  // Insert
  template <size_t __numRows, size_t __numColumns>
  inline void insert(size_t row, size_t column, const Matrix<__numRows, __numColumns>& q) {
    assert(row + __numRows <= _numRows && column + __numColumns <= _numColumns);
    for (size_t i = 0; i < __numRows; ++i) {
      for (size_t j = 0; j < __numColumns; ++j) {
        (*this)(row + i, column + j) = q(i, j);
      }
    }
  }
  
  // Unary minus
  inline Matrix<_numRows, _numColumns> operator-() const {
    Matrix<_numRows, _numColumns> m;
    for (size_t i = 0; i < _numRows * _numColumns; ++i) {
      m._elems[i] = -_elems[i];
    }
    return m;
  }
  
  // Unary plus
  inline const Matrix<_numRows, _numColumns>& operator+() const { 
    return *this; 
  }

  // Equality
  inline bool operator==(const Matrix<_numRows, _numColumns>& q) const { 
    for (size_t i = 0; i < _numRows * _numColumns; ++i) {
      if (_elems[i] < q._elems[i] || _elems[i] > q._elems[i]) {
        return false;
      }
    }
    return true;
  }

  // Inequality
  inline bool operator!=(const Matrix<_numRows, _numColumns>& q) const { 
    for (size_t i = 0; i < _numRows * _numColumns; ++i) {
        if (_elems[i] < q._elems[i] || _elems[i] > q._elems[i]) {
        return true;
      }
    }
    return false;
  }

  // Matrix addition
  inline Matrix<_numRows, _numColumns> operator+(const Matrix<_numRows, _numColumns>& q) const { 
    Matrix<_numRows, _numColumns> m;
    for (size_t i = 0; i < _numRows * _numColumns; ++i) {
      m._elems[i] = _elems[i] + q._elems[i];
    }
    return m;
  }
  inline const Matrix<_numRows, _numColumns>& operator+=(const Matrix<_numRows, _numColumns>& q) { 
    for (size_t i = 0; i < _numRows * _numColumns; ++i) {
      _elems[i] += q._elems[i];
    }
    return *this; 
  }
  
  // Matrix subtraction
  inline Matrix<_numRows, _numColumns> operator-(const Matrix<_numRows, _numColumns>& q) const { 
    Matrix<_numRows, _numColumns> m;
    for (size_t i = 0; i < _numRows * _numColumns; ++i) {
      m._elems[i] = _elems[i] - q._elems[i];
    }
    return m;
  }
  inline const Matrix<_numRows, _numColumns>& operator-=(const Matrix<_numRows, _numColumns>& q) { 
    for (size_t i = 0; i < _numRows * _numColumns; ++i) {
      _elems[i] -= q._elems[i];
    }
    return *this; 
  }

  // Scalar multiplication
  inline Matrix<_numRows, _numColumns> operator*(double a) const { 
    Matrix<_numRows, _numColumns> m;
    for (size_t i = 0; i < _numRows * _numColumns; ++i) {
      m._elems[i] = _elems[i] * a;
    }
    return m;
  }
  inline const Matrix<_numRows, _numColumns>& operator*=(double a) { 
    for (size_t i = 0; i < _numRows * _numColumns; ++i) {
      _elems[i] *= a;
    }
    return *this;
  }
  inline void multiply(double a) { 
    for (size_t i = 0; i < _numRows * _numColumns; ++i) {
      _elems[i] *= a;
    }
  }
  
  // Scalar division
  inline Matrix<_numRows, _numColumns> operator/(double a) const { 
    Matrix<_numRows, _numColumns> m;
    for (size_t i = 0; i < _numRows * _numColumns; ++i) {
      m._elems[i] = _elems[i] / a;
    }
    return m;
  }
  inline const Matrix<_numRows, _numColumns>& operator/=(double a) { 
    for (size_t i = 0; i < _numRows * _numColumns; ++i) {
      _elems[i] /= a;
    }
    return *this;
  }

  // Matrix multiplication
  /*
  template <size_t __numColumns>
  inline Matrix<_numRows, __numColumns> operator*(const Matrix<_numColumns, __numColumns>& q) const {
    Matrix<_numRows, __numColumns> m;
    for (size_t i = 0; i < _numRows; ++i) {
      for (size_t j = 0; j < __numColumns; ++j) {
        double temp = double(0);
        for (size_t k = 0; k < _numColumns; ++k) {
          temp += ((*this)(i, k) * q(k, j));
        }
        m(i, j) = temp;
      }
    }
    return m;
  }

  template <>
  inline Matrix<_numRows, 2> operator*(const Matrix<_numColumns, 2>& q) const {
    Matrix<_numRows, 2> m;
	double temp0, temp1, temp3;
    for (size_t i = 0; i < _numRows; ++i) {
		temp0 = double(0);
		temp1 = double(0);
		for (size_t k = 0; k < _numColumns; ++k) {
			temp3 = (*this)(i, k);
			temp0 += (temp3 * q(k, 0));
			temp1 += (temp3 * q(k, 1));
		}
		m(i, 0) = temp0;
		m(i, 1) = temp1;
    }
    return m;
  }
  */

  template <size_t __numColumns>
  Matrix<_numRows, __numColumns> operator*(const Matrix<_numColumns, __numColumns>& q) const {
    Matrix<_numRows, __numColumns> m;
	double temp;
	size_t toffset, goffset;
    for (size_t i = 0; i < _numRows; ++i) {
	  toffset = (i* _numColumns);
	  goffset = (i * m.numColumns());
      for (size_t j = 0; j < __numColumns; ++j) {
        temp = double(0);
        for (size_t k = 0; k < _numColumns; ++k) {
          //temp += ((*this)(i, k) * q(k, j));
			temp += (this->_elems[toffset + k])*q._elems[k*q.numColumns() + j];
        }
        //m(i, j) = temp;
		m._elems[goffset + j] = temp;
      }
    }
    return m;
  }

  /*
  inline Matrix<_numRows, 2> operator*(const Matrix<_numColumns, 2>& q) const {
    Matrix<_numRows, 2> m;
    for (size_t i = 0; i < _numRows; ++i) {
	  size_t t1 = (i *_numColumns);
	  size_t t2 = (i * m.numColumns());

	  double temp1 = double(0);
      double temp2 = double(0);
      for (size_t k = 0; k < _numColumns; ++k) {
		double x = (_elems[t1 + k]);
		temp1 += (x*q._elems[k*q.numColumns()]);
		temp2 += (x*q._elems[k*q.numColumns() + 1]);
      }

	  m._elems[t2] = temp1;
	  m._elems[t2 + 1] = temp2;
    }
    return m;
  }
  */
  /*
  template<>
  inline Matrix<2, 2> Matrix<2,2>::operator*<2>(const Matrix<2, 2>& q) const {
    Matrix<2, 2> m;
	m(1,1) = (((*this)(1,1)*q(1,1)) + ((*this)(1,2)*q(2,1)));
	m(1,2) = (((*this)(1,1)*q(1,2)) + ((*this)(1,2)*q(2,2)));
	m(2,1) = (((*this)(2,1)*q(1,1)) + ((*this)(2,2)*q(2,1)));
	m(2,2) = (((*this)(2,1)*q(1,2)) + ((*this)(2,2)*q(2,2)));
    return m;
  }
  */
  inline const Matrix<_numRows, _numColumns>& operator*=(const Matrix<_numColumns, _numColumns>& q) { 
    return ((*this) = (*this) * q); 
  }

  // Matrix transpose
  inline Matrix<_numColumns, _numRows> operator~() const {
    Matrix<_numColumns, _numRows> m;
    for (size_t i = 0; i < _numColumns; ++i) {
      for (size_t j = 0; j < _numRows; ++j) {
        //m(i, j) = (*this)(j, i);
		  m._elems[i * m.numColumns() + j] = this->_elems[j * _numColumns + i];
      }
    }
    return m;
  }
};

#ifdef SPECIALIZE_2X2
template <> class Matrix<2,2> {

private:

  //double *_elems;

public:
	double _elems[4];
  /*Matrix() { _elems = new double[_numRows * _numColumns]; }
  Matrix( const Matrix<_numRows, _numColumns>& q ) {
    _elems = new double[_numRows * _numColumns];
    for (size_t i = 0; i < _numRows * _numColumns; ++i) {
      _elems[i] = q._elems[i]; 
    }
  }
  
  ~Matrix() { delete[] _elems; }
  
  Matrix<_numRows, _numColumns>& operator = (const Matrix<_numRows, _numColumns>& q) {
    for (size_t i = 0; i < _numRows * _numColumns; ++i) {
      _elems[i] = q._elems[i]; 
    }
    return (*this);
  }*/

  // Retrieval
  inline size_t numRows() const { 
    return 2;
  }
  inline size_t numColumns() const { 
    return 2;
  }
  
  // Subscript operator
  inline double& operator () (size_t row, size_t column) {
    assert(row < 2 && column < 2);
    return _elems[row * 2 + column]; 
  }
  inline double  operator () (size_t row, size_t column) const {
    assert(row < 2 && column < 2);
    return _elems[row * 2 + column]; 
  }

  inline double& operator [] (size_t elt) {
    assert(elt < 2 * 2);
    return _elems[elt]; 
  }
  inline double  operator [] (size_t elt) const {
    assert(elt < 2 * 2);
    return _elems[elt]; 
  }

  // Reset to zeros
  inline void reset() { 
	_elems[0] = 0.0;
	_elems[1] = 0.0;
	_elems[2] = 0.0;
	_elems[3] = 0.0;
  }

  // Submatrix
  template <size_t __numRows, size_t __numColumns>
  inline Matrix<__numRows, __numColumns> subMatrix(size_t row, size_t column) const {
    assert(row + __numRows <= 2 && column + __numColumns <= 2);
    Matrix<__numRows, __numColumns> m;
    for (size_t i = 0; i < __numRows; ++i) {
      for (size_t j = 0; j < __numColumns; ++j) {
		  m._elems[i * _numRows + j] = this->_elems[(row + i) * _numRows + (column + j)];
      }
    }
    return m;
  }

  template <size_t __numRows>
  inline Matrix<2> subMatrix(size_t row, size_t column) const {
    assert(row + __numRows <= 2 && column < 2);
    Matrix<__numRows> m;
    for (size_t i = 0; i < __numRows; ++i) {
      m[i] = this->_elems[(row + i)*2 + column];
    }
    return m;
  }

  inline Matrix<2> column(size_t columnNr) const {
    assert(columnNr < 2);
    Matrix<2> m;
	m._elems[0] = this->_elems[columnNr];
	m._elems[1] = this->_elems[2 + columnNr];
	return m;
  }

  inline Matrix<1, 2> row(size_t rowNr) const {
    assert(rowNr < 2);
    Matrix<1, 2> m;
	m._elems[0] = this->_elems[rowNr];
	m._elems[1] = this->_elems[rowNr + 1];
    return m;
  }

  // Insert
  template <size_t __numRows, size_t __numColumns>
  inline void insert(size_t row, size_t column, const Matrix<__numRows, __numColumns>& q) {
    assert(row + __numRows <= 2 && column + __numColumns <= 2);
    for (size_t i = 0; i < __numRows; ++i) {
      for (size_t j = 0; j < __numColumns; ++j) {
        (*this)(row + i, column + j) = q(i, j);
      }
    }
  }
  
  // Unary minus
  inline Matrix<2, 2> operator-() const {
    Matrix<2, 2> m;
    m._elems[0] = -(this->_elems[0]);
    m._elems[1] = -(this->_elems[1]);
    m._elems[2] = -(this->_elems[2]);
    m._elems[3] = -(this->_elems[3]);
    return m;
  }
  
  // Unary plus
  inline const Matrix<2, 2>& operator+() const { 
    return *this; 
  }

  // Equality
  inline bool operator==(const Matrix<2, 2>& q) const { 
	if (this->_elems[0] < q._elems[0] || this->_elems[0] > q._elems[0]) {
	return false;
	}
	if (this->_elems[1] < q._elems[1] || this->_elems[1] > q._elems[1]) {
	return false;
	}
	if (this->_elems[2] < q._elems[2] || this->_elems[2] > q._elems[2]) {
	return false;
	}
	if (this->_elems[3] < q._elems[3] || this->_elems[3] > q._elems[3]) {
	return false;
	}
	return true;
  }

  // Inequality
  inline bool operator!=(const Matrix<2, 2>& q) const { 
	if (this->_elems[0] < q._elems[0] || this->_elems[0] > q._elems[0]) {
		return true;
	}
	if (this->_elems[1] < q._elems[1] || this->_elems[1] > q._elems[1]) {
		return true;
	}
	if (this->_elems[2] < q._elems[2] || this->_elems[2] > q._elems[2]) {
		return true;
	}
	if (this->_elems[3] < q._elems[3] || this->_elems[3] > q._elems[3]) {
		return true;
	}
    return false;
  }

  // Matrix addition
  inline Matrix<2, 2> operator+(const Matrix<2, 2>& q) const { 
    Matrix<2, 2> m;
    m._elems[0] = this->_elems[0] + q._elems[0];
    m._elems[1] = this->_elems[1] + q._elems[1];
    m._elems[2] = this->_elems[2] + q._elems[2];
    m._elems[3] = this->_elems[3] + q._elems[3];
    return m;
  }
  inline const Matrix<2, 2>& operator+=(const Matrix<2, 2>& q) { 
	this->_elems[0] += q._elems[0];
	this->_elems[1] += q._elems[1];
	this->_elems[2] += q._elems[2];
	this->_elems[3] += q._elems[3];
	return *this; 
  }
  
  // Matrix subtraction
  inline Matrix<2, 2> operator-(const Matrix<2, 2>& q) const { 
    Matrix<2, 2> m;
    m._elems[0] = this->_elems[0] - q._elems[0];
    m._elems[1] = this->_elems[1] - q._elems[1];
    m._elems[2] = this->_elems[2] - q._elems[2];
    m._elems[3] = this->_elems[3] - q._elems[3];
    return m;
  }
  inline const Matrix<2, 2>& operator-=(const Matrix<2, 2>& q) { 
    this->_elems[0] -= q._elems[0];
    this->_elems[1] -= q._elems[1];
    this->_elems[2] -= q._elems[2];
    this->_elems[3] -= q._elems[3];
    return *this; 
  }

  // Scalar multiplication
  inline Matrix<2, 2> operator*(double a) const { 
    Matrix<2, 2> m;
    m._elems[0] = this->_elems[0] * a;
    m._elems[1] = this->_elems[1] * a;
    m._elems[2] = this->_elems[2] * a;
    m._elems[3] = this->_elems[3] * a;
    return m;
  }
  inline const Matrix<2, 2>& operator*=(double a) { 
	this->_elems[0] *= a;
	this->_elems[1] *= a;
	this->_elems[2] *= a;
	this->_elems[3] *= a;
    return *this;
  }
  inline void multiply(double a) { 
	this->_elems[0] *= a;
	this->_elems[1] *= a;
	this->_elems[2] *= a;
	this->_elems[3] *= a;
  }
  
  // Scalar division
  inline Matrix<2, 2> operator/(double a) const { 
    Matrix<2, 2> m;
	m._elems[0] = this->_elems[0] / a;
	m._elems[1] = this->_elems[1] / a;
	m._elems[2] = this->_elems[2] / a;
	m._elems[3] = this->_elems[3] / a;
    return m;
  }
  inline const Matrix<2, 2>& operator/=(double a) { 
	this->_elems[0] /= a;
	this->_elems[1] /= a;
	this->_elems[2] /= a;
	this->_elems[3] /= a;
    return *this;
  }

  // Matrix multiplication
  /*
  template <size_t __numColumns>
  inline Matrix<2, __numColumns> operator*(const Matrix<2, __numColumns>& q) const {
    Matrix<2, __numColumns> m;
    for (size_t i = 0; i < 2; ++i) {
      for (size_t j = 0; j < __numColumns; ++j) {
        double temp = double(0);
        for (size_t k = 0; k < 2; ++k) {
          temp += ((*this)(i, k) * q(k, j));
        }
        m(i, j) = temp;
      }
    }
    return m;
  }

  template <>
  inline Matrix<2, 2> operator*(const Matrix<2, 2>& q) const {
    Matrix<2, 2> m;
	double temp0, temp1, temp3;
    for (size_t i = 0; i < 2; ++i) {
		temp0 = double(0);
		temp1 = double(0);
		for (size_t k = 0; k < 2; ++k) {
			temp3 = (*this)(i, k);
			temp0 += (temp3 * q(k, 0));
			temp1 += (temp3 * q(k, 1));
		}
		m(i, 0) = temp0;
		m(i, 1) = temp1;
    }
    return m;
  }
  */


  template <size_t __numColumns>
  Matrix<2, __numColumns> operator*(const Matrix<2, __numColumns>& q) const {
    Matrix<2, __numColumns> m;
	double temp;
	size_t toffset, goffset;
    for (size_t i = 0; i < 2; ++i) {
	  toffset = (i* 2);
	  goffset = (i * m.numColumns());
      for (size_t j = 0; j < __numColumns; ++j) {
        temp = double(0);
        for (size_t k = 0; k < 2; ++k) {
          //temp += ((*this)(i, k) * q(k, j));
			temp += (this->_elems[toffset + k])*q._elems[k*q.numColumns() + j];
        }
        //m(i, j) = temp;
		m._elems[goffset + j] = temp;
      }
    }
    return m;
  }

  /*
  inline Matrix<2, 2> operator*(const Matrix<2, 2>& q) const {
    Matrix<_numRows, 2> m;
    for (size_t i = 0; i < 2; ++i) {
	  size_t t1 = (i * 2);
	  size_t t2 = (i * m.numColumns());

	  double temp1 = double(0);
      double temp2 = double(0);
      for (size_t k = 0; k < 2; ++k) {
		double x = (_elems[t1 + k]);
		temp1 += (x*q._elems[k*q.numColumns()]);
		temp2 += (x*q._elems[k*q.numColumns() + 1]);
      }

	  m._elems[t2] = temp1;
	  m._elems[t2 + 1] = temp2;
    }
    return m;
  }
  */
 
  template<>
  inline Matrix<2, 2> operator*(const Matrix<2, 2>& q) const {
    Matrix<2, 2> m;
	m._elems[0] = ((this->_elems[0]*q._elems[0]) + (this->_elems[1]*q._elems[2]));
	m._elems[1] = ((this->_elems[0]*q._elems[1]) + (this->_elems[1]*q._elems[3]));
	m._elems[2] = ((this->_elems[2]*q._elems[0]) + (this->_elems[3]*q._elems[2]));
	m._elems[3] = ((this->_elems[2]*q._elems[1]) + (this->_elems[3]*q._elems[3]));
    return m;
  }

  inline const Matrix<2, 2>& operator*=(const Matrix<2, 2>& q) { 
    return ((*this) = (*this) * q); 
  }

  // Matrix transpose
  inline Matrix<2, 2> operator~() const {
    Matrix<2, 2> m;
	m._elems[0] = this->_elems[0];
	m._elems[1] = this->_elems[2];
	m._elems[2] = this->_elems[1];
	m._elems[3] = this->_elems[3];
    return m;
  }
};
#endif
 
/*template < >
class Matrix<1,1>: public Matrix
{
  // Casting to double for 1x1 matrix
  inline operator double() const {
    return _elems[0];
  }
};*/


// Scalar multiplication 
template <size_t _numRows, size_t _numColumns>
inline Matrix<_numRows, _numColumns> operator*(double a, const Matrix<_numRows, _numColumns>& q) { return q*a; }

// Matrix trace
template <size_t _size>
inline double tr(const Matrix<_size, _size>& q) { 
  double trace = double(0);
  for (size_t i = 0; i < _size; ++i){
    trace += q(i, i);
  }
  return trace;
}

// Matrix 1-norm
template <size_t _numRows, size_t _numColumns>
inline double norm(const Matrix<_numRows, _numColumns>& q) {
  double norm1 = double(0);
  for (size_t j = 0; j < _numColumns; ++j) {
    double colabssum = double(0);
    for (size_t i = 0; i < _numRows; ++i) {
      colabssum += abs(q(i,j));
    }
    if (colabssum > norm1) {
      norm1 = colabssum;
    }
  }
  return norm1;
}


// Identity matrix
template <size_t _size>
inline Matrix<_size, _size> eye() {
  Matrix<_size, _size> m;
  for (size_t i = 0; i < _size; ++i) {
    for (size_t j = 0; j < _size; ++j) {
      m(i, j) = (i == j ? double(1) : double(0));
    }
  }
  return m;
}

// Zero matrix
template <size_t _numRows, size_t _numColumns>
inline Matrix<_numRows, _numColumns> zeros() {
  Matrix<_numRows, _numColumns> m;
  m.reset();
  return m;
}

template <size_t _numRows>
inline Matrix<_numRows> zeros() {
  Matrix<_numRows> m;
  m.reset();
  return m;
}

// Matrix determinant
template <size_t _size>
inline double det(const Matrix<_size, _size>& q) { 
  Matrix<_size, _size> m(q);
  double D = double(1);

  size_t row_p[_size];
  size_t col_p[_size];
  for (size_t i = 0; i < _size; ++i) {
    row_p[i] = i; col_p[i] = i;
  }

  // Gaussian elimination
  for (size_t k = 0; k < _size; ++k) {
    // find maximal pivot element
    double maximum = double(0); size_t max_row = k; size_t max_col = k;
    for (size_t i = k; i < _size; ++i) {
      for (size_t j = k; j < _size; ++j) {
        double abs_ij = std::abs(m(row_p[i], col_p[j]));
        if (abs_ij > maximum) {
          maximum = abs_ij; max_row = i; max_col = j;
        }
      }
    }
         
    // swap rows and columns
    if (k != max_row) {
      size_t swap = row_p[k]; row_p[k] = row_p[max_row]; row_p[max_row] = swap;
      D = -D;
    }
    if (k != max_col) {
      size_t swap = col_p[k]; col_p[k] = col_p[max_col]; col_p[max_col] = swap;
      D = -D;
    }

    D *= m(row_p[k], col_p[k]);
    if (D == double(0)) {
      return double(0);
    }

    // eliminate column
    for (size_t i = k + 1; i < _size; ++i) {
      double factor = m(row_p[i], col_p[k]) / m(row_p[k], col_p[k]);
      for (size_t j = k + 1; j < _size; ++j) {
        m(row_p[i], col_p[j]) -= factor * m(row_p[k], col_p[j]);
      }
    }  
  }

  return D;
}

// P%Q solves PX = Q for X
template <size_t _size, size_t _numColumns>
Matrix<_size, _numColumns> operator%(const Matrix<_size, _size>& p, const Matrix<_size, _numColumns>& q) {
#ifndef USE_PARTIAL_PIVOTING
  Matrix<_size, _size> m(p);
  Matrix<_size, _numColumns> inv(q);
  size_t row_p[_size];
  size_t col_p[_size];
  for (size_t i = 0; i < _size; ++i) {
    row_p[i] = i; col_p[i] = i;
  }

  // Gaussian elimination
  for (size_t k = 0; k < _size; ++k) {
    // find maximal pivot element
    double maximum = double(0); size_t max_row = k; size_t max_col = k;
    for (size_t i = k; i < _size; ++i) {
      for (size_t j = k; j < _size; ++j) {
        double abs_ij = std::abs(m(row_p[i], col_p[j]));
        if (abs_ij > maximum) {
			maximum = abs_ij; max_row = i; max_col = j;
        }
      }
    }

    assert(maximum != double(0));

    // swap rows and columns
    std::swap(row_p[k], row_p[max_row]);
    std::swap(col_p[k], col_p[max_col]);
 
    // eliminate column
	double coeff = m._elems[row_p[k] * _size + col_p[k]];
    for (size_t i = k + 1; i < _size; ++i) {
      //double factor = m(row_p[i], col_p[k]) / m(row_p[k], col_p[k]);
	  double factor = m._elems[row_p[i] * _size + col_p[k]] / coeff;
      for (size_t j = k + 1; j < _size; ++j) {
        //m(row_p[i], col_p[j]) -= factor * m(row_p[k], col_p[j]);
		m._elems[row_p[i] * _size + col_p[j]] -= (factor * m._elems[row_p[k] * _size + col_p[j]]);
      }
      for (size_t j = 0; j < _numColumns; ++j) {
        //inv(row_p[i], j) -= factor * inv(row_p[k], j);
		inv._elems[row_p[i] * _numColumns + j] -= (factor * inv._elems[row_p[k] * _numColumns + j]);
      }
    } 
  }

  // Backward substitution
  for (size_t k = _size - 1; k != -1; --k) {
    //double quotient = m(row_p[k], col_p[k]);
	double quotient = m._elems[row_p[k] * _size + col_p[k]];

    for (size_t j = 0; j < _numColumns; ++j) {
      //inv(row_p[k], j) /= quotient;
	  inv._elems[row_p[k] * _numColumns + j] /= quotient;
    }
    
    for (size_t i = 0; i < k; ++i) {
      //double factor = m(row_p[i], col_p[k]);
		double factor = m._elems[row_p[i] * _size + col_p[k]];
      for (size_t j = 0; j < _numColumns; ++j) {
        //inv(row_p[i], j) -= factor * inv(row_p[k], j);
		inv._elems[row_p[i] * _numColumns + j] -= (factor * inv._elems[row_p[k] * _numColumns + j]);
      }
    }
  }

  // reshuffle result
  size_t invrow_p[_size];
  for (size_t i = 0; i < _size; ++i) {
    invrow_p[row_p[i]] = i;
  }
  for (size_t i = 0; i < _size; ++i) {
    for (size_t j = 0; j < _numColumns; ++j) {
      std::swap(inv(col_p[i], j), inv(row_p[i], j));
    }
    row_p[invrow_p[col_p[i]]] = row_p[i];
    invrow_p[row_p[i]] = invrow_p[col_p[i]];
  }
  
  return inv;
#else
  Matrix<_size, _size> m(p);
  Matrix<_size, _numColumns> inv(q);
  size_t row_p[_size];

  for (size_t i = 0; i < _size; ++i) {
    row_p[i] = i;
  }

  // Gaussian elimination
  for (size_t k = 0; k < _size; ++k) {
    // find maximal pivot element
    double maximum = double(0); size_t max_row = k;
    for (size_t i = k; i < _size; ++i) {
      for (size_t j = k; j < _size; ++j) {
        double abs_ij = std::abs(m(row_p[i], j));
        if (abs_ij > maximum) {
			maximum = abs_ij; max_row = i;
        }
      }
    }

    assert(maximum != double(0));

    // swap rows and columns
    std::swap(row_p[k], row_p[max_row]);
 
    // eliminate column
	double num = m._elems[row_p[k] * _size + k];
    for (int i = k + 1; i < _size; ++i) {
      //double factor = m(row_p[i], col_p[k]) / m(row_p[k], col_p[k]);
	  double factor = m._elems[row_p[i] * _size + k] / num;
      for (size_t j = k + 1; j < _size; ++j) {
        //m(row_p[i], col_p[j]) -= factor * m(row_p[k], col_p[j]);
		m._elems[row_p[i] * _size + j] -= (factor * m._elems[row_p[k] * _size + j]);
      }
      for (size_t j = 0; j < _numColumns; ++j) {
        //inv(row_p[i], j) -= factor * inv(row_p[k], j);
		inv._elems[row_p[i] * _numColumns + j] -= (factor * inv._elems[row_p[k] * _numColumns + j]);
      }
    } 
  }

  // Backward substitution
  for (size_t k = _size - 1; k != -1; --k) {
    //double quotient = m(row_p[k], col_p[k]);
	double quotient = m._elems[row_p[k] * _size + k];

    for (size_t j = 0; j < _numColumns; ++j) {
      //inv(row_p[k], j) /= quotient;
	  inv._elems[row_p[k] * _numColumns + j] /= quotient;
    }
    
    for (size_t i = 0; i < k; ++i) {
      //double factor = m(row_p[i], col_p[k]);
		double factor = m._elems[row_p[i] * _size + k];
      for (size_t j = 0; j < _numColumns; ++j) {
        //inv(row_p[i], j) -= factor * inv(row_p[k], j);
		inv._elems[row_p[i] * _numColumns + j] -= (factor * inv._elems[row_p[k] * _numColumns + j]);
      }
    }
  }

  // reshuffle result
  size_t invrow_p[_size];
  for (size_t i = 0; i < _size; ++i) {
    invrow_p[row_p[i]] = i;
  }
  for (size_t i = 0; i < _size; ++i) {
    for (size_t j = 0; j < _numColumns; ++j) {
      std::swap(inv(i, j), inv(row_p[i], j));
    }
    row_p[invrow_p[i]] = row_p[i];
    invrow_p[row_p[i]] = invrow_p[i];
  }
  
  return inv;
#endif
}

#ifdef SPECIALIZE_2X2
template <>
Matrix<2, 1> operator%(const Matrix<2, 2>& p, const Matrix<2, 1>& q) {
	Matrix<2, 2> m(p);
	Matrix<2, 1> inv(q);
	size_t row_p[2];

#ifndef USE_PARTIAL_PIVOTING
	size_t col_p[2];
#endif

	for (size_t i = 0; i < 2; ++i) {
		row_p[i] = i;
#ifndef USE_PARTIAL_PIVOTING
		col_p[i] = i;
#endif
	}

#ifndef USE_PARTIAL_PIVOTING
	// Gaussian elimination
	// find maximal pivot element
	double maximum = double(0); size_t max_row = 0; size_t max_col = 0;
	for (size_t i = 0; i < 2; ++i) {
		for (size_t j = 0; j < 2; ++j) {
			double abs_ij = std::abs(m._elems[row_p[i] * 2 + col_p[j]]);
			if (abs_ij > maximum) {
				maximum = abs_ij; max_row = i; max_col = j;
			}
		}
	}

	assert(maximum != double(0));

	// swap rows and columns
	std::swap(row_p[0], row_p[max_row]);
	std::swap(col_p[0], col_p[max_col]);
 
	// eliminate column
	double factor = m._elems[row_p[1] * 2 + col_p[0]] / m._elems[row_p[0] * 2 + col_p[0]];
	m._elems[row_p[1] * 2 + col_p[1]] -= (factor * m._elems[row_p[0] * 2 + col_p[1]]);
	inv._elems[row_p[1] * 1] -= (factor * inv._elems[row_p[0] * 1]);

	// Backward substitution
	inv._elems[row_p[1]] /= m._elems[row_p[1] * 2 + col_p[1]];
	inv._elems[row_p[0]] -= (m._elems[row_p[0] * 2 + col_p[1]] * inv._elems[row_p[1]]);
	inv._elems[row_p[0]] /= m._elems[row_p[0] * 2 + col_p[0]];

	// reshuffle result
	size_t invrow_p[2];
	for (size_t i = 0; i < 2; ++i) {
		invrow_p[row_p[i]] = i;
	}
	for (size_t i = 0; i < 2; ++i) {
		std::swap(inv._elems[col_p[i]*2], inv._elems[row_p[i]*2]);
		row_p[invrow_p[col_p[i]]] = row_p[i];
		invrow_p[row_p[i]] = invrow_p[col_p[i]];
	}
#else
	// Gaussian elimination
	// find maximal pivot element
	double maximum = double(0); size_t max_row = 0;
	for (size_t i = 0; i < 2; ++i) {
		for (size_t j = 0; j < 2; ++j) {
			double abs_ij = std::abs(m._elems[row_p[i] * 2 + j]);
			if (abs_ij > maximum) {
				maximum = abs_ij; max_row = i;
			}
		}
	}

	assert(maximum != double(0));

	// swap rows and columns
	std::swap(row_p[0], row_p[max_row]);
 
	// eliminate column
	double factor = m._elems[row_p[1] * 2] / m._elems[row_p[0] * 2];
	m._elems[row_p[1] * 2 + 1] -= (factor * m._elems[row_p[0] * 2 + 1]);
	inv._elems[row_p[1] * 1] -= (factor * inv._elems[row_p[0] * 1]);

	// Backward substitution
	inv._elems[row_p[1]] /= m._elems[row_p[1] * 2 + 1];
	inv._elems[row_p[0]] -= (m._elems[row_p[0] * 2 + 1] * inv._elems[row_p[1]]);
	inv._elems[row_p[0]] /= m._elems[row_p[0] * 2 + 0];

	// reshuffle result
	size_t invrow_p[2];
	for (size_t i = 0; i < 2; ++i) {
		invrow_p[row_p[i]] = i;
	}
	for (size_t i = 0; i < 2; ++i) {
		std::swap(inv._elems[i*2], inv._elems[row_p[i]*2]);
		row_p[invrow_p[i]] = row_p[i];
		invrow_p[row_p[i]] = invrow_p[i];
	}
#endif

	return inv;
}
#endif

template <size_t _size, size_t _numRows>
inline Matrix<_numRows, _size> operator/(const Matrix<_numRows, _size>& p, const Matrix<_size, _size>& q) {
  return ~(~q%~p);
}

// Matrix pseudo-inverse
template <size_t _numRows, size_t _numColumns>
inline Matrix<_numColumns, _numRows> pseudoInverse(const Matrix<_numRows, _numColumns>& q) { 
  if (_numColumns <= _numRows) {
    Matrix<_numColumns, _numColumns> Vec, Val;
    jacobi2(~q*q, Vec, Val);

    for (size_t i = 0; i < _numColumns; ++i) {
      if (abs(Val(i,i)) <= sqrt(DBL_EPSILON)) {
        Val(i,i) = 0.0;
      } else {
        Val(i,i) = 1.0 / Val(i,i);
      }
    }
    return (Vec*Val*~Vec)*~q;
  } else {
    Matrix<_numRows, _numRows> Vec, Val;
    jacobi2(q*~q, Vec, Val);

    for (size_t i = 0; i < _numRows; ++i) {
      if (abs(Val(i,i)) <= sqrt(DBL_EPSILON)) {
        Val(i,i) = 0.0;
      } else {
        Val(i,i) = 1.0 / Val(i,i);
      }
    }
    return ~q*(Vec*Val*~Vec);
  }
}

/*template <size_t _numRows, size_t _numColumns>
inline Matrix<_numColumns, _numRows> pseudoInverse(const Matrix<_numRows, _numColumns>& q) { 
  Matrix<_numRows, _numColumns> m(q);
  
  size_t row_p[_numRows];
  size_t col_p[_numColumns];
  for (size_t i = 0; i < _numRows; ++i) {
    row_p[i] = i;
  }
  for (size_t i = 0; i < _numColumns; ++i) {
    col_p[i] = i;
  }

  // Gaussian elimination to determine rank
  size_t rank = (_numRows < _numColumns ? _numRows : _numColumns);
  for (size_t k = 0; k < rank; ++k) {
    // find maximal pivot element
    double maximum = double(0); size_t max_row = k; size_t max_col = k;
    for (size_t i = k; i < _numRows; ++i) {
      for (size_t j = k; j < _numColumns; ++j) {
        double abs_ij = abs(m(row_p[i], col_p[j]));
        if (abs_ij > maximum) {
          maximum = abs_ij; max_row = i; max_col = j;
        }
      }
    }

    // check if zero
    if (maximum <= DBL_EPSILON) {
      rank = k;
      break;
    }

    // swap rows and columns
    if (k != max_row) {
      size_t swap = row_p[k]; row_p[k] = row_p[max_row]; row_p[max_row] = swap;
    }
    if (k != max_col) {
      size_t swap = col_p[k]; col_p[k] = col_p[max_col]; col_p[max_col] = swap;
    }

    // eliminate column
    for (size_t i = k + 1; i < _numRows; ++i) {
      double factor = m(row_p[i], col_p[k]) / m(row_p[k], col_p[k]);
      for (size_t j = k + 1; j < _numColumns; ++j) {
        m(row_p[i], col_p[j]) -= factor * m(row_p[k], col_p[j]);
      }
    } 
  }

  if (rank == 0) { // zero or empty matrix
    return zeros<_numColumns, _numRows>();
  } else if (rank == _numRows && rank == _numColumns) { // full rank square matrix
    // convert so that compiler accepts number of rows and columns for inverse
    Matrix<_numRows, _numRows> q2;
    for (size_t i = 0; i < _numRows * _numRows; ++i) {
      q2[i] = q[i];
    }
    q2 = !q2;
    Matrix<_numColumns, _numRows> q3;
    for (size_t i = 0; i < _numRows * _numRows; ++i) {
      q3[i] = q2[i];
    }
    return q3;
  } else if (rank == _numRows) { // full row-rank matrix
    return ~q/(q*~q);
  } else if (rank == _numColumns) { // full column-rank matrix
    return (~q*q)%~q;
  } // else: not full rank, perform rank decomposition

  // bring m into reduced row echelon form
  
  // normalize rows such that 1's appear on the diagonal
  for (size_t k = 0; k < rank; ++k) {
    double quotient = m(row_p[k], col_p[k]);
    for (size_t j = 0; j < k; ++j) {
      m(row_p[k], col_p[j]) = double(0);
    }
    m(row_p[k], col_p[k]) = double(1);
    for (size_t j = k + 1; j < _numColumns; ++j) {
      m(row_p[k], col_p[j]) /= quotient;
    }
  }
  
  // subtract rows such that 0's appear above leading row elements
  for (size_t k = rank - 1; k != -1; --k) {
    for (size_t i = 0; i < k; ++i) {
      double factor = m(row_p[i], col_p[k]);
      m(row_p[i], col_p[k]) = double(0);
      for (size_t j = k + 1; j < _numColumns; ++j) {
        m(row_p[i], col_p[j]) -= factor * m(row_p[k], col_p[j]);
      }
    } 
  }

  // copy m into smaller matrix C and swap columns
  Matrix<(_numRows < _numColumns ? _numRows : _numColumns), _numColumns> C;
  for (size_t k = 0; k < rank; ++k) {
    for (size_t j = 0; j < _numColumns; ++j) {
      C(k,j) = m(row_p[k], j);
    }
  }
  for (size_t k = rank; k < C.numRows(); ++k) {
    for (size_t j = 0; j < _numColumns; ++j) {
      C(k,j) = double(0);
    }
  }

  // B is copy of A with columns swapped and non-pivot columns left out
  Matrix<_numRows, (_numRows < _numColumns ? _numRows : _numColumns)> B;
  for (size_t k = 0; k < _numRows; ++k) {
    for (size_t j = 0; j < rank; ++j ) {
      B(k,j) = q(k, col_p[j]);
    }
    for (size_t j = rank; j < B.numColumns(); ++j) {
      B(k,j) = double(0);
    }
  }

  // Now, q = B*C and B and C are of full row/column rank
  return ~C*!((~B*B)*(C*~C))*~B;
}*/

// Matrix inverse -- consider using A%B (A\B) instead of (!A)*B
template <size_t _size>
inline Matrix<_size, _size> operator!(const Matrix<_size, _size>& q) { 
  Matrix<_size, _size> m(q);
  Matrix<_size, _size> inv = eye<_size>();
  
  size_t row_p[_size];
  size_t col_p[_size];
  for (size_t i = 0; i < _size; ++i) {
    row_p[i] = i; col_p[i] = i;
  }

  // Gaussian elimination
  for (size_t k = 0; k < _size; ++k) {
    // find maximal pivot element
    double maximum = double(0); size_t max_row = k; size_t max_col = k;
    for (size_t i = k; i < _size; ++i) {
      for (size_t j = k; j < _size; ++j) {
        double abs_ij = abs(m(row_p[i], col_p[j]));
        if (abs_ij > maximum) {
          maximum = abs_ij; max_row = i; max_col = j;
        }
      }
    }

    // swap rows and columns
    size_t swap = row_p[k]; row_p[k] = row_p[max_row]; row_p[max_row] = swap;
           swap = col_p[k]; col_p[k] = col_p[max_col]; col_p[max_col] = swap;

    // eliminate column
    assert(maximum != double(0));
    for (size_t i = k + 1; i < _size; ++i) {
      double factor = m(row_p[i], col_p[k]) / m(row_p[k], col_p[k]);
      
      for (size_t j = k + 1; j < _size; ++j) {
        m(row_p[i], col_p[j]) -= factor * m(row_p[k], col_p[j]);
      }
      
      for (size_t j = 0; j < k; ++j) {
        inv(row_p[i], row_p[j]) -= factor * inv(row_p[k], row_p[j]);
      }
      inv(row_p[i], row_p[k]) = -factor;
    } 
  }

  // Backward substitution
  for (size_t k = _size - 1; k != -1; --k) {
    double quotient = m(row_p[k], col_p[k]);
    
    for (size_t j = 0; j < _size; ++j) {
      inv(row_p[k], j) /= quotient;
    }
    
    for (size_t i = 0; i < k; ++i) {
      double factor = m(row_p[i], col_p[k]);
      for (size_t j = 0; j < _size; ++j) {
        inv(row_p[i], j) -= factor * inv(row_p[k], j);
      }
    }
  }

  // reshuffle result
  for (size_t i = 0; i < _size; ++i) {
    for (size_t j = 0; j < _size; ++j) {
      m(col_p[i], j) = inv(row_p[i], j);
    }
  }

  return m; 
}

template <size_t _size>
inline void jacobi(const Matrix<_size, _size>& m, Matrix<_size, _size>& V, Matrix<_size, _size>& D) {
  D = m;
  V = eye<_size>();

  if (_size <= 1) {
    return;
  }

  size_t pivotRow = 0;
  size_t zeroCount = 0;

  while (true) {
    double maximum = 0;
    size_t p, q;
    for (size_t i = 0; i < pivotRow; ++i) {
      if (abs(D(i,pivotRow)) > maximum) {
        maximum = abs(D(i,pivotRow));
        p = i; q = pivotRow;
      }
    }
    for (size_t j = pivotRow + 1; j < _size; ++j) {
      if (abs(D(pivotRow,j)) > maximum) {
        maximum = abs(D(pivotRow,j));
        p = pivotRow; q = j;
      }
    }
    
    pivotRow = (pivotRow + 1) % _size;

    if (maximum <= DBL_EPSILON) {
      ++zeroCount;
      if (zeroCount == _size) {
        break;
      } else {
        continue;
      }
    } else {
      zeroCount = 0;
    }

    double theta = 0.5*(D(q,q) - D(p,p)) / D(p,q);
    double t = 1 / (abs(theta) + hypot(theta, 1));
    if (theta < 0) t = -t;
    double c = 1 / hypot(t, 1);
    double s = c*t;
    double tau = s / (1 + c);

    // update D // 
    
    // update D(r,p) and D(r,q)
    for (size_t r = 0; r < p; ++r) {
      double Drp = D(r,p); double Drq = D(r,q);
      D(r,p) -= s*(Drq + tau*Drp);
      D(r,q) += s*(Drp - tau*Drq);
    }
    for (size_t r = p + 1; r < q; ++r) {
      double Drp = D(p,r); double Drq = D(r,q);
      D(p,r) -= s*(Drq + tau*Drp);
      D(r,q) += s*(Drp - tau*Drq);
    }
    for (size_t r = q + 1; r < _size; ++r) {
      double Drp = D(p,r); double Drq = D(q,r);
      D(p,r) -= s*(Drq + tau*Drp);
      D(q,r) += s*(Drp - tau*Drq);
    }

    // update D(p,p), D(q,q), and D(p,q);
    D(p,p) -= t*D(p,q);
    D(q,q) += t*D(p,q);
    D(p,q) = 0;

    // Update V
    for (size_t r = 0; r < _size; ++r) {
      double Vrp = V(r,p); double Vrq = V(r,q);
      V(r,p) -= s*(Vrq + tau*Vrp);
      V(r,q) += s*(Vrp - tau*Vrq);
    }
  }

  // clean up D
  for (size_t i = 0; i < _size - 1; ++i) {
    for (size_t j = i+1; j < _size; ++j) {
      D(j,i) = D(i,j) = 0;
    }
  }
};



// Matrix exponentiation
#define _B0 1729728e1
#define _B1 864864e1
#define _B2 199584e1
#define _B3 2772e2
#define _B4 252e2
#define _B5 1512e0
#define _B6 56e0
#define _B7 1e0
#define _NORMLIM 9.504178996162932e-1

template <size_t _size>
inline Matrix<_size, _size> exp(const Matrix<_size, _size>& q) { 
  Matrix<_size, _size> A(q);
  int s = (int) max(double(0), ceil(log(norm(A)/_NORMLIM)*M_LOG2E)); 

  A /= pow(2.0,s);
  Matrix<_size, _size> A2(A*A);
  Matrix<_size, _size> A4(A2*A2);
  Matrix<_size, _size> A6(A2*A4);
  Matrix<_size, _size> U( A*(A6*_B7 + A4*_B5 + A2*_B3 + eye<_size>()*_B1) );
  Matrix<_size, _size> V( A6*_B6 + A4*_B4 + A2*_B2 + eye<_size>()*_B0 ); 
  Matrix<_size, _size> R7 = (V - U)%(V + U);

  for (int i = 0; i < s; ++i) {
    R7 *= R7;
  }
  return R7;
}

// Input stream
template <size_t _numRows, size_t _numColumns>
inline std::istream& operator>>(std::istream& is, Matrix<_numRows, _numColumns>& q) {
  for (size_t i = 0; i < _numRows; ++i) {
    for (size_t j = 0; j < _numColumns; ++j) {
      is >> q(i,j);
    }
  }
  return is;
}

// Output stream
template <size_t _numRows, size_t _numColumns>
inline std::ostream& operator<<(std::ostream& os, const Matrix<_numRows, _numColumns>& q) {
  for (size_t i = 0; i < _numRows; ++i) {
    for (size_t j = 0; j < _numColumns; ++j) {
      os  << std::left << std::setw(13) << q(i,j) << " ";
    }
    os << std::endl;
  }
  //os << std::defaultfloat;
  return os;
}

// Hilbert matrix
template <size_t _size>
inline Matrix<_size, _size> hilbert() {
  Matrix<_size, _size> m;
  for (size_t i = 0; i < _size; ++i) {
    for (size_t j = 0; j < _size; ++j) {
      m(i, j) = double(1) / (double) (i + j + 1);
    }
  }
  return m;
}

// Solution to Continuous-time Algebraic Ricatti Equation ~AX + XA - XGX + Q = 0
template <size_t _size>
inline Matrix<_size, _size> care(const Matrix<_size, _size>& A, const Matrix<_size, _size>& G, const Matrix<_size, _size>& Q) {
  Matrix<2*_size, 2*_size> H, H2, W, SqrtH2;
  H.insert(0,0, A);      H.insert(0, _size, -G);
  H.insert(_size,0, -Q); H.insert(_size, _size, -~A);

  H2 = H*H;

  SqrtH2 = eye<2*_size>();
  for (size_t i = 0; i < 100; ++i) {
    SqrtH2 = 0.5*(SqrtH2 + !SqrtH2*H2);
  }

  W = H - SqrtH2;

  return W.subMatrix<_size, _size>(_size, 0) * !W.subMatrix<_size, _size>(0, 0);
}


// Given a positive-definite symmetric matrix q, construct its Cholesky decomposition, q = z.z'
// From: Numerical recipes in C++ (3rd edition)
template <size_t _size>
inline void chol(const Matrix<_size, _size>& q, Matrix<_size, _size>& z)
{
	z = q;
	int n = z.numRows();
	int i,j,k;
	double sum;
	if (z.numColumns() != n) {
		std::cerr << "Need square matrix" << std::endl;
		std::exit(-1);
	}
	for (i = 0; i < n; i++) {
		for (j = i; j < n; j++) {
			for (sum = z(i,j),k = i-1; k >= 0; k--) { 
				sum -= z(i,k)*z(j,k);
			}
			if (i == j) {
				// Matrix, with rounding errors, is not positive-definite.
				if (sum <= 0.0) {
					std::cerr << "Cholesky failed, matrix not positive definite" << std::endl;
					std::exit(-1);
				}
				z(i,i) = sqrt(sum);
			} else {
				z(j,i) = sum/z(i,i);
			}
		}
	}
	for (i = 0; i < n; i++) {
		for (j = 0; j < i; j++) { 
			z(j,i) = 0.;
		}
	}
}

// Eigen decomposition of a real symmetric matrix by converting to tridiagonal form followed by QL iteration
// From: Numerical recipes in C++ (3rd edition)
template <size_t _size>
inline void jacobi2(const Matrix<_size, _size>&	q, Matrix<_size, _size>& z, Matrix<_size, _size>& D) 
{
	// Initializations
	z = q;
	Matrix<_size> d, e;
	d.reset();
	e.reset();

	int l,k,j,i,m,iter;
	double scale,hh,h,g,f,s,r,p,dd,c,b,absf,absg,pfg;

	//Convert to tridiagonal form
	for (i=_size-1;i>0;i--) 
	{
		l=i-1;
		h=scale=0.0;
		if (l > 0) {
			for (k=0;k<i;k++)
				scale += abs(z(i,k));
			if (scale == 0.0)
				e[i]=z(i,l);
			else {
				for (k=0;k<i;k++) {
					z(i,k) /= scale;
					h += z(i,k)*z(i,k);
				}
				f=z(i,l);
				g=(f >= 0.0 ? -sqrt(h) : sqrt(h));
				e[i]=scale*g;
				h -= f*g;
				z(i,l)=f-g;
				f=0.0;
				for (j=0;j<i;j++) {
					z(j,i)=z(i,j)/h;
					g=0.0;
					for (k=0;k<j+1;k++)
						g += z(j,k)*z(i,k);
					for (k=j+1;k<i;k++)
						g += z(k,j)*z(i,k);
					e[j]=g/h;
					f += e[j]*z(i,j);
				}
				hh=f/(h+h);
				for (j=0;j<i;j++) {
					f=z(i,j);
					e[j]=g=e[j]-hh*f;
					for (k=0;k<j+1;k++)
						z(j,k) -= (f*e[k]+g*z(i,k));
				}
			}
		} else {
			e[i]=z(i,l);
		}
		d[i]=h;
	}
	d[0]=0.0;

	e[0]=0.0;
	for (i=0;i<_size;i++) {
		if (d[i] != 0.0) {
			for (j=0;j<i;j++) {
				g=0.0;
				for (k=0;k<i;k++)
					g += z(i,k)*z(k,j);
				for (k=0;k<i;k++)
					z(k,j) -= g*z(k,i);
			}
		}
		d[i]=z(i,i);
		z(i,i)=1.0;
		for (j=0;j<i;j++) z(j,i)=z(i,j)=0.0;
	}

	// QL iteration
	for (i=1;i<_size;i++) e[i-1]=e[i];
	e[_size-1]=0.0;
	for (l=0;l<_size;l++) {
		iter=0;
		do {
			for (m=l;m<_size-1;m++) {
				dd=abs(d[m])+abs(d[m+1]);
				if (abs(e[m]) <= std::numeric_limits<double>::epsilon()*dd) break;
			}
			if (m != l) {
				if (iter++ == 30) { 
					std::cerr << "Too many iterations in tqli" << std::endl;
					std::exit(-1);
				}
				g=(d[l+1]-d[l])/(2.0*e[l]);
				absg=abs(g);
				r = ( (absg > 1.0) ? absg*sqrt(1.0+(1.0/absg)*(1.0/absg)) : sqrt(1.0+absg*absg));
				g=d[m]-d[l]+e[l]/(g+((g>=0.0) ? fabs(r):-fabs(r)));
				s=c=1.0;
				p=0.0;
				for (i=m-1;i>=l;i--) {
					f=s*e[i];
					b=c*e[i];
					absf=abs(f);
					absg=abs(g);
					pfg = (absf > absg ? absf*sqrt(1.0+(absg/absf)*(absg/absf)) : (absg == 0.0 ? 0.0 : absg*sqrt(1.0+(absf/absg)*(absf/absg))));
					e[i+1]=(r=pfg); 
					if (r == 0.0) {
						d[i+1] -= p;
						e[m]=0.0;
						break;
					}
					s=f/r;
					c=g/r;
					g=d[i+1]-p;
					r=(d[i]-g)*s+2.0*c*b;
					d[i+1]=g+(p=s*r);
					g=c*r-b;
					for (k=0;k<_size;k++) {
						f=z(k,i+1);
						z(k,i+1)=s*z(k,i)+c*f;
						z(k,i)=c*z(k,i)-s*f;
					}
				}
				if (r == 0.0 && i >= l) continue;
				d[l] -= p;
				e[l]=g;
				e[m]=0.0;
			}
		} while (m != l);
	}
	
	// Sort eigenvalues and reorder eigenvectors
	/*
	for (int i=0;i<_size-1;i++) {
		double p=d[k=i];
		for (int j=i;j<_size;j++)
			if (d[j] >= p) p=d[k=j];
		if (k != i) {
			d[k]=d[i];
			d[i]=p;
			for (int j=0;j<_size;j++) {
				p=z(j,i);
				z(j,i)=z(j,k);
				z(j,k)=p;
			}
		}
	}
	*/

	//Copy over data
	D.reset();
	for(size_t i=0;i<_size;++i) {
		D(i,i) = d[i];
	}
}

template <size_t _numRows1, size_t _numRows2, size_t _numCols1, size_t _numCols2>
inline Matrix<_numRows1 + _numRows2, _numCols1 + _numCols2> block(const Matrix<_numRows1, _numCols1> A, const Matrix<_numRows1, _numCols2> B,
                                                                  const Matrix<_numRows2, _numCols1> C, const Matrix<_numRows2, _numCols2> D) {
  Matrix<_numRows1 + _numRows2, _numCols1 + _numCols2> m;
  m.insert(0,0, A);          m.insert(0,_numCols1, B);
  m.insert(_numRows1, 0, C); m.insert(_numRows1, _numCols1, D);
  return m;
}

template <size_t _size>
inline Matrix<_size, _size> sqrt(const Matrix<_size, _size>& q) 
{
	Matrix<_size, _size> V, D;
	jacobi2(q, V, D);
	for (size_t i = 0; i < _size; ++i) {
		if (D(i,i) > 0.0) {
			D(i,i) = sqrt(D(i,i));
    } else {
      D(i,i) = 0;
    }
	}
	return V*D*~V;
}


template <size_t _size>
inline void forcePD(Matrix<_size, _size>& q) 
{
	Matrix<_size, _size> evec, eval;
	jacobi2(q, evec, eval);
	for (size_t i = 0; i < _size; ++i) {
		if (eval(i,i) < 0.0) {
			eval(i,i) = 0.0;
		}
	}
	q = evec * eval * ~evec;
}

template <size_t _numRows, size_t _numColumns>
inline void serializeMatrix(std::ofstream& ofs, const Matrix<_numRows, _numColumns>& q) 
{
	for (size_t i = 0; i < q.numRows(); ++i) {
		for (size_t j = 0; j < q.numColumns(); ++j) {
			ofs << std::setprecision(12) << " " << q(i,j);
		}
	}
}

template <size_t _numRows, size_t _numColumns>
inline void deserializeMatrix(std::ifstream& ifs, Matrix<_numRows, _numColumns>& q) 
{
	for (size_t i = 0; i < q.numRows(); ++i) {
		for (size_t j = 0; j < q.numColumns(); ++j) {
			ifs >> q(i,j);
		}
	}
}

template <size_t _numRows, size_t _numColumns>
inline void checkNaN(const Matrix<_numRows, _numColumns>& q) 
{
	for (size_t i = 0; i < q.numRows(); ++i) {
		for (size_t j = 0; j < q.numColumns(); ++j) {
			if (q(i,j) != q(i,j)) {
				std::cerr << "NaN values encountered in matrix" << std::endl;
				std::exit(-1);
			}
		}
	}
}


#endif
