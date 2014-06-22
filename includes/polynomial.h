#ifndef __POLYNOMIAL_H__
#define __POLYNOMIAL_H__

#include <cmath>
#include <assert.h>
#include <iostream>
#include <algorithm>
#include <complex>
#include <vector>

template <size_t _degree, class _Ty = double> class Polynomial {
private:
  _Ty _coeffs[_degree + 1];

public:

  inline size_t degree() const {
    return _degree;
  }

  // access to coefficients
  inline _Ty& operator [] (size_t coeff) {
    assert(coeff <= _degree);
    return _coeffs[coeff]; 
  }
  inline _Ty  operator [] (size_t coeff) const {
    assert(coeff <= _degree);
    return _coeffs[coeff];
  }

  // evaluation
  inline _Ty operator () (double x) const {
    _Ty y = _coeffs[_degree];
    for (size_t j = _degree - 1; j != -1; --j) {
      y = y*x + _coeffs[j];
    }
    return y;
  }
  inline std::complex<double> operator () (const std::complex<double>& x) const {
    std::complex<double> y = _coeffs[_degree];
    for (size_t j = _degree - 1; j != -1; --j) {
      y = y*x + _coeffs[j];
    }
    return y;
  }

  // Taylor expansion about x up to order
  template <size_t order>
  inline Polynomial<order, _Ty> taylor(double x) const {
    Polynomial<order, _Ty> f;

    f[0] = _coeffs[_degree];
    for (size_t j = 1; j <= order; ++j) {
      f[j] = _Ty(0);
    }

    for (size_t j = _degree - 1; j != -1; --j) {
      for (size_t i = (order < _degree - j ? order : _degree - j); i > 0; --i) {
        f[i] = f[i]*x + f[i - 1];
      }
      f[0] = f[0]*x + _coeffs[j];
    }
    return f;
  }
  
  template <size_t order>
  inline Polynomial<order, std::complex<double> > taylor(const std::complex<double>& x) const {
    Polynomial<order, std::complex<double> > f;

    f[0] = _coeffs[_degree];
    for (size_t j = 1; j <= order; ++j) {
      f[j] = std::complex<double>(0);
    }

    for (size_t j = _degree - 1; j != -1; --j) {
      for (size_t i = (order < _degree - j ? order : _degree - j); i > 0; --i) {
        f[i] = f[i]*x + f[i - 1];
      }
      f[0] = f[0]*x + _coeffs[j];
    }
    return f;
  }

  inline Polynomial<1, _Ty> linearize(double x) const {
    return taylor<1>(x);
  }
  inline Polynomial<1, std::complex<double> > linearize(const std::complex<double>& x) const {
    return taylor<1>(x);
  }

  inline Polynomial<2, _Ty> quadratize(double x) const {
    return taylor<2>(x);
  }
  inline Polynomial<2, std::complex<double> > quadratize(const std::complex<double>& x) const {
    return taylor<2>(x);
  }

  // derivative
  inline Polynomial<(_degree > 0 ? _degree - 1 : 0), _Ty> derivative() const {
    Polynomial<(_degree > 0 ? _degree - 1 : 0), _Ty> f;
    if (_degree == 0) {
      f[0] = _Ty(0);
    } else {
      for (size_t j = 0; j <= _degree - 1; ++j) {
        f[j] = _coeffs[j + 1] * (j + 1);
      }
    }
    return f;
  }

  // nd'th derivative
  template <size_t nd>
  inline Polynomial<(_degree > nd ? _degree - nd : 0), _Ty> derivative() const {
    Polynomial<(_degree > nd ? _degree - nd : 0), _Ty> f;
    if (nd > _degree) {
      f[0] = _Ty(0);
    } else {
      double factor = 1;
      for (size_t j = 2; j <= nd; ++j) {
        factor *= j;
      }
      f[0] = _coeffs[nd] * factor;
      for (size_t j = 1; j <= _degree - nd; ++j) {
        factor /= j;
        factor *= (j + nd);
        f[j] = _coeffs[j + nd] * factor;
      }
    }
    return f;
  }

  // integral
  inline Polynomial<_degree + 1, _Ty> integral() const {
    Polynomial<_degree + 1, _Ty> f;
    f[0] = _Ty(0);
    for (size_t j = 1; j <= _degree + 1; ++j) {
      f[j] = _coeffs[j - 1] / j;
    }
    return f;
  }

  // nd'th integral
  template <size_t nd>
  inline Polynomial<_degree + nd, _Ty> integral() const {
    Polynomial<_degree + nd, _Ty> f;
    double factor = 1;
    for (size_t j = 0; j < nd; ++j) {
      f[j] = _Ty(0);
      factor *= (j + 1);
    }

    f[nd] = _coeffs[0] / factor;
    for (size_t j = nd + 1; j <= _degree + nd; ++j) {
      factor /= (j - nd);
      factor *= j;
      f[j] = _coeffs[j - nd] / factor;
    }
    return f;
  }

  // Unary plus
  inline const Polynomial<_degree, _Ty>& operator+() const { 
    return *this; 
  }

  // Unary minus
  inline Polynomial<_degree, _Ty> operator-() const { 
    Polynomial<_degree> f;
    for (size_t j = 0; j <= _degree; ++j) {
      f[j] = -_coeffs[j];
    }
    return f;
  }

  // Equality
  inline bool operator==(const Polynomial<_degree, _Ty>& g) const { 
    for (size_t j = 0; j <= _degree; ++j) {
      if (_coeffs[j] != g[j]) {
        return false;
      }
    }
    return true;
  }

  // Inequality
  inline bool operator!=(const Polynomial<_degree, _Ty>& g) const { 
    for (size_t j = 0; j <= _degree; ++j) {
      if (_coeffs[j] == g[j]) {
        return false;
      }
    }
    return true;
  }

  // Polynomial addition
  template <size_t degree>
  inline Polynomial<(_degree > degree ? _degree : degree), _Ty> operator+(const Polynomial<degree, double>& g) const {
    Polynomial<(_degree > degree ? _degree : degree), _Ty> f;
    for (size_t j = 0; j <= (_degree < degree ? _degree : degree); ++j) {
      f[j] = _coeffs[j] + g[j];
    }
    for (size_t j = (_degree < degree ? _degree : degree) + 1; j <= (_degree > degree ? _degree : degree); ++j) {
      f[j] = (_degree > degree ? _coeffs[j] : g[j]);
    }
    return f;
  }

  template <size_t degree>
  inline Polynomial<(_degree > degree ? _degree : degree), std::complex<double> > operator+(const Polynomial<degree, std::complex<double> >& g) const {
    Polynomial<(_degree > degree ? _degree : degree), std::complex<double> > f;
    for (size_t j = 0; j <= (_degree < degree ? _degree : degree); ++j) {
      f[j] = _coeffs[j] + g[j];
    }
    for (size_t j = (_degree < degree ? _degree : degree) + 1; j <= (_degree > degree ? _degree : degree); ++j) {
      f[j] = (_degree > degree ? _coeffs[j] : g[j]);
    }
    return f;
  }

  // Polynomial subtraction
  template <size_t degree>
  inline Polynomial<(_degree > degree ? _degree : degree), _Ty> operator-(const Polynomial<degree, double>& g) const {
    Polynomial<(_degree > degree ? _degree : degree), _Ty> f;
    for (size_t j = 0; j <= (_degree < degree ? _degree : degree); ++j) {
      f[j] = _coeffs[j] - g[j];
    }
    for (size_t j = (_degree < degree ? _degree : degree) + 1; j <= (_degree > degree ? _degree : degree); ++j) {
      f[j] = (_degree > degree ? _coeffs[j] : -g[j]);
    }
    return f;
  }

  template <size_t degree>
  inline Polynomial<(_degree > degree ? _degree : degree), std::complex<double> > operator-(const Polynomial<degree, std::complex<double> >& g) const {
    Polynomial<(_degree > degree ? _degree : degree), std::complex<double> > f;
    for (size_t j = 0; j <= (_degree < degree ? _degree : degree); ++j) {
      f[j] = _coeffs[j] - g[j];
    }
    for (size_t j = (_degree < degree ? _degree : degree) + 1; j <= (_degree > degree ? _degree : degree); ++j) {
      f[j] = (_degree > degree ? _coeffs[j] : -g[j]);
    }
    return f;
  }
  
  // Scalar multiplication
  inline Polynomial<_degree, _Ty> operator*(double a) const { 
    Polynomial<_degree, _Ty> f;
    for (size_t j = 0; j <= _degree; ++j) {
      f[j] = _coeffs[j] * a;
    }
    return f;
  }
  inline Polynomial<_degree, std::complex<double> > operator*(const std::complex<double>& a) const { 
    Polynomial<_degree, std::complex<double> > f;
    for (size_t j = 0; j <= _degree; ++j) {
      f[j] = _coeffs[j] * a;
    }
    return f;
  }

  inline const Polynomial<_degree, _Ty>& operator*=(const _Ty& a) { 
    for (size_t j = 0; j <= _degree; ++j) {
      _coeffs[j] *= a;
    }
    return *this;
  }

  // Scalar division
  inline Polynomial<_degree, _Ty> operator/(double a) const { 
    Polynomial<_degree, _Ty> f;
    for (size_t j = 0; j <= _degree; ++j) {
      f[j] = _coeffs[j] / a;
    }
    return f;
  }
  inline Polynomial<_degree, std::complex<double> > operator/(const std::complex<double>& a) const { 
    Polynomial<_degree, std::complex<double> > f;
    for (size_t j = 0; j <= _degree; ++j) {
      f[j] = _coeffs[j] / a;
    }
    return f;
  }

  inline const Polynomial<_degree, _Ty>& operator/=(const _Ty& a) { 
    for (size_t j = 0; j <= _degree; ++j) {
      _coeffs[j] /= a;
    }
    return *this;
  }
  
  // Polynomial multiplication
  template <size_t degree>
  inline Polynomial<_degree + degree, _Ty> operator*(const Polynomial<degree, double>& g) const {
    Polynomial<_degree + degree, _Ty> f;
    for (size_t i = 0; i <= _degree + degree; ++i) {
      f[i] = _Ty(0);
      for (size_t j = (i > degree ? i - degree : 0); j <= (i < _degree ? i : _degree); ++j) {
        f[i] += _coeffs[j] * g[i - j];
      }
    }
    return f;
  }

  template <size_t degree>
  inline Polynomial<_degree + degree, std::complex<double> > operator*(const Polynomial<degree, std::complex<double> >& g) const {
    Polynomial<_degree + degree, std::complex<double> > f;
    for (size_t i = 0; i <= _degree + degree; ++i) {
      f[i] = std::complex<double>(0);
      for (size_t j = (i > degree ? i - degree : 0); j <= (i < _degree ? i : _degree); ++j) {
        f[i] += _coeffs[j] * g[i - j];
      }
    }
    return f;
  }

  // Polynomial division (returns quotient)
  template <size_t degree>
  inline Polynomial<(_degree > degree ? _degree - degree : 0), _Ty> operator/(const Polynomial<degree, double>& v) const {
    assert(v[degree] != double(0));

    Polynomial<(_degree > degree ? _degree - degree : 0), _Ty> q;
    
    if (degree > _degree) {
      q[0] = _Ty(0);
    } else if (degree == 0) {
      for (size_t i = 0; i <= _degree; ++i) {
        q[i] = _coeffs[i] / v[0];
      }
    } else {
      Polynomial<degree - 1, _Ty> r;
      for (size_t k = _degree; k != _degree - degree; --k) {
        r[k - (_degree - degree) - 1] = _coeffs[k]; 
      }
      for (size_t k = _degree - degree; k != -1; --k) {
        q[k] = r[degree - 1] / v[degree];
        for (size_t j = degree - 1; j > 0; --j) {
          r[j] = r[j - 1] - q[k]*v[j];
        }
        r[0] = _coeffs[k] - q[k]*v[0];
      }
    }
        
    return q;
  }

  template <size_t degree>
  inline Polynomial<(_degree > degree ? _degree - degree : 0), std::complex<double> > operator/(const Polynomial<degree, std::complex<double> >& v) const {
    assert(v[degree] != std::complex<double>(0));

    Polynomial<(_degree > degree ? _degree - degree : 0), std::complex<double> > q;
    
    if (degree > _degree) {
      q[0] = std::complex<double>(0);
    } else if (degree == 0) {
      for (size_t i = 0; i <= _degree; ++i) {
        q[i] = _coeffs[i] / v[0];
      }
    } else {
      Polynomial<degree - 1, std::complex<double> > r;
      for (size_t k = _degree; k != _degree - degree; --k) {
        r[k - (_degree - degree) - 1] = _coeffs[k]; 
      }
      for (size_t k = _degree - degree; k != -1; --k) {
        q[k] = r[degree - 1] / v[degree];
        for (size_t j = degree - 1; j > 0; --j) {
          r[j] = r[j - 1] - q[k]*v[j];
        }
        r[0] = _coeffs[k] - q[k]*v[0];
      }
    }
        
    return q;
  }

  // Polynomial division (returns remainder)
  template <size_t degree>
  inline Polynomial<(degree > 0 ? (degree > _degree ? degree : degree - 1) : 0), _Ty> operator%(const Polynomial<degree, double>& v) const {
    Polynomial<(degree > 0 ? (degree > _degree ? degree : degree - 1) : 0), _Ty> r;
    
    if (degree == 0) {
      r[0] = _Ty(0);
    } else if (degree > _degree) {
      for (size_t i = 0; i <= degree; ++i) {
        r[i] = v[i];
      }
    } else {
      assert(v[degree] != double(0));
      for (size_t k = _degree; k != _degree - degree; --k) {
        r[k - (_degree - degree) - 1] = _coeffs[k];
      }
      for (size_t k = _degree - degree; k != -1; --k) {
        _Ty q = r[degree - 1] / v[degree];
        for (size_t j = degree - 1; j > 0; --j) {
          r[j] = r[j - 1] - q*v[j];
        }
        r[0] = _coeffs[k] - q*v[0];
      }
    }

    return r;
  }

  template <size_t degree>
  inline Polynomial<(degree > 0 ? (degree > _degree ? degree : degree - 1) : 0), std::complex<double> > operator%(const Polynomial<degree, std::complex<double> >& v) const {
    Polynomial<(degree > 0 ? (degree > _degree ? degree : degree - 1) : 0), std::complex<double> > r;
    
    if (degree == 0) {
      r[0] = std::complex<double>(0);
    } else if (degree > _degree) {
      for (size_t i = 0; i <= degree; ++i) {
        r[i] = v[i];
      }
    } else {
      assert(v[degree] != std::complex<double>(0));
      for (size_t k = _degree; k != _degree - degree; --k) {
        r[k - (_degree - degree) - 1] = _coeffs[k];
      }
      for (size_t k = _degree - degree; k != -1; --k) {
        std::complex<double> q = r[degree - 1] / v[degree];
        for (size_t j = degree - 1; j > 0; --j) {
          r[j] = r[j - 1] - q*v[j];
        }
        r[0] = _coeffs[k] - q*v[0];
      }
    }

    return r;
  }
};

// Input stream
template <size_t _degree, class _Ty>
inline std::istream& operator>>(std::istream& is, Polynomial<_degree, _Ty>& q) {
  for (size_t j = 0; j <= _degree; ++j) {
    is >> q[j];
  }
  return is;
}

// Output stream
template <size_t _degree>
inline std::ostream& operator<<(std::ostream& os, const Polynomial<_degree, double>& q) {
  size_t j = 0;
  while (j < _degree && q[j] == double(0)) {
    ++j;
  }
  if (j > 0 && q[j] == double(-1)) {
    os << "-";
  } else if (j == 0 || q[j] != double(1)) {
    os << q[j];
  }
  if (j == 1 && q[j] != double(0)) {
    os << "x";
  } else if (j > 1 && q[j] != double(0)) {
    os << "x^" << j;
  }

  for (j = j + 1; j <= _degree; ++j) {
    if (q[j] < 0) {
      os << " - ";
    } else if (q[j] > 0) {
      os << " + ";
    }
    if (q[j] != double(1) && q[j] != double(-1) && q[j] != double(0)) {
      os << (q[j] < 0 ? -q[j] : q[j]);
    }
    if (q[j] != double(0) && j == 1) {
      os << "x";
    } else if (q[j] != double(0)) {
      os << "x^" << j;
    }
  }
  //os << std::defaultfloat;
  return os;
}

template <size_t _degree>
inline std::ostream& operator<<(std::ostream& os, const Polynomial<_degree, std::complex<double> >& q) {
  os << q[0];

  for (size_t j = 1; j <= _degree; ++j) {
    os << " + " << q[j];
    if (j == 1) {
      os << "x";
    } else {
      os << "x^" << j;
    }
  }
  //os << std::defaultfloat;
  return os;
}

// Scalar multiplication
template <size_t _degree, class _Ty>
inline Polynomial<_degree, _Ty> operator*(const _Ty& a, const Polynomial<_degree, _Ty>& g) { 
  Polynomial<_degree, _Ty> f;
  for (size_t j = 0; j <= _degree; ++j) {
    f[j] = a * g[j];
  }
  return f;
}

template <size_t _degree>
inline Polynomial<_degree, std::complex<double> > operator*(const std::complex<double>& a, const Polynomial<_degree, double>& g) { 
  Polynomial<_degree, std::complex<double> > f;
  for (size_t j = 0; j <= _degree; ++j) {
    f[j] = a * g[j];
  }
  return f;
}


inline void roots(const Polynomial<0, double>& g, std::vector<double>& realRoots, std::vector<std::complex<double> >& complexRoots) {
#ifdef _POLY_DEBUG
	std::cout << g << std::endl;
#endif
  return;
}
inline void roots(const Polynomial<1, double>& g, std::vector<double>& realRoots, std::vector<std::complex<double> >& complexRoots) {
#ifdef _POLY_DEBUG
	std::cout << g << std::endl;
#endif
  if (g[1] == 0.0) {
    return;
  }
  realRoots.push_back(-g[0] / g[1]);
}
inline void roots(const Polynomial<2, double>& g, std::vector<double>& realRoots, std::vector<std::complex<double> >& complexRoots) {
#ifdef _POLY_DEBUG
  std::cout << g << std::endl;
#endif
  if (g[2] == 0.0) {
    Polynomial<1> f;
    f[0] = g[0]; f[1] = g[1];
    roots(f, realRoots, complexRoots);
    return;
  } 
  double discSq = g[1]*g[1] - 4.0*g[2]*g[0];
  if (discSq >= 0.0) {
    double d = sqrt(discSq);
    double q;
    if (g[1] >= 0.0) {
      q = -0.5*(g[1] + d);
    } else {
      q = -0.5*(g[1] - d);
    }
    if (q == 0.0) {
      realRoots.push_back( 0.0 );
      realRoots.push_back( 0.0 );
    } else {
      realRoots.push_back( q / g[2] );
      realRoots.push_back( g[0] / q );
    }
  } else {
    std::complex<double> d = sqrt(std::complex<double>(discSq));
    std::complex<double> q;
    if (g[1] >= 0) {
      q = -0.5*(g[1] + d);
    } else {
      q = -0.5*(g[1] - d);
    }
    complexRoots.push_back( q / g[2] );
    complexRoots.push_back( g[0] / q );
  }
}

template <size_t _degree>
inline void roots(const Polynomial<_degree, double>& g, std::vector<double>& realRoots, std::vector<std::complex<double> >& complexRoots) {
#ifdef _POLY_DEBUG
  std::cout << g << std::endl;
#endif
  if (g[_degree] == 0.0) {
    Polynomial<_degree - 1> g_reduced;
    for (size_t i = 0; i < _degree; ++i) {
      g_reduced[i] = g[i];
    }
    roots(g_reduced, realRoots, complexRoots);
    return;
  } 

  std::complex<double> h;
  std::complex<double> x(0.0929);
  bool doubleRoot = false;
  size_t iter = 0;
  do {
    Polynomial<2, std::complex<double> > f = g.quadratize(x);
#ifdef _POLY_DEBUG
	std::cout << f << std::endl;
#endif
    //if (abs(f[1]) < DBL_EPSILON && abs(f[2]) < DBL_EPSILON) { // no root, find root using higher-derivative.
    std::complex<double> d = sqrt(f[1]*f[1] - 4.0*f[2]*f[0]);
    std::complex<double> q;
    if (abs(f[1] + d) >= abs(f[1] - d)) {
      q = -0.5*(f[1] + d);
    } else { 
      q = -0.5*(f[1] - d);
    }
    if (abs(q) == 0.0 && abs(f[0]) == 0.0) {
      doubleRoot = true;
      h = std::complex<double>(0);
    } else {
      h = f[0] / q;
      x += h;
    }
#ifdef _POLY_DEBUG
    std::cout << "    " << x << " " << abs(h) << std::endl;
#endif
	iter++;
  //} while (!(abs(h) < DBL_EPSILON || (abs(x) != 0.0 && abs(h)/abs(x) < DBL_EPSILON)));
  } while (!(abs(h) == 0.0 || (abs(x) != 0.0 && abs(h)/abs(x) < DBL_EPSILON) || (iter == 20))); 
  if (doubleRoot) {
    realRoots.push_back(x.real());
    realRoots.push_back(x.real());

    Polynomial<2> factor;
    factor[0] = x.real()*x.real(); factor[1] = -2.0*x.real(); factor[2] = 1.0;
    roots(g / factor, realRoots, complexRoots);
  } else if (x.imag() == 0.0 ||		abs(x.imag())/abs(x) < (DBL_EPSILON > abs(h) ? DBL_EPSILON : abs(h))) {
    // real root
    realRoots.push_back(x.real());

    Polynomial<1> factor;
    factor[0] = -x.real(); factor[1] = 1;
    roots(g / factor, realRoots, complexRoots);
  } else {
    // complex root, real coefficient-polynomial
    complexRoots.push_back(x);
    complexRoots.push_back(conj(x));

    Polynomial<2> factor;
    factor[0] = x.real()*x.real() + x.imag()*x.imag(); factor[1] = -2*x.real(); factor[2] = 1.0;
    roots(g / factor, realRoots, complexRoots);
  }
}

#endif