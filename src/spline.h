/*
 * spline.h
 *
 * simple cubic spline interpolation library without external
 * dependencies
 *
 * ---------------------------------------------------------------------
 * Copyright (C) 2011, 2014 Tino Kluge (ttk448 at gmail.com)
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * ---------------------------------------------------------------------
 *
 */


#ifndef TK_SPLINE_H
#define TK_SPLINE_H

#include <cstdio>
#include <cassert>
#include <vector>
#include <algorithm>
#include <array>
#include <math.h>
#include <boost/math/quadrature/gauss.hpp>
#include <boost/math/quadrature/gauss_kronrod.hpp>

using namespace boost::math::quadrature;

// unnamed namespace only because the implementation is in this
// header file and we don't want to export symbols to the obj files
namespace
{

namespace tk
{

// band matrix solver
class band_matrix
{
private:
    std::vector< std::vector<double> > m_upper;  // upper band
    std::vector< std::vector<double> > m_lower;  // lower band
public:
    band_matrix() {};                             // constructor
    band_matrix(int dim, int n_u, int n_l);       // constructor
    ~band_matrix() {};                            // destructor
    void resize(int dim, int n_u, int n_l);      // init with dim,n_u,n_l
    int dim() const;                             // matrix dimension
    int num_upper() const
    {
        return m_upper.size()-1;
    }
    int num_lower() const
    {
        return m_lower.size()-1;
    }
    // access operator
    double & operator () (int i, int j);            // write
    double   operator () (int i, int j) const;      // read
    // we can store an additional diogonal (in m_lower)
    double& saved_diag(int i);
    double  saved_diag(int i) const;
    void lu_decompose();
    std::vector<double> r_solve(const std::vector<double>& b) const;
    std::vector<double> l_solve(const std::vector<double>& b) const;
    std::vector<double> lu_solve(const std::vector<double>& b,
                                 bool is_lu_decomposed=false);

};


// spline interpolation
template <int DIMENSION=1>
class spline
{
public:
    enum bd_type {
        first_deriv = 1,
        second_deriv = 2
    };

private:
    std::vector<double> m_x;            // x,y coordinates of points
    std::vector<std::array<double, DIMENSION>> m_y;

    // interpolation parameters
    // f(x) = a*(x-x_i)^3 + b*(x-x_i)^2 + c*(x-x_i) + y_i
    std::vector<std::array<double, DIMENSION>> m_a,m_b,m_c;        // spline coefficients
    std::array<double, DIMENSION>  m_b0, m_c0;                     // for left extrapol
    std::array<bd_type, DIMENSION> m_left, m_right;
    std::array<double, DIMENSION>  m_left_value, m_right_value;
    bool    m_force_linear_extrapolation;


    void set_coefficients(bool cubic_spline=true);

public:
    // set default boundary condition to be zero curvature at both ends
    spline(): m_force_linear_extrapolation(false)
    {
      for(int d=0; d<DIMENSION; d++){
        m_left[d] = second_deriv;
        m_right[d] = second_deriv;
        m_left_value[d] = 0;
        m_right_value[d] = 0;
      }
    }

    // optional, but if called it has to come be before set_points()
    void set_boundary(std::array<bd_type, DIMENSION> left, std::array<double, DIMENSION> left_value,
                      std::array<bd_type, DIMENSION> right, std::array<double, DIMENSION> right_value,
                      bool force_linear_extrapolation=false);
    void set_points(const std::vector<double>& x,
                    const std::vector<std::array<double, DIMENSION>>& y, bool cubic_spline=true);
    // reparameterizes spline based on arc length - suitable for paths through multidimensional spaces
    void normalize_path(int start_index=0);
    std::array<double, DIMENSION> operator() (double x) const;
    // returns the maximum curvature over a given parameter range
    double max_path_curvature (double s1, double s2);

};



// ---------------------------------------------------------------------
// implementation part, which could be separated into a cpp file
// ---------------------------------------------------------------------


// band_matrix implementation
// -------------------------

band_matrix::band_matrix(int dim, int n_u, int n_l)
{
    resize(dim, n_u, n_l);
}
void band_matrix::resize(int dim, int n_u, int n_l)
{
    assert(dim>0);
    assert(n_u>=0);
    assert(n_l>=0);
    m_upper.resize(n_u+1);
    m_lower.resize(n_l+1);
    for(size_t i=0; i<m_upper.size(); i++) {
        m_upper[i].resize(dim);
    }
    for(size_t i=0; i<m_lower.size(); i++) {
        m_lower[i].resize(dim);
    }
}
int band_matrix::dim() const
{
    if(m_upper.size()>0) {
        return m_upper[0].size();
    } else {
        return 0;
    }
}


// defines the new operator (), so that we can access the elements
// by A(i,j), index going from i=0,...,dim()-1
double & band_matrix::operator () (int i, int j)
{
    int k=j-i;       // what band is the entry
    assert( (i>=0) && (i<dim()) && (j>=0) && (j<dim()) );
    assert( (-num_lower()<=k) && (k<=num_upper()) );
    // k=0 -> diogonal, k<0 lower left part, k>0 upper right part
    if(k>=0)   return m_upper[k][i];
    else	    return m_lower[-k][i];
}
double band_matrix::operator () (int i, int j) const
{
    int k=j-i;       // what band is the entry
    assert( (i>=0) && (i<dim()) && (j>=0) && (j<dim()) );
    assert( (-num_lower()<=k) && (k<=num_upper()) );
    // k=0 -> diogonal, k<0 lower left part, k>0 upper right part
    if(k>=0)   return m_upper[k][i];
    else	    return m_lower[-k][i];
}
// second diag (used in LU decomposition), saved in m_lower
double band_matrix::saved_diag(int i) const
{
    assert( (i>=0) && (i<dim()) );
    return m_lower[0][i];
}
double & band_matrix::saved_diag(int i)
{
    assert( (i>=0) && (i<dim()) );
    return m_lower[0][i];
}

// LR-Decomposition of a band matrix
void band_matrix::lu_decompose()
{
    int  i_max,j_max;
    int  j_min;
    double x;

    // preconditioning
    // normalize column i so that a_ii=1
    for(int i=0; i<this->dim(); i++) {
        assert(this->operator()(i,i)!=0.0);
        this->saved_diag(i)=1.0/this->operator()(i,i);
        j_min=std::max(0,i-this->num_lower());
        j_max=std::min(this->dim()-1,i+this->num_upper());
        for(int j=j_min; j<=j_max; j++) {
            this->operator()(i,j) *= this->saved_diag(i);
        }
        this->operator()(i,i)=1.0;          // prevents rounding errors
    }

    // Gauss LR-Decomposition
    for(int k=0; k<this->dim(); k++) {
        i_max=std::min(this->dim()-1,k+this->num_lower());  // num_lower not a mistake!
        for(int i=k+1; i<=i_max; i++) {
            assert(this->operator()(k,k)!=0.0);
            x=-this->operator()(i,k)/this->operator()(k,k);
            this->operator()(i,k)=-x;                         // assembly part of L
            j_max=std::min(this->dim()-1,k+this->num_upper());
            for(int j=k+1; j<=j_max; j++) {
                // assembly part of R
                this->operator()(i,j)=this->operator()(i,j)+x*this->operator()(k,j);
            }
        }
    }
}
// solves Ly=b
std::vector<double> band_matrix::l_solve(const std::vector<double>& b) const
{
    assert( this->dim()==(int)b.size() );
    std::vector<double> x(this->dim());
    int j_start;
    double sum;
    for(int i=0; i<this->dim(); i++) {
        sum=0;
        j_start=std::max(0,i-this->num_lower());
        for(int j=j_start; j<i; j++) sum += this->operator()(i,j)*x[j];
        x[i]=(b[i]*this->saved_diag(i)) - sum;
    }
    return x;
}
// solves Rx=y
std::vector<double> band_matrix::r_solve(const std::vector<double>& b) const
{
    assert( this->dim()==(int)b.size() );
    std::vector<double> x(this->dim());
    int j_stop;
    double sum;
    for(int i=this->dim()-1; i>=0; i--) {
        sum=0;
        j_stop=std::min(this->dim()-1,i+this->num_upper());
        for(int j=i+1; j<=j_stop; j++) sum += this->operator()(i,j)*x[j];
        x[i]=( b[i] - sum ) / this->operator()(i,i);
    }
    return x;
}

std::vector<double> band_matrix::lu_solve(const std::vector<double>& b,
        bool is_lu_decomposed)
{
    assert( this->dim()==(int)b.size() );
    std::vector<double>  x,y;
    if(is_lu_decomposed==false) {
        this->lu_decompose();
    }
    y=this->l_solve(b);
    x=this->r_solve(y);
    return x;
}




// spline implementation
// -----------------------
template <int DIMENSION>
void spline<DIMENSION>::set_boundary(std::array<bd_type, DIMENSION> left, std::array<double, DIMENSION> left_value,
                          std::array<bd_type, DIMENSION> right, std::array<double, DIMENSION> right_value,
                          bool force_linear_extrapolation)
{
    assert(m_x.size()==0);          // set_points() must not have happened yet
    m_left=left;
    m_right=right;
    m_left_value=left_value;
    m_right_value=right_value;
    m_force_linear_extrapolation=force_linear_extrapolation;
}


template <int DIMENSION>
void spline<DIMENSION>::set_coefficients(bool cubic_spline)
{
    int   n=m_x.size();
    for(int d=0; d<DIMENSION; d++){
      if(cubic_spline==true) { // cubic spline interpolation
          // setting up the matrix and right hand side of the equation system
          // for the parameters b[]
          band_matrix A(n,1,1);
          std::vector<double>  rhs(n);
          for(int i=1; i<n-1; i++) {
              A(i,i-1)=1.0/3.0*(m_x[i]-m_x[i-1]);
              A(i,i)=2.0/3.0*(m_x[i+1]-m_x[i-1]);
              A(i,i+1)=1.0/3.0*(m_x[i+1]-m_x[i]);
              rhs[i]=(m_y[i+1][d]-m_y[i][d])/(m_x[i+1]-m_x[i]) - (m_y[i][d]-m_y[i-1][d])/(m_x[i]-m_x[i-1]);
//              rhs[i]=(y[i+1][d]-y[i][d])/(x[i+1]-x[i]) - (y[i][d]-y[i-1][d])/(x[i]-x[i-1]);
          }
          // boundary conditions
          if(m_left[d] == spline::second_deriv) {
              // 2*b[0] = f''
              A(0,0)=2.0;
              A(0,1)=0.0;
              rhs[0]=m_left_value[d];
          } else if(m_left[d] == spline::first_deriv) {
              // c[0] = f', needs to be re-expressed in terms of b:
              // (2b[0]+b[1])(x[1]-x[0]) = 3 ((y[1]-y[0])/(x[1]-x[0]) - f')
              A(0,0)=2.0*(m_x[1]-m_x[0]);
              A(0,1)=1.0*(m_x[1]-m_x[0]);
              rhs[0]=3.0*((m_y[1][d]-m_y[0][d])/(m_x[1]-m_x[0])-m_left_value[d]);
          } else {
              assert(false);
          }
          if(m_right[d] == spline::second_deriv) {
              // 2*b[n-1] = f''
              A(n-1,n-1)=2.0;
              A(n-1,n-2)=0.0;
              rhs[n-1]=m_right_value[d];
          } else if(m_right[d] == spline::first_deriv) {
              // c[n-1] = f', needs to be re-expressed in terms of b:
              // (b[n-2]+2b[n-1])(x[n-1]-x[n-2])
              // = 3 (f' - (y[n-1]-y[n-2])/(x[n-1]-x[n-2]))
              A(n-1,n-1)=2.0*(m_x[n-1]-m_x[n-2]);
              A(n-1,n-2)=1.0*(m_x[n-1]-m_x[n-2]);
              rhs[n-1]=3.0*(m_right_value[d]-(m_y[n-1][d]-m_y[n-2][d])/(m_x[n-1]-m_x[n-2]));
          } else {
              assert(false);
          }

          // solve the equation system to obtain the parameters b[]
          std::vector<double> m_bd=A.lu_solve(rhs);
          m_a.resize(n);
          m_b.resize(n);
          m_c.resize(n);
          for(int i=0; i<n; i++){
            m_b[i][d] = m_bd[i];
          }
          // calculate parameters a[] and c[] based on b[]
          m_a.resize(n);
          m_c.resize(n);
          for(int i=0; i<n-1; i++) {
              m_a[i][d]=1.0/3.0*(m_b[i+1][d]-m_b[i][d])/(m_x[i+1]-m_x[i]);
              m_c[i][d]=(m_y[i+1][d]-m_y[i][d])/(m_x[i+1]-m_x[i])
                         - 1.0/3.0*(2.0*m_b[i][d]+m_b[i+1][d])*(m_x[i+1]-m_x[i]);
        }
    } else { // linear interpolation
        m_a.resize(n);
        m_b.resize(n);
        m_c.resize(n);
        for(int i=0; i<n-1; i++) {
            m_a[i][d]=0.0;
            m_b[i][d]=0.0;
            m_c[i][d]=(m_y[i+1][d]-m_y[i][d])/(m_x[i+1]-m_x[i]);
        }
    }
    // for left extrapolation coefficients
    m_b0[d] = (m_force_linear_extrapolation==false) ? m_b[0][d] : 0.0;
    m_c0[d] = m_c[0][d];

    // for the right extrapolation coefficients
    // f_{n-1}(x) = b*(x-x_{n-1})^2 + c*(x-x_{n-1}) + y_{n-1}
    double h=m_x[n-1]-m_x[n-2];
    // m_b[n-1] is determined by the boundary condition
    m_a[n-1][d]=0.0;
    m_c[n-1][d]=3.0*m_a[n-2][d]*h*h+2.0*m_b[n-2][d]*h+m_c[n-2][d];   // = f'_{n-2}(x_{n-1})
    if(m_force_linear_extrapolation==true)
        m_b[n-1][d]=0.0;
    }
}

template <int DIMENSION>
void spline<DIMENSION>::set_points(const std::vector<double>& x,
                        const std::vector<std::array<double, DIMENSION>>& y, bool cubic_spline)
{
    assert(x.size()==y.size());
    assert(x.size()>2);
    m_x=x;
    m_y=y;
    for(int i=0; i<x.size()-1; i++) {
        assert(m_x[i]<m_x[i+1]);
    }
    set_coefficients(cubic_spline);
}


template <int DIMENSION>
void spline<DIMENSION>::normalize_path(int start_index)
{
  assert(start_index >=0 && start_index < m_x.size());
  double increment = 0;
  for(int idx=start_index; idx<m_x.size()-1; idx++){
  // compute arc length of current section
    double h=m_x[idx+1]-m_x[idx]+increment;
    double pa=0;
    double pb=0;
    double pc=0;
    double pd=0;
    double pe=0;
    for(int d=0; d<DIMENSION; d++){
      pa += 9.0 * m_a[idx][d] * m_a[idx][d];
      pb += 12.0 * m_a[idx][d] * m_b[idx][d];
      pc += 6.0 * m_a[idx][d] * m_c[idx][d] + 4.0 * m_b[idx][d] * m_b[idx][d];
      pd += 4.0 * m_b[idx][d] * m_c[idx][d];
      pe += m_c[idx][d] * m_c[idx][d];
    }
    auto f1 = [&pa, &pb, &pc, &pd, &pe](const double& t) { return sqrt( pa*t*t*t*t + pb*t*t*t + pc*t*t + pd*t + pe); };
//    double arclength = gauss<double, 7>::integrate(f1, 0, h);
    double error;
    double arclength = gauss_kronrod<double, 15>::integrate(f1, 0, h, 7, 1e-8, &error);
    increment += arclength - h;
    // adjust parameter value for the next control point
    m_x[idx+1] += increment;
  }
  /* re-compute spline coefficients based on new parameterization of the control points
 *   note: a naive linear scaling of h would be faster, but gives a poor approximation
 *   if there is much curvature whatsoever on the segments between control points
 */
  set_coefficients();
}

template <int DIMENSION>
std::array<double, DIMENSION> spline<DIMENSION>::operator() (double x) const
{
    size_t n=m_x.size();
    // find the closest point m_x[idx] < x, idx=0 even if x<m_x[0]
    std::vector<double>::const_iterator it;
    it=std::lower_bound(m_x.begin(),m_x.end(),x);
    int idx=std::max( int(it-m_x.begin())-1, 0);

    double h=x-m_x[idx];
    std::array<double, DIMENSION> interpol;
    for(int d=0; d<DIMENSION; d++){
      if(x<m_x[0]) {
          // extrapolation to the left
          interpol[d]=(m_b0[d]*h + m_c0[d])*h + m_y[d][0];
      } else if(x>m_x[n-1]) {
          // extrapolation to the right
          interpol[d]=(m_b[n-1][d]*h + m_c[n-1][d])*h + m_y[n-1][d];
      } else {
          // interpolation
          interpol[d]=((m_a[idx][d]*h + m_b[idx][d])*h + m_c[idx][d])*h + m_y[idx][d];
      }
    }
    return interpol;
}

template <int DIMENSION>
double spline<DIMENSION>::max_path_curvature (double s1, double s2)
{
  assert(s1 >= m_x[0]);
  assert(s2 >= s1);
  assert(m_x[m_x.size()-1] >= s2);
    size_t n=m_x.size();
    // find the closest point m_x[idx] < x
    std::vector<double>::const_iterator it;
    it=std::lower_bound(m_x.begin(),m_x.end(),s1);
    int idx=std::max( int(it-m_x.begin())-1, 0);

    std::vector<double> kappa_candidates;

    while(m_x[idx]<=s2 && idx<n-1){
      double h_low = std::max(s1-m_x[idx], 0.0);
      double h_high = std::min(s2-m_x[idx], m_x[idx+1]-m_x[idx]);
      double a_b=0;
      double a_a=0;
      double b_b=0;
      for(int d=0; d<DIMENSION; d++){
        a_b += m_a[idx][d]*m_b[idx][d];
        a_a += m_a[idx][d]*m_a[idx][d];
        b_b += m_b[idx][d]*m_b[idx][d];
      }
      auto kappa = [&a_b, &a_a, &b_b](double s) 
      {
        return sqrt(36*a_a*s*s + 24*a_b*s + 4*b_b);
      };

      double h_star = -a_b / (3*a_a);
      kappa_candidates.push_back(kappa(h_low));
      if(h_star > h_low && h_star < h_high){
        kappa_candidates.push_back(kappa(h_star));
      }
      kappa_candidates.push_back(kappa(h_high));
      idx++;
    }
    auto kappa_p = std::max_element(std::begin(kappa_candidates), std::end(kappa_candidates));
    return *kappa_p;
}

} // namespace tk


} // namespace

#endif /* TK_SPLINE_H */
