#ifndef TYPE_DEFINITIONS_HPP_
#define TYPE_DEFINITIONS_HPP_

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using Eigen::Matrix;
using std::cout;
using std::endl;

typedef Matrix<double, 4, 1> mat41_t;
typedef Matrix<double, 4, 4> mat44_t;
typedef Matrix<double, 4, 2> mat42_t;
typedef Matrix<double, 2, 1> mat21_t;


#endif