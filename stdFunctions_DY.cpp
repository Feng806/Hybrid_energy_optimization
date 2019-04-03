//
//  stdFunctions_DY.cpp
//  GTLC
//
//  Created by David Yang on 6/25/18.
//  Copyright © 2018 David Yang. All rights reserved.
//

#include <stdio.h>
#include "fuelOpt_DP.hpp"
// Function that adds together two vectors that are the same length and hold doubles
std::vector <double> addVectors(std::vector <double> a, std::vector <double> b){
    // std::plus adds together its two arguments:
    std::transform (a.begin(), a.end(), b.begin(), a.begin(), std::plus<double>());
    return a;
}
// Function to calculate the norm of a std::vector
double l2_norm(std::vector<double> const& u) {
    double accum = 0.;
    for (int i = 0; i < u.size(); ++i) {
        accum += u[i] * u[i];
    }
    return sqrt(accum);
}

// Function to return a linearly spaced vector (MATLAB style)
std::vector<double> linspace(double a, double b, std::size_t N)
{
    double h = (b - a) / static_cast<double>(N-1);
    std::vector<double> xs(N);
    std::vector<double>::iterator x;
    double val;
    for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h) {
        *x = val;
    }
    return xs;
}
