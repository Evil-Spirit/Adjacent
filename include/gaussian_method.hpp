#ifndef ADJACENT_GAUSSIAN_METHOD_HPP
#define ADJACENT_GAUSSIAN_METHOD_HPP

#include <xtensor/xtensor.hpp>

class GaussianMethod
{
public:
    static constexpr double epsilon = 1e-10;
    static constexpr double rank_epsilon = 1e-8;

    // copy A so it doesn't get overwritten
    // note: could use xt::linalg::rank
    static int rank(xt::xtensor<double, 2> A);

    // copy A & B so they don't get overwritten
    static void solve(xt::xtensor<double, 2> A,
    				  xt::xtensor<double, 1> B,
                      xt::xtensor<double, 1>& X);
};

#endif