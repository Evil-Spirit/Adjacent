#ifndef ADJACENT_GAUSSIAN_METHOD_HPP
#define ADJACENT_GAUSSIAN_METHOD_HPP

#include <xtensor/xtensor.hpp>

class GaussianMethod {

public:
	static constexpr double epsilon = 1e-10;
	static constexpr double rankEpsilon = 1e-8;

	// public static string Print<T>(this T[,] A) {
	// 	string result = "";
	// 	for(int r = 0; r < A.GetLength(0); r++) {
	// 		for(int c = 0; c < A.GetLength(1); c++) {
	// 			result += A[r, c].ToString() + " ";
	// 		}
	// 		result += "\n";
	// 	}
	// 	return result;
	// }

	// public static string Print<T>(this T[] A) {
	// 	string result = "";
	// 	for(int r = 0; r < A.GetLength(0); r++) {
	// 		result += A[r].ToString() + "\n";
	// 	}
	// 	return result;
	// }

	// copy A so it doesn't get overwritten
	// note: could use xt::linalg::rank
	static int rank(xt::xtensor<double, 2> A);

	// copy A & B so they don't get overwritten
	static void solve(xt::xtensor<double, 2> A, xt::xtensor<double, 1> B, xt::xtensor<double, 1>& X);
};

#endif