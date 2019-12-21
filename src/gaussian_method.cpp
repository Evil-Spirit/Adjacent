#include <xtensor/xtensor.hpp>

#include "gaussian_method.hpp"

// copy A so it doesn't get overwritten
// note: could use xt::linalg::rank
int GaussianMethod::rank(xt::xtensor<double, 2> A) {
	std::size_t rows = A.shape(0);
	std::size_t cols = A.shape(1);

	int rank = 0;
	xt::xtensor<double, 1> rowsLength = xt::empty<double>({rows});

	for(std::size_t i = 0; i < rows; i++)
	{
		for(std::size_t ii = 0; ii < i; ii++)
		{
			if(rowsLength(ii) <= rankEpsilon) continue;

			double sum = 0;
			for(std::size_t j = 0; j < cols; j++)
			{
				sum += A(ii, j) * A(i, j);
			}
			
			for(std::size_t j = 0; j < cols; j++)
			{
				A(i, j) -= A(ii, j) * sum / rowsLength(ii);
			}
		}

		double len = 0;
		for(int j = 0; j < cols; j++)
		{
			len += A(i, j) * A(i, j);
		}
		if(len > rankEpsilon)
		{
			rank++;
		}
		rowsLength(i) = len;
	}

	return rank;
}

// copy A & B so they don't get overwritten
void GaussianMethod::solve(xt::xtensor<double, 2> A, xt::xtensor<double, 1> B, xt::xtensor<double, 1>& X) {

	std::size_t rows = A.shape(0);
	std::size_t cols = A.shape(1);
	double t = 0.0;

	for(std::size_t r = 0; r < rows; r++) {

		std::size_t mr = r;
		double max = 0.0;
		for(std::size_t rr = r; rr < rows; rr++) {
			if(std::abs(A(rr, r)) <= max) continue;
			max = std::abs(A(rr, r));
			mr = rr;
		}

		if(max < epsilon) continue;

		for(std::size_t c = 0; c < cols; c++) {
			t = A(r, c);
			A(r, c) = A(mr, c);
			A(mr, c) = t;
		}

		t = B(r);
		B(r) = B(mr);
		B(mr) = t;

		// normalize
		/*
		double scale = A[r, r];
		for(std::size_t c = 0; c < cols; c++) {
			A[r, c] /= scale;
		}
		B[r] /= scale;
		*/

		// 
		for(std::size_t rr = r + 1; rr < rows; rr++) {
			double coef = A(rr, r) / A(r, r);
			for(std::size_t c = 0; c < cols; c++) {
				A(rr, c) -= A(r, c) * coef;
			}
			B(rr) -= B(r) * coef;
		}
	}

	for(std::size_t r = rows - 1; r >= 0; r--) {
		if(std::abs(A(r, r)) < epsilon) continue;
		double xx = B(r) / A(r, r);
		for(std::size_t rr = rows - 1; rr > r; rr--) {
			xx -= X(rr) * A(r, rr) / A(r, r);
		}
		X(r) = xx;
	}
}
