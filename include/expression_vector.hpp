#ifndef ADJACENT_EXPRESSION_VECTOR_HPP
#define ADJACENT_EXPRESSION_VECTOR_HPP

#include "expression.hpp"

class ExpVector;

ExpVector operator+(const ExpVector& a, const ExpVector& b);
ExpVector operator-(const ExpVector& a, const ExpVector& b);
ExpVector operator*(const ExpVector& a, const ExpVector& b);
ExpVector operator/(const ExpVector& a, const ExpVector& b);
ExpVector operator-(const ExpVector& b);
ExpVector operator*(const std::shared_ptr<Expr>& a, const ExpVector& b);
ExpVector operator*(const ExpVector& a, const std::shared_ptr<Expr>& b);
ExpVector operator/(const std::shared_ptr<Expr>& a, const ExpVector& b);
ExpVector operator/(const ExpVector& a, const std::shared_ptr<Expr>& b);
std::shared_ptr<Expr> dot(const ExpVector& a, const ExpVector& b);
ExpVector cross(const ExpVector& a, const ExpVector& b);


class ExpVector
{
public:
	std::shared_ptr<Expr> x, y, z;

	ExpVector(const std::shared_ptr<Expr>& x,
			  const std::shared_ptr<Expr>& y,
			  const std::shared_ptr<Expr>& z);

	// constructor from eigen etc.
	// public static implicit operator ExpVector(Vector3 v) {
	// 	return  ExpVector(v.x, v.y, v.z);
	// }

	std::shared_ptr<Expr> magnitude() const;
	ExpVector normalized() const;

	// public Vector3 eval() {
	// 	return new Vector3((float)x->eval(), (float)y->eval(), (float)z->eval());
	// }

	bool values_equals(const ExpVector& o, double eps) const;
};

#endif