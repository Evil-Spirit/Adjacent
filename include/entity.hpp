#include "expression.hpp"
#include "expression_vector.hpp"

#include <xtensor/xtensor.hpp>

#ifndef ADJACENT_ENTITY_HPP
#define ADJACENT_ENTITY_HPP

using ExpVectorPtr = std::shared_ptr<ExpVector>;

class Entity {
public:
	// virtual std::shared_ptr<ExpVector> get_points() = 0;
	// virtual std::shared_ptr<ExpVector> get_segments() = 0;
	virtual	std::string to_string() = 0; 
	virtual ExpVectorPtr point_on(const ExprPtr& t) = 0;
	virtual ExpVectorPtr tangent_at(const ExprPtr& t) = 0;
	virtual ExprPtr length() = 0;
	virtual ExprPtr radius() = 0;
	// virtual ExpVectorPtr center() = 0;
};

class PointE : public Entity {
public:
	ParamPtr x, y, z;
	std::shared_ptr<ExpVector> exp_;

	PointE() = default;
	PointE(const ParamPtr& x, const ParamPtr& y, const ParamPtr& z) :
		x(x), y(y), z(z)
	{
	}

	// PointE& PointE(const PointE&) = default;

	std::string to_string()
	{
		return "Point(" + x->to_string() + ", " + y->to_string() + ", " + z->to_string() + ")";
	}

	ExpVector expr() {
		if (exp_ == nullptr) {
			exp_ = std::make_shared<ExpVector>(x->expr(), y->expr(), z->expr());
		}
		// TODO transform
		return *exp_;
	}

	bool is_changed() {
		return x->m_changed || y->m_changed || z->m_changed;
	}

	// parameters() {}

	// points()

	bool is_3d() {
		return false;
	}

	void on_drag(const xt::xtensor<double, 1>& delta) {
		x->set_value(x->value() + delta[0]);
		y->set_value(y->value() + delta[1]);
		if (is_3d()) {
			z->set_value(z->value() + delta[2]);
		}
	}
	
	std::shared_ptr<ExpVector> tangent_at(const ExprPtr&) {
		return nullptr;
	}

	ExpVectorPtr point_on(const ExprPtr&) { return std::make_shared<ExpVector>(expr()); }	
	ExprPtr length() { return nullptr; }	
	ExprPtr radius() { return nullptr; }	
};

class CircleE : public Entity {
public:

	PointE center;
	ParamPtr m_radius;

	CircleE() {

	}

	std::string to_string() {
		return "Circle(" + center.to_string() + ", " + m_radius->to_string() + ")";
	}

	ExpVectorPtr tangent_at(const ExprPtr& t) {
		auto angle = t * expr(2.0 * M_PI);
		return std::make_shared<ExpVector>(-sin(angle), cos(angle), zero);
	}

	ExprPtr radius() {
		return abs(m_radius->expr());
	}

	ExprPtr length() {
		return expr(2.0 * M_PI) * radius();
	}

	ExpVectorPtr point_on(const ExprPtr& t) {
		auto angle = t * expr(2.0 * M_PI);
		return std::make_shared<ExpVector>(center.expr() + ExpVector(cos(angle), sin(angle), zero) * radius());
	}

};

class LineE : public Entity {
public:
	PointE p0, p1;

	LineE(const PointE& p0, const PointE& p1) : p0(p0), p1(p1)
	{
	}

	bool is_changed() {
		return p0.is_changed() || p1.is_changed();
	}

	PointE& source() { return p0; }
	PointE& target() { return p1; }

	std::string to_string() {
		return std::string("Line(") + p0.to_string() + " -> " + p1.to_string() + ")";
	}

	ExpVectorPtr point_on(const ExprPtr& t) {
		return std::make_shared<ExpVector>(p0.expr() + (p1.expr() - p0.expr()) * t);
	}

	ExpVectorPtr tangent_at(const ExprPtr& t) {
		return std::make_shared<ExpVector>(p1.expr() - p0.expr());
	}

	ExprPtr length() {
		return (p1.expr() - p0.expr()).magnitude();
	}

	ExprPtr radius() {
		return nullptr;
	}
};

#endif