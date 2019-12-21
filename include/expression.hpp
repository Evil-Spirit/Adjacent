#ifndef ADJACENT_EXPRESSION_HPP
#define ADJACENT_EXPRESSION_HPP

#include <memory>
#include <string>

class Expr;

template <class T>
class Param
{
private:
	T m_value;

public:
	std::string m_name;
	bool m_reduceable = true;
	bool m_changed = false;
	std::shared_ptr<Expr> m_expr;

	Param() = default;
	Param(const std::string& name, bool reduceable = true);
	Param(const std::string& name, double value);

	void set_value(const T& other);
	T value() const;

	std::shared_ptr<Expr> expr();

	bool operator==(const Param& other) const;
};

template <class T>
void Param<T>::set_value(const T& other)
{
	if(other == m_value) return;
	m_changed = true;
	m_value = other;
}

template <class T>
T Param<T>::value() const {
	return m_value;
}

template <class T>
std::shared_ptr<Expr> Param<T>::expr()
{
	return m_expr;
}

template <class T>
Param<T>::Param(const std::string& name, bool reduceable /* = true */)
	: m_name(name), m_reduceable(reduceable)
{
	m_expr = std::make_shared<Expr>(this);
}

template <class T>
Param<T>::Param(const std::string& name, double value)
	: m_name(name), m_value(value)
{
	m_expr = std::make_shared<Expr>(this);
}

template <class T>
bool Param<T>::operator==(const Param& other) const
{
	return m_name == other.m_name;
}

namespace std {

  template <>
  struct hash<Param<double>>
  {
    std::size_t operator()(const Param<double>& k) const
    {
      // Compute individual hash values for first,
      // second and third and combine them using XOR
      // and bit shifting:
        return std::hash<std::string>()(k.m_name);
      // return ((hash<string>()(k.first)
      //          ^ (hash<string>()(k.second) << 1)) >> 1)
      //          ^ (hash<int>()(k.third) << 1);
    }
  };

}


enum Op {
	Undefined,
	Const,
	ParamOp,
	Add,
	Sub,
	Mul,
	Div,
	Sin,
	Cos,
	ACos,
	ASin,
	Sqrt,
	Sqr,
	Atan2,
	Abs,
	Sign,
	Neg,
	Pos,
	Drag,
	Exp,
	Sinh,
	Cosh,
	SFres,
	CFres,
	//Pow,
};


class Expr
	: public std::enable_shared_from_this<Expr>
{
public:
	Op op;

	std::shared_ptr<Expr> a;
	std::shared_ptr<Expr> b;
	Param<double>* param;
	double value;
	Expr() = default;

	// Todo automatic conversion from double?!
	Expr(double value);

	Expr(Param<double>* p);

	Expr(const Op& op, const std::shared_ptr<Expr>& a, const std::shared_ptr<Expr>& b);

	std::shared_ptr<Expr> drag(const std::shared_ptr<Expr>& to)
	{
		return std::make_shared<Expr>(Op::Drag, std::shared_ptr<Expr>(this), to);
	}

	bool is_zero_const() const;
	bool is_one_const() const;
	bool is_minus_one_const() const;
	bool is_const() const;
	bool is_drag() const;

	bool is_unary();

	bool is_additive();

	double eval();

	std::string quoted();
	std::string quoted_add();
	std::string to_string();
	bool is_dependend_on(const Param<double>& p);
	std::shared_ptr<Expr> derivative(const Param<double>& p);
	std::shared_ptr<Expr> d(const Param<double>& p);

	bool is_substitution_form() const;

	Param<double>* get_substitution_param_a() const;
	Param<double>* get_substitution_param_b() const;

	void substitute(Param<double>& pa, Param<double>& pb);
	void substitute(Param<double>& p, std::shared_ptr<Expr> e);

	bool has_two_operands() const;
	Op get_op() const;
};

static std::shared_ptr<Expr> zero = std::make_shared<Expr>(0.), one = std::make_shared<Expr>(1.), 
							 mOne = std::make_shared<Expr>(-1.), two = std::make_shared<Expr>(2.0);

std::shared_ptr<Expr> expr(double);

// note missing unary +?
std::shared_ptr<Expr> operator-(const std::shared_ptr<Expr>& a);
std::shared_ptr<Expr> operator-(const std::shared_ptr<Expr>& a, const std::shared_ptr<Expr>& b);
std::shared_ptr<Expr> operator+(const std::shared_ptr<Expr>& a, const std::shared_ptr<Expr>& b);
std::shared_ptr<Expr> operator*(const std::shared_ptr<Expr>& a, const std::shared_ptr<Expr>& b);
std::shared_ptr<Expr> operator/(const std::shared_ptr<Expr>& a, const std::shared_ptr<Expr>& b);

std::shared_ptr<Expr> sin (const std::shared_ptr<Expr>& x);
std::shared_ptr<Expr> cos (const std::shared_ptr<Expr>& x);
std::shared_ptr<Expr> acos (const std::shared_ptr<Expr>& x);
std::shared_ptr<Expr> asin (const std::shared_ptr<Expr>& x);
std::shared_ptr<Expr> sqrt (const std::shared_ptr<Expr>& x);
std::shared_ptr<Expr> sqr (const std::shared_ptr<Expr>& x);
std::shared_ptr<Expr> abs (const std::shared_ptr<Expr>& x);
std::shared_ptr<Expr> sign (const std::shared_ptr<Expr>& x);
std::shared_ptr<Expr> atan2 (const std::shared_ptr<Expr>& x, const std::shared_ptr<Expr>& y);
std::shared_ptr<Expr> expo (const std::shared_ptr<Expr>& x);
std::shared_ptr<Expr> sinh (const std::shared_ptr<Expr>& x);
std::shared_ptr<Expr> cosh (const std::shared_ptr<Expr>& x);
std::shared_ptr<Expr> sfres (const std::shared_ptr<Expr>& x);
std::shared_ptr<Expr> cfres (const std::shared_ptr<Expr>& x);

#endif