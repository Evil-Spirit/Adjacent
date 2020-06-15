#include <string>
#include <memory>
#include <cmath>

#include "expression.hpp"

std::shared_ptr<Expr> expr(double);

std::shared_ptr<Expr> operator-(const std::shared_ptr<Expr>& a)
{
    if (a->is_zero_const())
        return a;
    if (a->is_const())
        return std::make_shared<Expr>(-a->value);
    if (a->op == Op::Neg)
        return a->a;
    return std::make_shared<Expr>(Op::Neg, a, nullptr);
}

std::shared_ptr<Expr> operator-(const std::shared_ptr<Expr>& a, const std::shared_ptr<Expr>& b)
{
    if (a->is_zero_const())
        return -b;
    if (b->is_zero_const())
        return a;
    return std::make_shared<Expr>(Op::Sub, a, b);
}

std::shared_ptr<Expr> operator+(const std::shared_ptr<Expr>& a, const std::shared_ptr<Expr>& b)
{
    if (a->is_zero_const())
        return b;
    if (b->is_zero_const())
        return a;
    if (b->op == Op::Neg)
        return a - b->a;
    if (b->op == Op::Pos)
        return a + b->a;
    return std::make_shared<Expr>(Op::Add, a, b);
}

std::shared_ptr<Expr> operator*(const std::shared_ptr<Expr>& a, const std::shared_ptr<Expr>& b)
{
    if (a->is_zero_const())
        return zero;
    if (b->is_zero_const())
        return zero;
    if (a->is_one_const())
        return b;
    if (b->is_one_const())
        return a;
    if (a->is_minus_one_const())
        return -b;
    if (b->is_minus_one_const())
        return -a;
    if (a->is_const() && b->is_const())
        return std::make_shared<Expr>(a->value * b->value);
    return std::make_shared<Expr>(Op::Mul, a, b);
}

std::shared_ptr<Expr> operator/(const std::shared_ptr<Expr>& a, const std::shared_ptr<Expr>& b)
{
    if (b->is_one_const())
        return a;
    if (a->is_zero_const())
        return zero;
    if (b->is_minus_one_const())
        return -a;
    return std::make_shared<Expr>(Op::Div, a, b);
}

std::shared_ptr<Expr> sin(const std::shared_ptr<Expr>& x)
{
    return std::make_shared<Expr>(Op::Sin, x);
}
std::shared_ptr<Expr> cos(const std::shared_ptr<Expr>& x)
{
    return std::make_shared<Expr>(Op::Cos, x);
}
std::shared_ptr<Expr> acos(const std::shared_ptr<Expr>& x)
{
    return std::make_shared<Expr>(Op::ACos, x);
}
std::shared_ptr<Expr> asin(const std::shared_ptr<Expr>& x)
{
    return std::make_shared<Expr>(Op::ASin, x);
}
std::shared_ptr<Expr> sqrt(const std::shared_ptr<Expr>& x)
{
    return std::make_shared<Expr>(Op::Sqrt, x);
}
std::shared_ptr<Expr> sqr(const std::shared_ptr<Expr>& x)
{
    return std::make_shared<Expr>(Op::Sqr, x);
}
std::shared_ptr<Expr> abs(const std::shared_ptr<Expr>& x)
{
    return std::make_shared<Expr>(Op::Abs, x);
}
std::shared_ptr<Expr> sign(const std::shared_ptr<Expr>& x)
{
    return std::make_shared<Expr>(Op::Sign, x);
}
std::shared_ptr<Expr> atan2(const std::shared_ptr<Expr>& x, const std::shared_ptr<Expr>& y)
{
    return std::make_shared<Expr>(Op::Atan2, x, y);
}
std::shared_ptr<Expr> expo(const std::shared_ptr<Expr>& x)
{
    return std::make_shared<Expr>(Op::Exp, x);
}
std::shared_ptr<Expr> sinh(const std::shared_ptr<Expr>& x)
{
    return std::make_shared<Expr>(Op::Sinh, x);
}
std::shared_ptr<Expr> cosh(const std::shared_ptr<Expr>& x)
{
    return std::make_shared<Expr>(Op::Cosh, x);
}
std::shared_ptr<Expr> sfres(const std::shared_ptr<Expr>& x)
{
    return std::make_shared<Expr>(Op::SFres, x);
}
std::shared_ptr<Expr> cfres(const std::shared_ptr<Expr>& x)
{
    return std::make_shared<Expr>(Op::CFres, x);
}


// Todo automatic conversion from double?!
Expr::Expr(double value)
    : value(value)
    , op(Op::Const)
{
}

Expr::Expr(std::shared_ptr<Param<double>> p)
    : param(p)
    , op(Op::ParamOp)
{
}

Expr::Expr(const Op& op, const std::shared_ptr<Expr>& a, const std::shared_ptr<Expr>& b)
    : op(op)
    , a(a)
    , b(b)
{
}

// Todo figure out enable_shared_from_this
// std::shared_ptr<Expr> drag(const std::shared_ptr<Expr>& to)
// {
//  return std::make_shared<Expr>(Op::Drag, this, to);
// }

bool Expr::is_zero_const() const
{
    return op == Op::Const && value == 0.0;
}

bool Expr::is_one_const() const
{
    return op == Op::Const && value == 1.0;
}

bool Expr::is_minus_one_const() const
{
    return op == Op::Const && value == -1.0;
}

bool Expr::is_const() const
{
    return op == Op::Const;
}

bool Expr::is_drag() const
{
    return op == Op::Drag;
}

bool Expr::is_unary()
{
    switch (op)
    {
        case Op::Const:
        case Op::ParamOp:
        case Op::Sin:
        case Op::Cos:
        case Op::ACos:
        case Op::ASin:
        case Op::Sqrt:
        case Op::Sqr:
        case Op::Abs:
        case Op::Sign:
        case Op::Neg:
        case Op::Pos:
        case Op::Exp:
        case Op::Cosh:
        case Op::Sinh:
        case Op::CFres:
        case Op::SFres:
            return true;
    }
    return false;
}

bool Expr::is_additive()
{
    switch (op)
    {
        case Op::Drag:
        case Op::Sub:
        case Op::Add:
            return true;
    }
    return false;
}

double Expr::eval()
{
    switch (op)
    {
        case Op::Const:
            return value;
        case Op::ParamOp:
            return param->value();
        case Op::Add:
            return a->eval() + b->eval();
        case Op::Drag:
        case Op::Sub:
            return a->eval() - b->eval();
        case Op::Mul:
            return a->eval() * b->eval();
        case Op::Div:
        {
            double bv = b->eval();
            if (std::abs(bv) < 1e-10)
            {
                // Debug.Log("Division by zero");
                bv = 1.0;
            }
            return a->eval() / bv;
        }
        case Op::Sin:
            return std::sin(a->eval());
        case Op::Cos:
            return std::cos(a->eval());
        case Op::ACos:
            return std::acos(a->eval());
        case Op::ASin:
            return std::asin(a->eval());
        case Op::Sqrt:
            return std::sqrt(a->eval());
        case Op::Sqr:
        {
            double av = a->eval();
            return av * av;
        }
        case Op::Atan2:
            return std::atan2(a->eval(), b->eval());
        case Op::Abs:
            return std::abs(a->eval());
        case Op::Sign:
            return sign(a->eval());
        case Op::Neg:
            return -a->eval();
        case Op::Pos:
            return a->eval();
        case Op::Exp:
            return std::exp(a->eval());
        case Op::Sinh:
            return std::sinh(a->eval());
        case Op::Cosh:
            return std::cosh(a->eval());
        case Op::SFres:
            return s_fres(a->eval());
        case Op::CFres:
            return c_fres(a->eval());
            // case Op::Pow: return std::Pow(a.Eval(), b.Eval());
    }
    return 0.0;
}

std::string Expr::quoted()
{
    if (is_unary())
        return to_string();
    return "(" + to_string() + ")";
}

std::string Expr::quoted_add()
{
    if (!is_additive())
        return to_string();
    return "(" + to_string() + ")";
}

std::string Expr::to_string()
{
    switch (op)
    {
        case Op::Const:
            return std::to_string(value);
        case Op::ParamOp:
            return param->m_name;
        case Op::Add:
            return a->to_string() + " + " + b->to_string();
        case Op::Sub:
            return a->to_string() + " - " + b->quoted_add();
        case Op::Mul:
            return a->quoted_add() + " * " + b->quoted_add();
        case Op::Div:
            return a->quoted_add() + " / " + b->quoted();
        case Op::Sin:
            return "sin(" + a->to_string() + ")";
        case Op::Cos:
            return "cos(" + a->to_string() + ")";
        case Op::ASin:
            return "asin(" + a->to_string() + ")";
        case Op::ACos:
            return "acos(" + a->to_string() + ")";
        case Op::Sqrt:
            return "sqrt(" + a->to_string() + ")";
        case Op::Sqr:
            return a->quoted() + " ^ 2";
        case Op::Abs:
            return "abs(" + a->to_string() + ")";
        case Op::Sign:
            return "sign(" + a->to_string() + ")";
        case Op::Atan2:
            return "atan2(" + a->to_string() + ", " + b->to_string() + ")";
        case Op::Neg:
            return "-" + a->quoted();
        case Op::Pos:
            return "+" + a->quoted();
        case Op::Drag:
            return a->to_string() + " ≈ " + b->quoted_add();
        case Op::Exp:
            return "exp(" + a->to_string() + ")";
        case Op::Sinh:
            return "sinh(" + a->to_string() + ")";
        case Op::Cosh:
            return "cosh(" + a->to_string() + ")";
        case Op::SFres:
            return "sfres(" + a->to_string() + ")";
        case Op::CFres:
            return "cfres(" + a->to_string() + ")";
            // case Op.Pow: return Quoted(a) + " ^ " + Quoted(b);
    }
    return "";
}

bool Expr::is_dependend_on(const std::shared_ptr<Param<double>>& p)
{
    if (op == Op::ParamOp)
        return param == p;
    if (a != nullptr)
    {
        if (b != nullptr)
        {
            return a->is_dependend_on(p) || b->is_dependend_on(p);
        }
        return a->is_dependend_on(p);
    }
    return false;
}

std::shared_ptr<Expr> Expr::derivative(const std::shared_ptr<Param<double>>& p)
{
    return d(p);
}

std::shared_ptr<Expr> Expr::d(const std::shared_ptr<Param<double>>& p)
{
    switch (op)
    {
        case Op::Const:
            return zero;
        case Op::ParamOp:
            return (param == p) ? one : zero;
        case Op::Add:
            return a->d(p) + b->d(p);
        case Op::Drag:
        case Op::Sub:
            return a->d(p) - b->d(p);
        case Op::Mul:
            return a->d(p) * b + a * b->d(p);
        case Op::Div:
            return (a->d(p) * b - a * b->d(p)) / sqr(b);
        case Op::Sin:
            return a->d(p) * cos(a);
        case Op::Cos:
            return a->d(p) * -sin(a);
        case Op::ASin:
            return a->d(p) / sqrt(one - sqr(a));
        case Op::ACos:
            return a->d(p) * mOne / sqrt(one - sqr(a));
        case Op::Sqrt:
            return a->d(p) / (two * sqrt(a));
        case Op::Sqr:
            return a->d(p) * two * a;
        case Op::Abs:
            return a->d(p) * sign(a);
        case Op::Sign:
            return zero;
        case Op::Neg:
            return -a->d(p);
        case Op::Atan2:
            return (b * a->d(p) - a * b->d(p)) / (sqr(a) + sqr(b));
        case Op::Exp:
            return a->d(p) * expo(a);
        case Op::Sinh:
            return a->d(p) * cosh(a);
        case Op::Cosh:
            return a->d(p) * sinh(a);
        case Op::SFres:
            return a->d(p) * sin(expr(M_PI) * sqr(a) / two);
        case Op::CFres:
            return a->d(p) * cos(expr(M_PI) * sqr(a) / two);
    }
    return zero;
}

std::shared_ptr<Expr> expr(double value)
{
    return std::make_shared<Expr>(value);
}

bool Expr::is_substitution_form() const
{
    return op == Op::Sub && a->op == Op::ParamOp && b->op == Op::ParamOp;
}

std::shared_ptr<Param<double>> Expr::get_substitution_param_a() const
{
    if (!is_substitution_form())
        return nullptr;
    return a->param;
}

std::shared_ptr<Param<double>> Expr::get_substitution_param_b() const
{
    if (!is_substitution_form())
        return nullptr;
    return b->param;
}

void Expr::substitute(std::shared_ptr<Param<double>>& pa, std::shared_ptr<Param<double>>& pb)
{
    if (a != nullptr)
    {
        a->substitute(pa, pb);
        if (b != nullptr)
        {
            b->substitute(pa, pb);
        }
    }
    else
    {
        if (op == Op::ParamOp && param == pa)
        {
            // TODO make memory management better
            param = pb;
        }
    }
}

void Expr::substitute(std::shared_ptr<Param<double>>& p, std::shared_ptr<Expr> e)
{
    if (a != nullptr)
    {
        a->substitute(p, e);
        if (b != nullptr)
        {
            b->substitute(p, e);
        }
    }
    else
    {
        if (op == Op::ParamOp && param == p)
        {
            op = e->op;
            a = e->a;
            b = e->b;
            param = e->param;
            value = e->value;
        }
    }
}

bool Expr::has_two_operands() const
{
    return a != nullptr && b != nullptr;
}

Op Expr::get_op() const
{
    return op;
}


// public void Walk(Action<Exp> action) {
//  action(this);
//  if(a != null) {
//      action(a);
//      if(b != null) {
//          action(b);
//      }
//  }
// }

// public Exp DeepClone() {
//  Exp result = new Exp();
//  result.op = op;
//  result.param = param;
//  result.value = value;
//  if(a != null) {
//      result.a = a.DeepClone();
//      if(b != null) {
//          result.b = b.DeepClone();
//      }
//  }
//  return result;
// }

// public void ReduceParams(List<Param> pars) {
//  if(op == Op.Param) {
//      if(param.reduceable && !pars.Contains(param)) {
//          value = Eval();
//          op = Op.Const;
//          param = null;
//      }
//      return;
//  }

//  if(a != null) {
//      a.ReduceParams(pars);
//      if(b != null) b.ReduceParams(pars);
//      if(a.IsConst() && (b == null || b.IsConst())) {
//          value = Eval();
//          op = Op.Const;
//          a = null;
//          b = null;
//          param = null;
//      }
//  }
// }