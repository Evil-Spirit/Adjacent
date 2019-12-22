#include "expression_vector.hpp"

ExpVector::ExpVector(const std::shared_ptr<Expr>& x, const std::shared_ptr<Expr>& y,
                     const std::shared_ptr<Expr>& z)
    : x(x)
    , y(y)
    , z(z)
{
}

// constructor from eigen etc.
ExpVector::ExpVector(const double (&v)[3])
    : x(expr(v[0]))
    , y(expr(v[1]))
    , z(expr(v[2]))
{
}

std::shared_ptr<Expr> ExpVector::magnitude() const
{
    return sqrt(sqr(x) + sqr(y) + sqr(z));
}

ExpVector ExpVector::normalized() const
{
    return *this / magnitude();
}

// xt::xtensor_fixed<double, xt::xshape<3>> eval() {
//     return new xt::xtensor_fixed<double, xt::xshape<3>>({(double)x->eval(), (double)y->eval(),
//     (double)z->eval()});
// }

bool ExpVector::values_equals(const ExpVector& o, double eps) const
{
    return std::abs(x->eval() - o.x->eval()) < eps && std::abs(y->eval() - o.y->eval()) < eps
           && std::abs(z->eval() - o.z->eval()) < eps;
}

ExpVector operator+(const ExpVector& a, const ExpVector& b)
{
    return ExpVector(a.x + b.x, a.y + b.y, a.z + b.z);
}
ExpVector operator-(const ExpVector& a, const ExpVector& b)
{
    return ExpVector(a.x - b.x, a.y - b.y, a.z - b.z);
}

ExpVector operator*(const ExpVector& a, const ExpVector& b)
{
    return ExpVector(a.x * b.x, a.y * b.y, a.z * b.z);
}
ExpVector operator/(const ExpVector& a, const ExpVector& b)
{
    return ExpVector(a.x / b.x, a.y / b.y, a.z / b.z);
}

ExpVector operator-(const ExpVector& b)
{
    return ExpVector(-b.x, -b.y, -b.z);
}

ExpVector operator*(const std::shared_ptr<Expr>& a, const ExpVector& b)
{
    return ExpVector(a * b.x, a * b.y, a * b.z);
}

ExpVector operator*(const ExpVector& a, const std::shared_ptr<Expr>& b)
{
    return ExpVector(a.x * b, a.y * b, a.z * b);
}

ExpVector operator/(const std::shared_ptr<Expr>& a, const ExpVector& b)
{
    return ExpVector(a / b.x, a / b.y, a / b.z);
}

ExpVector operator/(const ExpVector& a, const std::shared_ptr<Expr>& b)
{
    return ExpVector(a.x / b, a.y / b, a.z / b);
}

std::shared_ptr<Expr> dot(const ExpVector& a, const ExpVector& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

ExpVector cross(const ExpVector& a, const ExpVector& b)
{
    return ExpVector(a.y * b.z - b.y * a.z, a.z * b.x - b.z * a.x, a.x * b.y - b.x * a.y);
}

std::shared_ptr<Expr> PointLineDistance(const ExpVector& point, const ExpVector& l0,
                                        const ExpVector& l1)
{
    auto d = l0 - l1;
    return cross(d, l0 - point).magnitude() / d.magnitude();
}

// float PointLineDistance(Vector3 point, Vector3 l0, Vector3 l1)
// {
//  var d = l0 - l1;
//  return Vector3.cross(d, l0 - point).magnitude / d.magnitude;
// }

ExpVector project_point_to_line(const ExpVector& p, const ExpVector& l0, const ExpVector& l1)
{
    auto d = l1 - l0;
    auto t = dot(d, p - l0) / dot(d, d);
    return l0 + d * t;
}

// public static Vector3 project_point_to_line(Vector3 p, Vector3 l0, Vector3 l1) {
//  var d = l1 - l0;
//  var t = Vector3.dot(d, p - l0) / Vector3.dot(d, d);
//  return l0 + d * t;
// }

ExpVector rotate_around(const ExpVector& point, const ExpVector& axis, const ExpVector& origin,
                        const std::shared_ptr<Expr>& angle)
{
    auto a = axis.normalized();
    auto c = cos(angle);
    auto s = sin(angle);
    auto u = ExpVector(c + (one - c) * a.x * a.x,
                       (one - c) * a.y * a.x + s * a.z,
                       (one - c) * a.z * a.x - s * a.y);
    auto v = ExpVector((one - c) * a.x * a.y - s * a.z,
                       c + (one - c) * a.y * a.y,
                       (one - c) * a.z * a.y + s * a.x);
    auto n = ExpVector((one - c) * a.x * a.z + s * a.y,
                       (one - c) * a.y * a.z - s * a.x,
                       c + (one - c) * a.z * a.z);
    auto p = point - origin;
    return p.x * u + p.y * v + p.z * n + origin;
}

// Vector3 rotate_around(Vector3 point, Vector3 axis, Vector3 origin, float angle) {
//  auto a = axis.normalized;
//  auto c = Mathf.Cos(angle);
//  auto s = Mathf.Sin(angle);
//  auto u = Vector3(c + (1 - c) * a.x * a.x, (1 - c) * a.y * a.x + s * a.z, (1 - c) * a.z * a.x - s
//  * a.y); auto v = Vector3((1 - c) * a.x * a.y - s * a.z, c + (1 - c) * a.y * a.y, (1 - c) * a.z *
//  a.y + s * a.x); auto n = Vector3((1 - c) * a.x * a.z + s * a.y, (1 - c) * a.y * a.z - s * a.x, c
//  + (1 - c) * a.z * a.z); auto p = point - origin; return p.x * u + p.y * v + p.z * n + origin;
// }
