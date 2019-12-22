#include <string>

#include "expression.hpp"
#include "expression_vector.hpp"
#include "equation_system.hpp"
#include "expr_basis.hpp"

// clang-format off
ExpBasis::ExpBasis() :
    ExpBasis({0., 0., 0., // pxyz
              1., 0., 0., // uxyz
              0., 1., 0., // vxyz
              0., 0., 1.  // nxyz
    })
{
}
// clang-format on

ExpBasis::ExpBasis(const double (&params)[12])
    : px(param("px", params[0]))
    , py(param("py", params[1]))
    , pz(param("pz", params[2]))
    , ux(param("ux", params[3]))
    , uy(param("uy", params[4]))
    , uz(param("uz", params[5]))
    , vx(param("vx", params[6]))
    , vy(param("vy", params[7]))
    , vz(param("vz", params[8]))
    , nx(param("nx", params[9]))
    , ny(param("ny", params[10]))
    , nz(param("nz", params[11]))
    , p(px->expr(), py->expr(), pz->expr())
    , u(ux->expr(), uy->expr(), uz->expr())
    , v(vx->expr(), vy->expr(), vz->expr())
    , n(nx->expr(), ny->expr(), nz->expr())
{
}

// public Matrix4x4 matrix {
//  get {
//      return UnityExt.Basis(u.Eval(), v.Eval(), n.Eval(), p.Eval());
//  }
// }

template <class L>
void ExpBasis::apply_each_param(L&& l)
{
    l(ux);
    l(uy);
    l(uz);
    l(vx);
    l(vy);
    l(vz);
    l(nx);
    l(ny);
    l(nz);
    l(px);
    l(py);
    l(pz);
}

std::string ExpBasis::to_string()
{
    std::string result = "";
    apply_each_param([&result](auto& p) { result += std::to_string(p->value()) + " "; });
    return result;
}

ExpVector ExpBasis::transform_position(const ExpVector& pos)
{
    return pos.x * u + pos.y * v + pos.z * n + p;
}

ExpVector ExpBasis::transform_direction(const ExpVector& dir)
{
    return dir.x * u + dir.y * v + dir.z * n;
}

void ExpBasis::generate_equations(EquationSystem& sys)
{
    apply_each_param([&sys](auto& p) { sys.add_parameter(p); });

    sys.add_equation(u.magnitude() - one);
    sys.add_equation(v.magnitude() - one);

    auto c = cross(u, v);
    auto d = dot(u, v);
    sys.add_equation(atan2(c.magnitude(), d) - expr(M_PI) / two);
    sys.add_equation(n - cross(u, v));
}

bool ExpBasis::changed()
{
    bool changed = false;
    apply_each_param([&changed](auto& p) {
        if (!changed && p->m_changed)
        {
            changed = true;
        }
    });
    return changed;
}

void ExpBasis::mark_unchanged()
{
    apply_each_param([](auto& p) { p->m_changed = false; });
}

/**
 * ExpBasis2d
 */

ExpBasis2d::ExpBasis2d()
    : ExpBasis2d({ 0., 0., 1., 0., 0., 1. })
{
}

ExpBasis2d::ExpBasis2d(const double (&params)[6])
    : px(param("px", params[0]))
    , py(param("py", params[1]))
    , ux(param("ux", params[2]))
    , uy(param("uy", params[3]))
    , vx(param("vx", params[4]))
    , vy(param("vy", params[5]))
    , p(px->expr(), py->expr(), zero)
    , u(ux->expr(), uy->expr(), zero)
    , v(vx->expr(), vy->expr(), zero)
{
}

void ExpBasis2d::set_pose_param(const std::shared_ptr<Param<double>>& x,
                                const std::shared_ptr<Param<double>>& y)
{
    px = x;
    py = y;
    p.x = px->expr();
    p.y = py->expr();
}

template <class L>
void ExpBasis2d::apply_each_param(L&& l)
{
    l(px);
    l(py);
    l(ux);
    l(uy);
    l(vx);
    l(vy);
}

//  /*
//  public Matrix4x4 matrix {
//      get {
//          return UnityExt.Basis(u.Eval(), v.Eval(), n.Eval(), p.Eval());
//      }
//  }*/

std::string ExpBasis2d::to_string()
{
    std::string result = "";
    apply_each_param([&result](auto& p) { result += std::to_string(p->value()) + " "; });
    return result;
}

ExpVector ExpBasis2d::transform_position(const ExpVector& pos)
{
    return pos.x * u + pos.y * v + p;
}

ExpVector ExpBasis2d::transform_direction(const ExpVector& dir)
{
    return dir.x * u + dir.y * v;
}

void ExpBasis2d::generate_equations(EquationSystem& sys)
{
    apply_each_param([&sys](auto& p) { sys.add_parameter(p); });
    sys.add_equation(u.magnitude() - one);
    sys.add_equation(v.magnitude() - one);

    auto c = cross(u, v);
    auto d = dot(u, v);
    sys.add_equation(atan2(c.magnitude(), d) - expr(M_PI) / two);
}

bool ExpBasis2d::changed()
{
    bool changed = false;
    apply_each_param([&changed](auto& p) {
        if (!changed && p->m_changed)
        {
            changed = true;
        }
    });
    return changed;
}

void ExpBasis2d::mark_unchanged()
{
    apply_each_param([](auto& p) { p->m_changed = false; });
}