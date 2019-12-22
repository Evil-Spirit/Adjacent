#include <string>

#include "expression.hpp"
#include "expression_vector.hpp"
#include "equation_system.hpp"

class ExpBasis
{
public:
    std::shared_ptr<Param<double>> px, py, pz, ux, uy, uz, vx, vy, vz, nx, ny, nz;
    ExpVector p, u, v, n;

    ExpBasis();
    ExpBasis(const double (&params)[12]);

    template <class L>
    void apply_each_param(L&& l);

    std::string to_string();

    inline static ExpBasis from_string(const std::string& str);
    ExpVector transform_position(const ExpVector& pos);
    ExpVector transform_direction(const ExpVector& dir);

    void generate_equations(EquationSystem& sys);

    bool changed();
    void mark_unchanged();
};

inline ExpBasis ExpBasis::from_string(const std::string& str)
{
    std::istringstream iss(str);
    double p[12];
    for (std::size_t i = 0; i < 12; ++i)
        iss >> p[i];
    return ExpBasis(p);
}


class ExpBasis2d
{
    std::shared_ptr<Param<double>> px, py, ux, uy, vx, vy;
    ExpVector u, v, p;

    ExpBasis2d();

    ExpBasis2d(const double (&params)[6]);

    void set_pose_param(const std::shared_ptr<Param<double>>& x,
                        const std::shared_ptr<Param<double>>& y);

    template <class L>
    void apply_each_param(L&& l);

    std::string to_string();
    inline static ExpBasis2d from_string(const std::string& str);

    ExpVector transform_position(const ExpVector& pos);
    ExpVector transform_direction(const ExpVector& dir);

    void generate_equations(EquationSystem& sys);

    bool changed();
    void mark_unchanged();
};

inline ExpBasis2d ExpBasis2d::from_string(const std::string& str)
{
    std::istringstream iss(str);
    double p[6];
    iss >> p[0] >> p[1] >> p[2] >> p[3] >> p[4] >> p[5];
    return ExpBasis2d(p);
}
