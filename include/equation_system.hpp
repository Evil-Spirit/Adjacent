#ifndef ADJACENT_EQUATION_SYSTEM_HPP
#define ADJACENT_EQUATION_SYSTEM_HPP

#include <vector>
#include <unordered_map>

#include <xtensor/xtensor.hpp>
#include "expression.hpp"
#include "expression_vector.hpp"
#include "gaussian_method.hpp"

enum SolveResult
{
    OKAY,
    DIDNT_CONVERGE,
    REDUNDANT,
    POSTPONE
};

using expr_ptr = std::shared_ptr<Expr>;

class EquationSystem
{
public:
    bool is_dirty = true;
    int max_steps = 20;
    int drag_steps = 3;
    bool revert_when_not_converged = true;

    std::string stats;
    bool dof_changed;

    xt::xtensor<std::shared_ptr<Expr>, 2> J;
    xt::xtensor<double, 2> A;
    xt::xtensor<double, 2> AAT;
    xt::xtensor<double, 1> B;
    xt::xtensor<double, 1> X;
    xt::xtensor<double, 1> Z;
    xt::xtensor<double, 1> old_param_value;

    std::vector<std::shared_ptr<Expr>> source_equations;
    std::vector<std::shared_ptr<Param<double>>> parameters;

    std::vector<std::shared_ptr<Expr>> equations;
    std::vector<std::shared_ptr<Param<double>>> current_params;

    std::unordered_map<std::shared_ptr<Param<double>>, std::shared_ptr<Param<double>>> subs;

    void add_equation(const std::shared_ptr<Expr>& eq);
    void add_equation(const ExpVector& v);
    void add_equations(const std::vector<ExprPtr>& v);

    void remove_equation(const std::shared_ptr<Expr>& eq);

    void add_parameter(const ParamPtr& p);
    void add_parameters(const std::vector<ParamPtr>& p);

    void remove_parameter(const std::shared_ptr<Param<double>>& p);

    void eval(xt::xtensor<double, 1>& B, bool clear_drag);

    bool is_converged(bool check_drag, bool print_non_converged = false);
    void store_params();
    void revert_params();

    xt::xtensor<std::shared_ptr<Expr>, 2> write_jacobian(
        const std::vector<std::shared_ptr<Expr>>& equations,
        const std::vector<std::shared_ptr<Param<double>>>& parameters);

    bool has_dragged();
    void eval_jacobian(const xt::xtensor<expr_ptr, 2>& J, xt::xtensor<double, 2>& A,
                       bool clear_drag);
    void solve_least_squares(const xt::xtensor<double, 2>& A, const xt::xtensor<double, 1>& B,
                             xt::xtensor<double, 1>& X);
    void clear();

    bool test_rank(int& dof);

    void update_dirty();

    void back_substitution(
        std::unordered_map<std::shared_ptr<Param<double>>, std::shared_ptr<Param<double>>>& subs);
    std::unordered_map<std::shared_ptr<Param<double>>, std::shared_ptr<Param<double>>>
    solve_by_substitution();

    SolveResult solve();
};

#endif