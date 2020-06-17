#include <vector>
#include <unordered_map>

#include <xtensor/xtensor.hpp>
#include <xtensor/xio.hpp>
#include "expression.hpp"
#include "expression_vector.hpp"
#include "gaussian_method.hpp"
#include "equation_system.hpp"

constexpr bool DEBUG = false;

void EquationSystem::add_equation(const std::shared_ptr<Expr>& eq)
{
    if (DEBUG)
        std::cout << "Adding equation: " << eq->to_string() << std::endl;
    source_equations.push_back(eq);
    is_dirty = true;
}

void EquationSystem::add_equation(const ExpVector& v)
{
    source_equations.push_back(v.x);
    source_equations.push_back(v.y);
    source_equations.push_back(v.z);
    is_dirty = true;
}

void EquationSystem::add_equations(const std::vector<ExprPtr>& eq)
{
    for (const auto& e : eq)
        add_equation(e);
}

void EquationSystem::remove_equation(const std::shared_ptr<Expr>& eq)
{
    auto it = std::find(source_equations.begin(), source_equations.end(), eq);
    if (it == source_equations.end())
    {
        throw std::runtime_error(
            "Could not remove equation, it doesn't exist in source_equations vector.");
    }
    source_equations.erase(it);
    is_dirty = true;
}

void EquationSystem::add_parameter(const std::shared_ptr<Param<double>>& p)
{
    if (DEBUG)
        std::cout << "Adding Parameter: " << p->to_string() << std::endl;
    if (std::find(parameters.begin(), parameters.end(), p) != parameters.end())
    {
        return;
    }
    parameters.push_back(p);
    is_dirty = true;
}

void EquationSystem::add_parameters(const std::vector<ParamPtr>& pv)
{
    for (const auto& p : pv)
        add_parameter(p);
}

void EquationSystem::remove_parameter(const std::shared_ptr<Param<double>>& p)
{
    if (DEBUG)
        std::cout << "Removing Parameter " << p->to_string() << std::endl;
    auto it = std::find(parameters.begin(), parameters.end(), p);
    if (it == parameters.end())
    {
        throw std::runtime_error(
            "Could not remove parameter, it doesn't exist in parameters vector.");
    }
}

void EquationSystem::eval(xt::xtensor<double, 1>& B, bool clear_drag)
{
    B.resize({ equations.size() });
    for (int i = 0; i < equations.size(); i++)
    {
        if (clear_drag && equations[i]->is_drag())
        {
            B(i) = 0.0;
            continue;
        }
        B(i) = equations[i]->eval();
    }
}

bool EquationSystem::is_converged(bool check_drag, bool print_non_converged /* = false*/)
{
    for (int i = 0; i < equations.size(); i++)
    {
        if (!check_drag && equations[i]->is_drag())
        {
            continue;
        }

        if (std::abs(B(i)) < GaussianMethod::epsilon)
            continue;

        if (print_non_converged)
        {
            std::cout << "Not converged: " + equations[i]->to_string() << "\n";
            continue;
            // continue; ???
        }
        return false;
    }
    return true;
}

void EquationSystem::store_params()
{
    for (std::size_t i = 0; i < parameters.size(); i++)
    {
        old_param_value[i] = parameters[i]->value();
    }
}

void EquationSystem::revert_params()
{
    for (std::size_t i = 0; i < parameters.size(); i++)
    {
        parameters[i]->set_value(old_param_value[i]);
    }
}

xt::xtensor<std::shared_ptr<Expr>, 2> EquationSystem::write_jacobian(
    const std::vector<std::shared_ptr<Expr>>& equations,
    const std::vector<std::shared_ptr<Param<double>>>& parameters)
{
    xt::xtensor<std::shared_ptr<Expr>, 2> J
        = xt::empty<std::shared_ptr<Expr>>({ equations.size(), parameters.size() });
    for (std::size_t r = 0; r < equations.size(); r++)
    {
        const auto& eq = equations[r];
        for (std::size_t c = 0; c < parameters.size(); c++)
        {
            const auto& u = parameters[c];
            J(r, c) = eq->derivative(u);

            if (DEBUG)
                std::cout << "Equation: " << eq->to_string() << "\n"
                          << "Derived by " << u->to_string() << "\n\n"
                          << J(r, c)->to_string() << "\n\n";

            /*
            if(!J[r, c].IsZeroConst()) {
                Debug.Log(J[r, c].ToString() + "\n");
            }
            */
        }
    }
    return J;
}

bool EquationSystem::has_dragged()
{
    return std::any_of(equations.begin(), equations.end(), [](auto& e) { return e->is_drag(); });
}

void EquationSystem::eval_jacobian(const xt::xtensor<expr_ptr, 2>& J, xt::xtensor<double, 2>& A,
                                   bool clear_drag)
{
    update_dirty();
    for (std::size_t r = 0; r < J.shape(0); r++)
    {
        if (clear_drag && equations[r]->is_drag())
        {
            for (std::size_t c = 0; c < J.shape(1); c++)
            {
                A(r, c) = 0.0;
            }
            continue;
        }
        for (std::size_t c = 0; c < J.shape(1); c++)
        {
            A(r, c) = J(r, c)->eval();
        }
    }
}

void EquationSystem::solve_least_squares(const xt::xtensor<double, 2>& A,
                                         const xt::xtensor<double, 1>& B, xt::xtensor<double, 1>& X)
{
    // A^T * A * X = A^T * B
    std::size_t rows = A.shape(0);
    std::size_t cols = A.shape(1);

    for (std::size_t r = 0; r < rows; r++)
    {
        // only up to `rows`?
        for (std::size_t c = 0; c < rows; c++)
        {
            double sum = 0.0;
            for (std::size_t i = 0; i < cols; i++)
            {
                if (A(c, i) == 0 || A(r, i) == 0)
                    continue;
                sum += A(r, i) * A(c, i);
            }
            AAT(r, c) = sum;
        }
    }

    GaussianMethod::solve(AAT, B, Z);

    for (int c = 0; c < cols; c++)
    {
        double sum = 0.0;
        for (int r = 0; r < rows; r++)
        {
            sum += Z(r) * A(r, c);
        }
        X(c) = sum;
    }
}

void EquationSystem::clear()
{
    parameters.clear();
    current_params.clear();
    equations.clear();
    source_equations.clear();
    is_dirty = true;
    update_dirty();
}

bool EquationSystem::test_rank(int& dof)
{
    eval_jacobian(J, A, false);
    int rank = GaussianMethod::rank(A);
    dof = A.shape(1) - rank;
    return rank == A.shape(0);
}

void EquationSystem::update_dirty()
{
    if (is_dirty)
    {
        // equations = source_equations.Select(e => e.DeepClone()).ToList();
        equations = source_equations;
        current_params = parameters;
        /*
        foreach(var e in equations) {
            e.ReduceParams(current_params);
        }*/
        // current_params = parameters.Where(p => equations.Any(e => e.IsDependOn(p))).ToList();
        subs = solve_by_substitution();

        J = write_jacobian(equations, current_params);
        A = xt::empty<double>(J.shape());
        B = xt::empty<double>({ equations.size() });
        X = xt::empty<double>({ current_params.size() });
        Z = xt::empty<double>({ A.shape(0) });
        AAT = xt::empty<double>({ A.shape(0), A.shape(0) });
        old_param_value = xt::empty<double>({ parameters.size() });
        is_dirty = false;
        dof_changed = true;
    }
}

void EquationSystem::back_substitution(
    std::unordered_map<std::shared_ptr<Param<double>>, std::shared_ptr<Param<double>>>& subs)
{
    if (subs.empty())
        return;
    for (std::size_t i = 0; i < parameters.size(); i++)
    {
        auto& p = parameters[i];
        if (subs.find(p) == subs.end())
            continue;
        p->set_value(subs[p]->value());
    }
}

std::unordered_map<std::shared_ptr<Param<double>>, std::shared_ptr<Param<double>>>
EquationSystem::solve_by_substitution()
{
    std::unordered_map<std::shared_ptr<Param<double>>, std::shared_ptr<Param<double>>> subs;
    if (DEBUG)
        std::cout << "Solving by substitution" << std::endl;
    for (std::size_t i = 0; i < equations.size(); i++)
    {
        auto& eq = equations[i];
        if (!eq->is_substitution_form())
        {
            continue;
        }
        auto a = eq->get_substitution_param_a();
        auto b = eq->get_substitution_param_b();
        if (std::abs(a->value() - b->value()) > GaussianMethod::epsilon)
            continue;
        // check if b in current params
        if (std::find(current_params.begin(), current_params.end(), b) != current_params.end())
        {
            // check if pointer swap is enough?!
            std::swap(a, b);
        }
        // TODO: Check errors
        // if(!parameters.Contains(b)) {
        //  continue;
        //}

        for (auto& kv : subs)
        {
            if (subs[kv.first] == b)
            {
                subs[kv.first] = a;
            }
        }
        subs[b] = a;

        equations.erase(equations.begin() + i);
        i--;
        current_params.erase(std::find(current_params.begin(), current_params.end(), b));
        for (std::size_t j = 0; j < equations.size(); j++)
        {
            equations[j]->substitute(b, a);
        }
    }
    return subs;
}

SolveResult EquationSystem::solve()
{
    dof_changed = false;
    update_dirty();
    store_params();
    int steps = 0;
    do
    {
        bool is_drag_step = steps <= drag_steps;
        eval(B, /*clear_drag*/ !is_drag_step);
        /*
        if(steps > 0) {
            BackSubstitution(subs);
            return SolveResult.POSTPONE;
        }
        */

        if (is_converged(is_drag_step))
        {
            if (steps > 0)
            {
                dof_changed = true;
                std::cout << "Solved " << equations.size() << " equations with "
                          << current_params.size() << " unknowns in " << steps << " steps.\n";
            }
            stats += "eqs: " + std::to_string(equations.size())
                     + "\nnunkn: " + std::to_string(current_params.size());
            back_substitution(subs);
            if (DEBUG)
            {
                for (std::size_t i = 0; i < J.shape(0); ++i)
                {
                    for (std::size_t j = 0; j < J.shape(1); ++j)
                        std::cout << J(i, j)->to_string() << ", ";
                    std::cout << "\n";
                }

                std::cout << "Params: \n";
                for (int i = 0; i < current_params.size(); i++)
                {
                    std::cout << current_params[i]->to_string() << std::endl;
                }
            }

            return SolveResult::OKAY;
        }
        eval_jacobian(J, A, !is_drag_step);
        solve_least_squares(A, B, X);

        for (int i = 0; i < current_params.size(); i++)
        {
            current_params[i]->set_value(current_params[i]->value() - X(i));
        }
    } while (steps++ <= max_steps);

    if (DEBUG)
    {
        for (std::size_t i = 0; i < J.shape(0); ++i)
        {
            for (std::size_t j = 0; j < J.shape(1); ++j)
                std::cout << J(i, j)->to_string() << ", ";
            std::cout << "\n";
        }

        std::cout << "Params: \n";
        for (int i = 0; i < current_params.size(); i++)
        {
            std::cout << current_params[i]->to_string() << std::endl;
        }
    }

    is_converged(false, true);

    if (revert_when_not_converged)
    {
        revert_params();
        dof_changed = false;
    }

    return SolveResult::DIDNT_CONVERGE;
}
