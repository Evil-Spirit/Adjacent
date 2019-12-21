#include <vector>
#include <unordered_map>

#include <xtensor/xtensor.hpp>
#include "expression.hpp"
#include "expression_vector.hpp"
#include "gaussian_method.hpp"
#include "equation_system.hpp"

void EquationSystem::add_equation(const std::shared_ptr<Expr>& eq)
{
	source_equations.push_back(eq);
	is_dirty = true;
}

void EquationSystem::add_equation(const ExpVector& v) {
	source_equations.push_back(v.x);
	source_equations.push_back(v.y);
	source_equations.push_back(v.z);
	is_dirty = true;
}

// public void AddEquations(IEnumerable<Exp> eq) {
// 	source_equations.AddRange(eq);
// 	is_dirty = true;
// }

// public void RemoveEquation(Exp eq) {
// 	source_equations.Remove(eq);
// 	is_dirty = true;
// }

void EquationSystem::add_parameter(const Param<double>& p)
{
	parameters.push_back(p);
	is_dirty = true;
}

// public void AddParameters(IEnumerable<Param> p) {
// 	parameters.AddRange(p);
// 	is_dirty = true;
// }

// public void RemoveParameter(Param p) {
// 	parameters.Remove(p);
// 	is_dirty = true;
// }

void EquationSystem::eval(xt::xtensor<double, 1>& B, bool clear_drag) {
	B.resize({equations.size()});
	for(int i = 0; i < equations.size(); i++) {
		if(clear_drag && equations[i]->is_drag()) {
			B(i) = 0.0;
			continue;
		}
		B(i) = equations[i]->eval();
	}
}

bool EquationSystem::is_converged(bool check_drag, bool print_non_converged /* = false*/) {
	for(int i = 0; i < equations.size(); i++) {
		if(!check_drag && equations[i]->is_drag())
		{
			continue;
		}

		if(std::abs(B(i)) < GaussianMethod::epsilon) continue;
		
		if(print_non_converged)
		{
			std::cout << "Not converged: " + equations[i]->to_string();
			continue;
			// continue; ???
		}
		return false;
	}
	return true;
}

void EquationSystem::store_params() {
	for(std::size_t i = 0; i < parameters.size(); i++) {
		old_param_value[i] = parameters[i].value();
	}
}

void EquationSystem::revert_params() {
	for(std::size_t i = 0; i < parameters.size(); i++) {
		parameters[i].set_value(old_param_value[i]);
	}
}

xt::xtensor<std::shared_ptr<Expr>, 2> EquationSystem::write_jacobian(const std::vector<std::shared_ptr<Expr>>& equations,
											         const std::vector<Param<double>>& parameters) {
	xt::xtensor<std::shared_ptr<Expr>, 2> J = xt::empty<std::shared_ptr<Expr>>({equations.size(), parameters.size()});
	for(std::size_t r = 0; r < equations.size(); r++) {
		const auto& eq = equations[r];
		for(std::size_t c = 0; c < parameters.size(); c++) {
			const auto& u = parameters[c];
			J(r, c) = eq->derivative(u);
			/*
			if(!J[r, c].IsZeroConst()) {
				Debug.Log(J[r, c].ToString() + "\n");
			}
			*/
		}
	}
	return J;
}

bool EquationSystem::has_dragged() {
	return std::any_of(equations.begin(), equations.end(), [](auto& e) { return e->is_drag(); });
}

void EquationSystem::eval_jacobian(const xt::xtensor<expr_ptr, 2>& J, xt::xtensor<double, 2>& A, bool clear_drag) {
	update_dirty();
	for(std::size_t r = 0; r < J.shape(0); r++) {
		if(clear_drag && equations[r]->is_drag()) {
			for(std::size_t c = 0; c < J.shape(1); c++) {
				A(r, c) = 0.0;
			}
			continue;
		}
		for(std::size_t c = 0; c < J.shape(1); c++) {
			A(r, c) = J(r, c)->eval();
		}
	}
}

void EquationSystem::solve_least_squares(const xt::xtensor<double, 2>& A,
								const xt::xtensor<double, 1>& B,
								xt::xtensor<double, 1>& X) {
	// A^T * A * X = A^T * B
	std::size_t rows = A.shape(0);
	std::size_t cols = A.shape(1);

	for(std::size_t r = 0; r < rows; r++) {
		// only up to `rows`?
		for(std::size_t c = 0; c < rows; c++) {
			double sum = 0.0;
			for(std::size_t i = 0; i < cols; i++) {
				if(A(c, i) == 0 || A(r, i) == 0) continue;
				sum += A(r, i) * A(c, i);
			}
			AAT(r, c) = sum;
		}
	}

	GaussianMethod::solve(AAT, B, Z);

	for(int c = 0; c < cols; c++) {
		double sum = 0.0;
		for(int r = 0; r < rows; r++) {
			sum += Z(r) * A(r, c);
		}
		X(c) = sum;
	}

}

void EquationSystem::clear() {
	parameters.clear();
	current_params.clear();
	equations.clear();
	source_equations.clear();
	is_dirty = true;
	update_dirty();
}

bool EquationSystem::test_rank(int& dof) {
	eval_jacobian(J, A, false);
	int rank = GaussianMethod::rank(A);
	dof = A.shape(1) - rank;
	return rank == A.shape(0);
}

void EquationSystem::update_dirty() {
	if (is_dirty)
	{
		// equations = source_equations.Select(e => e.DeepClone()).ToList();
		equations = source_equations;
		current_params = parameters;
		/*
		foreach(var e in equations) {
			e.ReduceParams(current_params);
		}*/
		//current_params = parameters.Where(p => equations.Any(e => e.IsDependOn(p))).ToList();
		subs = solve_by_substitution();

		J = write_jacobian(equations, current_params);
		A = xt::empty<double>(J.shape());
		B = xt::empty<double>({equations.size()});
		X = xt::empty<double>({current_params.size()});
		Z = xt::empty<double>({A.shape(0)});
		AAT = xt::empty<double>({A.shape(0), A.shape(0)});
		old_param_value = xt::empty<double>({parameters.size()});
		is_dirty = false;
		dof_changed = true;
	}
}

void EquationSystem::back_substitution(std::unordered_map<Param<double>, Param<double>>& subs) {
	if(subs.empty()) return;
	for(std::size_t i = 0; i < parameters.size(); i++) {
		auto& p = parameters[i];
		if(subs.find(p) == subs.end()) continue;
		p.set_value(subs[p].value());
	}
}

std::unordered_map<Param<double>, Param<double>> EquationSystem::solve_by_substitution()
{
	std::unordered_map<Param<double>, Param<double>> subs;

	for(std::size_t i = 0; i < equations.size(); i++) {
		auto& eq = equations[i];
		if(!eq->is_substitution_form()) continue;
		auto* a = eq->get_substitution_param_a();
		auto* b = eq->get_substitution_param_b();
		if(std::abs(a->value() - b->value()) > GaussianMethod::epsilon) continue;
		// check if b in current params
		if(std::find(current_params.begin(), current_params.end(), *b) != current_params.end())
		{
			// check if pointer swap is enough?!
			std::swap(a, b);
		}
		// TODO: Check errors
		//if(!parameters.Contains(b)) {
		//	continue;
		//}

		for (auto& kv : subs)
		{
			if (subs[kv.first] == *b)
			{
				subs[kv.first] = *a;
			}
		}
		subs[*b] = *a;

		equations.erase(equations.begin() + i);
		i--;
		current_params.erase(std::find(current_params.begin(), current_params.end(), *b));
		for(std::size_t j = 0; j < equations.size(); j++) {
			equations[j]->substitute(*b, *a);
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
	do {
		bool is_drag_step = steps <= drag_steps;
		eval(B, /*clear_drag*/!is_drag_step);
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
				// Debug.Log(String.Format("solved {0} equations with {1} unknowns in {2} steps", equations.Count, currentParams.Count, steps));
			}
			stats += "eqs: " + std::to_string(equations.size()) + "\nnunkn: " + std::to_string(current_params.size());
			back_substitution(subs);
			return SolveResult::OKAY;
		}
		eval_jacobian(J, A, !is_drag_step);
		solve_least_squares(A, B, X);
		for(int i = 0; i < current_params.size(); i++)
		{
			current_params[i].set_value(current_params[i].value() - X[i]);
		}
	} while(steps++ <= max_steps);
	is_converged(false, true);
	if(revert_when_not_converged) {
		revert_params();
		dof_changed = false;
	}
	return SolveResult::DIDNT_CONVERGE;

}
