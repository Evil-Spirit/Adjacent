#include <string>

#include "expression.hpp"
#include "expression_vector.hpp"
#include "equation_system.hpp"

class ExpBasis {
public:
	Param<double> px, py, pz;
	Param<double> ux, uy, uz;
	Param<double> vx, vy, vz;
	Param<double> nx, ny, nz;

	ExpVector u;
	ExpVector v;
	ExpVector n;
	ExpVector p;

	ExpBasis() :
		px("px", 0.0),
		py("py", 0.0),
		pz("pz", 0.0),
		ux("ux", 1.0),
		uy("uy", 0.0),
		uz("uz", 0.0),
		vx("vx", 0.0),
		vy("vy", 1.0),
		vz("vz", 0.0),
		nx("nx", 0.0),
		ny("ny", 0.0),
		nz("nz", 1.0),
		p(px.expr(), py.expr(), pz.expr()),
		u(ux.expr(), uy.expr(), uz.expr()),
		v(vx.expr(), vy.expr(), vz.expr()),
		n(nx.expr(), ny.expr(), nz.expr())
	{
		// why px == ux?
		// px = new Param("ux", 0.0);
		// py = new Param("uy", 0.0);
		// pz = new Param("uz", 0.0);
	}

	// public Matrix4x4 matrix {
	// 	get {
	// 		return UnityExt.Basis(u.Eval(), v.Eval(), n.Eval(), p.Eval());
	// 	}
	// }

	template <class L>
	void apply(L&& l)
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

	std::string to_string() {
		std::string result = "";
		apply([&result](auto& p) {
			result += std::to_string(p.value()) + " ";
		});
		return result;
	}

	// public void FromString(string str) {
	// 	char[] sep = { ' ' };
	// 	var values = str.Split(sep, StringSplitOptions.RemoveEmptyEntries);
	// 	int i = 0;
	// 	foreach(var p in parameters) {
	// 		p.value = values[i].ToDouble();
	// 		i++;
	// 	}
	// }

	ExpVector transform_position(const ExpVector& pos) {
		return pos.x * u + pos.y * v + pos.z * n + p;
	}

	ExpVector transform_direction(const ExpVector& dir) {
		return dir.x * u + dir.y * v + dir.z * n;
	}

	void generate_equations(EquationSystem& sys) {
		apply([&sys](auto& p) {
			sys.add_parameter(p);
		});

		sys.add_equation(u.magnitude() - one);
		sys.add_equation(v.magnitude() - one);

		auto c = cross(u, v);
		auto d = dot(u, v);
		sys.add_equation(atan2(c.magnitude(), d) - expr(M_PI) / two);
		sys.add_equation(n - cross(u, v));
	}

	bool changed()
	{
		bool changed = false;
		apply([&changed](auto& p) {
			if (!changed && p.m_changed)
			{
				changed = true;
			}
		});
		return changed;
	}

	void mark_unchanged() {
		apply([](auto& p) {
			p.m_changed = false;
		});
	}
};

class ExpBasis2d {
	Param<double> px, py;
	Param<double> ux, uy;
	Param<double> vx, vy;

	ExpVector u, v, p;

	ExpBasis2d() :
		px("px", 0.0),
		py("py", 0.0),
		ux("ux", 1.0),
		uy("uy", 0.0),
		vx("vx", 0.0),
		vy("vy", 1.0),
		p(px.expr(), py.expr(), zero),
		u(ux.expr(), uy.expr(), zero),
		v(vx.expr(), vy.expr(), zero)
	{
	}

	void set_pose_param(const Param<double>& x, const Param<double>& y) {
		px = x;
		py = y;
		p.x = px.expr();
		p.y = py.expr();
	}

	template <class L>
	void apply(L&& l)
	{
		l(px);
		l(py);
		l(ux);
		l(uy);
		l(vx);
		l(vy);
	}

// 	/*
// 	public Matrix4x4 matrix {
// 		get {
// 			return UnityExt.Basis(u.Eval(), v.Eval(), n.Eval(), p.Eval());
// 		}
// 	}*/

	std::string to_string()
	{
		std::string result = "";
		apply([&result](auto& p) {
			result += std::to_string(p.value()) + " ";
		});
		return result;
	}

// 	public void FromString(string str) {
// 		char[] sep = { ' ' };
// 		var values = str.Split(sep, StringSplitOptions.RemoveEmptyEntries);
// 		int i = 0;
// 		foreach(var p in parameters) {
// 			p.value = values[i].ToDouble();
// 			i++;
// 		}
// 	}

	ExpVector transform_position(const ExpVector& pos) {
		return pos.x * u + pos.y * v + p;
	}

	ExpVector TransformDirection(const ExpVector& dir) {
		return dir.x * u + dir.y * v;
	}

	void generate_equations(EquationSystem& sys)
	{
		apply([&sys](auto& p) {
			sys.add_parameter(p);
		});
		sys.add_equation(u.magnitude() - one);
		sys.add_equation(v.magnitude() - one);

		auto c = cross(u, v);
		auto d = dot(u, v);
		sys.add_equation(atan2(c.magnitude(), d) - expr(M_PI) / two);
	}

	bool changed()
	{
		bool changed = false;
		apply([&changed](auto& p) {
			if (!changed && p.m_changed)
			{
				changed = true;
			}
		});
		return changed;
	}

	void mark_unchanged() {
		apply([](auto& p) {
			p.m_changed = false;
		});
	}
};
