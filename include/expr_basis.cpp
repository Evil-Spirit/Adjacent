#include <string>

#include "expression.hpp"
#include "expression_vector.hpp"
#include "equation_system.hpp"

class ExpBasis {
public:
	std::shared_ptr<Param<double>> px, py, pz,
								   ux, uy, uz,
								   vx, vy, vz,
								   nx, ny, nz;
	ExpVector p, u, v, n;

	ExpBasis() :
		ExpBasis({0., 0., 0., // pxyz
			      1., 0., 0., // uxyz
			  	  0., 1., 0., // vxyz
			  	  0., 0., 1.  // nxyz
		})
	{
	}

	ExpBasis(const double (&params)[12]) :
		px(param("px", params[0])),
		py(param("py", params[1])),
		pz(param("pz", params[2])),
		ux(param("ux", params[3])),
		uy(param("uy", params[4])),
		uz(param("uz", params[5])),
		vx(param("vx", params[6])),
		vy(param("vy", params[7])),
		vz(param("vz", params[8])),
		nx(param("nx", params[9])),
		ny(param("ny", params[10])),
		nz(param("nz", params[11])),
		p(px->expr(), py->expr(), pz->expr()),
		u(ux->expr(), uy->expr(), uz->expr()),
		v(vx->expr(), vy->expr(), vz->expr()),
		n(nx->expr(), ny->expr(), nz->expr())
	{
	}

	// public Matrix4x4 matrix {
	// 	get {
	// 		return UnityExt.Basis(u.Eval(), v.Eval(), n.Eval(), p.Eval());
	// 	}
	// }

	template <class L>
	void apply_each_param(L&& l)
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
		apply_each_param([&result](auto& p) {
			result += std::to_string(p->value()) + " ";
		});
		return result;
	}

	static ExpBasis FromString(const std::string& str)
	{
		std::istringstream iss(str);
		double p[12];
		for (std::size_t i = 0; i < 12; ++i) iss >> p[i];
		return ExpBasis(p);
	}

	ExpVector transform_position(const ExpVector& pos) {
		return pos.x * u + pos.y * v + pos.z * n + p;
	}

	ExpVector transform_direction(const ExpVector& dir) {
		return dir.x * u + dir.y * v + dir.z * n;
	}

	void generate_equations(EquationSystem& sys) {
		apply_each_param([&sys](auto& p) {
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
		apply_each_param([&changed](auto& p) {
			if (!changed && p->m_changed)
			{
				changed = true;
			}
		});
		return changed;
	}

	void mark_unchanged() {
		apply_each_param([](auto& p) {
			p->m_changed = false;
		});
	}
};

class ExpBasis2d {
	std::shared_ptr<Param<double>> px, py,
								   ux, uy,
								   vx, vy;
	ExpVector u, v, p;

	ExpBasis2d() :
		ExpBasis2d({0., 0., 1., 0., 0. , 1.})
	{
	}

	ExpBasis2d(const double (&params)[6]) :
		px(param("px", params[0])),
		py(param("py", params[1])),
		ux(param("ux", params[2])),
		uy(param("uy", params[3])),
		vx(param("vx", params[4])),
		vy(param("vy", params[5])),
		p(px->expr(), py->expr(), zero),
		u(ux->expr(), uy->expr(), zero),
		v(vx->expr(), vy->expr(), zero)
	{
	}

	void set_pose_param(const std::shared_ptr<Param<double>>& x, const std::shared_ptr<Param<double>>& y) {
		px = x;
		py = y;
		p.x = px->expr();
		p.y = py->expr();
	}

	template <class L>
	void apply_each_param(L&& l)
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
		apply_each_param([&result](auto& p) {
			result += std::to_string(p->value()) + " ";
		});
		return result;
	}

	static ExpBasis2d from_string(const std::string& str)
	{
		std::istringstream iss(str);
		double p[6];
		iss >> p[0] >> p[1] >> p[2] >> p[3] >> p[4] >> p[5];
		return ExpBasis2d(p);
	}

	ExpVector transform_position(const ExpVector& pos) {
		return pos.x * u + pos.y * v + p;
	}

	ExpVector TransformDirection(const ExpVector& dir) {
		return dir.x * u + dir.y * v;
	}

	void generate_equations(EquationSystem& sys)
	{
		apply_each_param([&sys](auto& p) {
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
		apply_each_param([&changed](auto& p) {
			if (!changed && p->m_changed)
			{
				changed = true;
			}
		});
		return changed;
	}

	void mark_unchanged() {
		apply_each_param([](auto& p) {
			p->m_changed = false;
		});
	}
};
