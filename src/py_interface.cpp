#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "expression.hpp"
#include "entity.hpp"
#include "constraint.hpp"


namespace py = pybind11;

PYBIND11_MODULE(adjacent_api, m) {
    py::class_<Sketch>(m, "Sketch")
    	.def(py::init<>())
    	.def("add_entity", &Sketch::add_entity)
    	.def("add_constraint", &Sketch::add_constraint)
    	.def("update", &Sketch::update)
    ;

    using Prm = Param<double>;
    py::class_<Prm, std::shared_ptr<Prm>>(m, "Param")
    	.def(py::init<std::string, double>())
    	.def("set_value", &Prm::set_value)
    	.def("value", &Prm::value)
    	.def("name", [](Prm& self) {
    		return self.m_name;
    	})
    	.def("__repr__", &Prm::to_string)
    ;

    py::class_<Entity, std::shared_ptr<Entity>>(m, "Entity")
    ;

    py::class_<PointE, Entity, std::shared_ptr<PointE>>(m, "Point")
    	.def(py::init<ParamPtr, ParamPtr, ParamPtr>())
    	.def("expr", &PointE::expr)
        .def("eval", [](PointE& p) {
            return std::vector<double>({p.x->value(), p.y->value()});
        })
    	.def("__repr__", &PointE::to_string)
    ;

    py::class_<LineE, Entity, std::shared_ptr<LineE>>(m, "Line")
    	.def(py::init<PointE, PointE>())
        .def("source", &LineE::source)
        .def("target", &LineE::target)
    	// .def("expr", &LineE::expr)
    	.def("__repr__", &LineE::to_string)
    ;

    py::module sub = m.def_submodule("constraints");

    py::class_<Constraint, std::shared_ptr<Constraint>>(sub, "Constraint")
    ;

    py::class_<ValueConstraint, std::shared_ptr<ValueConstraint>>(sub, "ValueConstraint")
    ;

    py::class_<PointOnConstraint, ValueConstraint, Constraint, std::shared_ptr<PointOnConstraint>>(sub, "PointOn")
    	.def(py::init<EntityPtr, EntityPtr>())
    ;

    py::class_<LengthConstraint, ValueConstraint, Constraint, std::shared_ptr<LengthConstraint>>(sub, "Length")
        .def(py::init<EntityPtr, double>())
    ;

    py::class_<PointsCoincidentConstraint, Constraint, std::shared_ptr<PointsCoincidentConstraint>>(sub, "Coincident")
        .def(py::init<std::shared_ptr<PointE>&, std::shared_ptr<PointE>&>())
    ;

    py::class_<PointsDistanceConstraint, ValueConstraint, Constraint, std::shared_ptr<PointsDistanceConstraint>>(sub, "Distance")
        .def(py::init<std::shared_ptr<PointE>&, std::shared_ptr<PointE>&, double>())
        .def(py::init<std::shared_ptr<LineE>&, double>())
    ;

    // py::class_<AngleConstraint, ValueConstraint, Constraint, std::shared_ptr<AngleConstraint>>(sub, "Angle")
    //     .def(py::init<LineE, LineE, double>())
    //     .def(py::init<EntityPtr, double>())
    // ;

    py::enum_<HVOrientation>(sub, "HVOrientation")  
        .value("OX", HVOrientation::OX)
        .value("OY", HVOrientation::OY)
    ;

    py::class_<HVConstraint, Constraint, std::shared_ptr<HVConstraint>>(sub, "HV")
        .def(py::init<std::shared_ptr<PointE>, std::shared_ptr<PointE>, HVOrientation>())
        .def(py::init<std::shared_ptr<LineE>, HVOrientation>())
    ;

    py::class_<Expr, std::shared_ptr<Expr>>(m, "Expr")
    	// .def(py::init<>())
    	.def(py::init<double>())
    	.def("eval", &Expr::eval)
    	.def("__str__", &Expr::to_string)
    	.def("__repr__", &Expr::to_string)
    ;

    py::class_<ExpVector, std::shared_ptr<ExpVector>>(m, "ExprVector")
        .def("__repr__", [](ExpVector& self) -> std::string {
            std::string res = "{\n";
            res += self.x->to_string() + "\n";
            res += self.y->to_string() + "\n";
            res += self.z->to_string() + "\n";
            res += "}";
            return res;
        })
    ;

    // py::implicitly_convertible<LineE, Entity>();
    // py::implicitly_convertible<PointE, Entity>();
}