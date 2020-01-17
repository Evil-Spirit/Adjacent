#include "entity.hpp"
#include "expression.hpp"
#include "equation_system.hpp"

#ifndef ADJACENT_CONSTRAINT_HPP
#define ADJACENT_CONSTRAINT_HPP

class Constraint {
public:
    std::vector<Entity*> entities;
};

class ValueConstraint : public Constraint {
protected:
    ParamPtr value = param("c_value", 0);

public:

    bool reference;

    void set_reference(bool value) {
        reference = value;
        // mark dirty
    } 

    virtual bool on_satisfy() = 0;

    bool satisfy() {
        bool result = on_satisfy();
        if(!result) {
            std::cout << "satisfy failed!"; // << GetType() +;
        }
        return result;
    }

    std::vector<ParamPtr> parameters() {
        if (!reference) return {};
        else {
            return std::vector<ParamPtr>({value});
        }
    }

    // public override IEnumerable<Param> parameters {
    //  get {
    //      if(!reference) yield break;
    //      yield return value;
    //  }
    // }

    void set_value(double v) {
        // label to value for helix not implemented ...
        value->set_value(v);
    }
};

class PointOnConstraint : public ValueConstraint {
public:
    Entity* point;
    Entity* on;
    PointOnConstraint(Entity* point, Entity* on)
        : point(point), on(on)
    {
        reference = true;
        entities.push_back(point);
        entities.push_back(on);
        set_value(0.51);
        satisfy();
    }

    bool on_satisfy() {
        EquationSystem sys;
        sys.add_parameters(parameters());
        auto exprs = equations();
        sys.add_equations(exprs);

        double bestI = 0.0;
        double min = -1.0;
        for(double i = 0.0; i <= 1.0; i += 0.25 / 2.0) {
            value->set_value(i);
            sys.solve();
            double cur_value = 0;
            for (const auto& e : exprs) {
                cur_value += abs(e->eval());
            }
            if(min >= 0.0 && min < cur_value) continue;
            bestI = value->value();
            min = cur_value;
        }
        std::cout << "value : " << bestI << std::endl;
        value->set_value(bestI);
        return true;
    }

    std::vector<ExprPtr> equations() {
        std::vector<ExprPtr> res;
        // var eq = on.PointOnInPlane(value, sketch.plane) - p;
        ExpVector equation = *on->point_on(value->expr()) - ((PointE*)point)->expr();
        res.push_back(equation.x);
        res.push_back(equation.y);
        // if(sketch.is3d) yield return eq.z;
        return res;
    }
};

#endif