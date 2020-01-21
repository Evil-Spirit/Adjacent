#include <set>

#include "entity.hpp"
#include "expression.hpp"
#include "equation_system.hpp"

#ifndef ADJACENT_CONSTRAINT_HPP
#define ADJACENT_CONSTRAINT_HPP

enum CONSTRAINT_TYPE {
    INVALID,
    PointOn,
    PointsCoincident,
    Parallel,
    Length,
    PointsDistance,
    HV
};

class Constraint;

using ConstraintPtr = std::shared_ptr<Constraint>;
using EntityPtr = std::shared_ptr<Entity>;

class Constraint {
public:
    CONSTRAINT_TYPE type;
    std::vector<Entity*> entities;

    Constraint(CONSTRAINT_TYPE type) :
        type(type)
    {
    }

    virtual std::vector<ParamPtr> parameters() = 0;
    virtual std::vector<ExprPtr> equations() = 0;
};

class ValueConstraint : public Constraint {
public:
    ParamPtr value = param("c_value", 0);

    bool reference;

    ValueConstraint(CONSTRAINT_TYPE type) :
        Constraint(type)
    {
    }

    ValueConstraint(CONSTRAINT_TYPE type, double v) :
        Constraint(type)
    {
        value->set_value(v);
    }

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

    void set_value(double v) {
        // label to value for helix not implemented ...
        value->set_value(v);
    }
};

class PointOnConstraint : public ValueConstraint {
public:
    std::shared_ptr<Entity> point;
    std::shared_ptr<Entity> on;

    PointOnConstraint(std::shared_ptr<Entity> point, std::shared_ptr<Entity> on)
        : ValueConstraint::ValueConstraint(PointOn), point(point), on(on)
    {
        reference = true;
        entities.push_back(point.get());
        entities.push_back(on.get());
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
        ExpVector equation = *on->point_on(value->expr()) - ((PointE*)point.get())->expr();
        res.push_back(equation.x);
        res.push_back(equation.y);
        // if(sketch.is3d) yield return eq.z;
        return res;
    }
};


ExprPtr angle2d(const ExpVector& d0, const ExpVector& d1, bool angle360 = false) {
    auto nu = d1.x * d0.x + d1.y * d0.y;
    auto nv = d0.x * d1.y - d0.y * d1.x;
    if (angle360) return PI_E - atan2(nv, -nu);
    return atan2(nv, nu);
}

class Parallel : public Constraint {

    enum Option {
        Codirected,
        Antidirected
    };

    Option option_;

    ExprPtr angle;
    std::shared_ptr<Entity> l0, l1;

    Parallel(std::shared_ptr<Entity> l0, std::shared_ptr<Entity> l1) : 
        Constraint(CONSTRAINT_TYPE::Parallel), l0(l0), l1(l1)
    {
        entities.push_back(l0.get());
        entities.push_back(l1.get());
        choose_best_option();
    }

    std::vector<ExprPtr> equations()
    {
        // ExpVector d0 = l0.GetPointAtInPlane(0, sketch.plane) - l0.GetPointAtInPlane(1, sketch.plane);
        // ExpVector d1 = l1.GetPointAtInPlane(0, sketch.plane) - l1.GetPointAtInPlane(1, sketch.plane);
        ExpVector d0 = *l0->point_on(zero) - *l0->point_on(one);
        ExpVector d1 = *l1->point_on(zero) - *l1->point_on(one);
        // ExprPtr angle = sketch.is3d ? ConstraintExp.angle3d(d0, d1) : ConstraintExp.angle2d(d0, d1);
        if (!angle) {
            angle = angle2d(d0, d1);
        }
        switch(option_) {
            case Option::Codirected: return std::vector<ExprPtr>({angle});
            case Option::Antidirected: return std::vector<ExprPtr>({abs(angle) - PI_E});
        }
    }

    void choose_best_option() {
      
        double min_value = -1.0;
        int best_option = 0;
        
        for (int i = 0; i < 2; i++) {
            auto exprs = equations();
            double cur_value = 0.0;
            for (const auto& el : exprs) {
                cur_value += abs(el->eval());
            }
            std::clog << "check option " << i << " (min: " << min_value << ", cur: " << cur_value << ")\n";
            if (min_value < 0.0 || cur_value < min_value) {
                min_value = cur_value;
                best_option = i;
                option_ = (Option)i;
            }
        }
    }
};

class LengthConstraint : public ValueConstraint {
public:
    // public ExpVector p0exp { get { return GetPointInPlane(0, sketch.plane); } }
    // public ExpVector p1exp { get { return GetPointInPlane(1, sketch.plane); } }

    std::shared_ptr<Entity> entity;

    LengthConstraint(std::shared_ptr<Entity> e, double l) :
        ValueConstraint(CONSTRAINT_TYPE::Length, l),
        entity(e)
    {
        entities.push_back(e.get());
        satisfy();
    }

    bool on_satisfy() {
        return true;
    }

    std::vector<ExprPtr> equations() {
         return std::vector<ExprPtr>({entity->length() - value->expr()});
    }
    
    // ExpVector GetPointInPlane(int i, IPlane plane) {
    //     return GetEntity(0).GetPointAtInPlane(i, plane);
    // }

    // protected override Matrix4x4 OnGetBasis() {
    //     return sketch.plane.GetTransform();
    // }
};

class PointsCoincident : public Constraint {

    std::shared_ptr<PointE> p0, p1;

    PointsCoincident(const std::shared_ptr<PointE>& p0, const std::shared_ptr<PointE>& p1) :
        Constraint(CONSTRAINT_TYPE::PointsCoincident), p0(p0), p1(p1)
    {
        entities.push_back(p0.get());
        entities.push_back(p1.get());
    }

    std::vector<ExprPtr> equations() {
        // var pe0 = p0.GetPointAtInPlane(0, sketch.plane);
        // var pe1 = p1.GetPointAtInPlane(0, sketch.plane);

        return std::vector<ExprPtr>({
            p0->x->expr() - p1->x->expr(),
            p0->y->expr() - p1->y->expr()
        });
        // if 3d
        // if(sketch.is3d) yield return pe0.z - pe1.z;
    }

    std::shared_ptr<PointE>& get_other_point(const std::shared_ptr<PointE>& p) {
        if(p0 == p) return p1;
        return p0;
    }
};

class PointsDistance : public ValueConstraint {
public:

    EntityPtr p0, p1;

    PointsDistance(const EntityPtr& p0, const EntityPtr& p1, double d) :
        ValueConstraint(CONSTRAINT_TYPE::PointsDistance, d), p0(p0), p1(p1)
    {
        entities.push_back(p0.get());
        entities.push_back(p1.get());
        satisfy();
    }

    PointsDistance(const EntityPtr& line) : 
        ValueConstraint(CONSTRAINT_TYPE::PointsDistance), p0(line), p1(nullptr)
    {
        entities.push_back(line.get());
        satisfy();
    }

    std::vector<ExprPtr> equations() {
        return std::vector<ExprPtr>({
            // TODO caching
            (get_point(1) - get_point(0)).magnitude() - value->expr()
        });
    }

    ExpVector get_point(double i) {
        if (p1 == nullptr) {
            return i ? dynamic_cast<LineE*>(p0.get())->source().expr() : dynamic_cast<LineE*>(p0.get())->target().expr();
        }
        else {
            return i ? dynamic_cast<PointE*>(p0.get())->expr() : dynamic_cast<PointE*>(p1.get())->expr();
        }
    }            
};

enum HVOrientation {
    OX,
    OY,
    // OZ
};

class HVConstraint : public Constraint {
public:

    PointE* p0;
    PointE* p1;

    HVOrientation orientation = HVOrientation::OX;

    HVConstraint(std::shared_ptr<PointE> p0, std::shared_ptr<PointE> p1, HVOrientation o) :
        Constraint(CONSTRAINT_TYPE::HV), p0(p0.get()), p1(p1.get()), orientation(o)
    {
        entities.push_back(p0.get());
        entities.push_back(p1.get());
    }

    HVConstraint(std::shared_ptr<LineE> line, HVOrientation o) : 
        Constraint(CONSTRAINT_TYPE::HV), p0(&(line.get())->source()), p1(&(line.get())->target()), orientation(o) {
        entities.push_back(line.get());
    }

    std::vector<ExprPtr> equations() {
        ExprPtr exp;
        switch(orientation) {
            case HVOrientation::OX: exp = p0->x->expr() - p1->x->expr(); break;
            case HVOrientation::OY: exp = p0->y->expr() - p1->y->expr(); break;
            // case HVOrientation::OZ: exp = p0->z->expr() - p1->z->expr(); break;
        }

        return std::vector<ExprPtr>({ exp });
    }

    std::vector<ParamPtr> parameters() {
        return {};
    }
};





class Sketch {
public:

    bool constraintsTopologyChanged = true;
    bool constraintsChanged = true;
    bool entitiesChanged = true;
    bool loopsChanged = true;
    bool topologyChanged = true;
    bool supressSolve;
    EquationSystem sys;

    std::set<EntityPtr> entities;
    std::set<ConstraintPtr> constraints;

    void add_entity(const EntityPtr& e) {
        if(entities.find(e) != entities.end()) return;
        entities.insert(e);
        mark_dirty(/*topo*/true, /*constraints*/false, /*entities*/true, /*loops*/false);
    }

    void mark_dirty(bool topo, bool constraints, bool entities, bool loops) {
        topologyChanged = topologyChanged || topo;
        constraintsChanged = constraintsChanged || constraints;
        constraintsTopologyChanged = constraintsTopologyChanged || constraints;
        entitiesChanged = entitiesChanged || entities;
        loopsChanged = loopsChanged || loops;
    }

    void add_constraint(const ConstraintPtr& c) {
        if(constraints.find(c) != constraints.end()) return;
        constraints.insert(c);
        mark_dirty(/*topo*/c->type == PointsCoincident, /*constraints*/true, /*entities*/false, /*loops*/false);
        constraintsTopologyChanged = true;
    }

    bool is_dirty() const {
        return constraintsTopologyChanged || constraintsChanged || entitiesChanged || loopsChanged || topologyChanged;
    }

    bool is_entities_changed() const {
        return entitiesChanged;
    }

    bool is_constraints_changed() const {
        return constraintsChanged;
    }
    bool is_topology_changed() const {
        return topologyChanged;
    }

    void update() {
        if(is_constraints_changed() || is_entities_changed()) {
            supressSolve = false;
        }
        if(is_topology_changed()) {
            sys.clear();
            generate_equations(sys);
        }
        auto res = (!supressSolve || sys.has_dragged()) ? sys.solve() : DIDNT_CONVERGE;
        if(res == DIDNT_CONVERGE) {
            supressSolve = true;
        }
    }

    void generate_equations(EquationSystem& system) {
        for (const auto& en : entities) {
            system.add_parameters(en->parameters());
            // system.add_equations(en->equations());
        }
        for (const auto& c : constraints) {
            system.add_parameters(c->parameters());
            system.add_equations(c->equations());
        }
    }
};

#endif