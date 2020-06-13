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
    Length
};

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
protected:
    ParamPtr value = param("c_value", 0);

public:

    bool reference;

    ValueConstraint(CONSTRAINT_TYPE type) :
        Constraint(type)
    {
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
            return { value };
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
        // TODO Add runtime check that point is point, and on is some other entity! 
        reference = true;
        entities.push_back(point.get());
        entities.push_back(on.get());
        set_value(0.51);
        satisfy();
    }

    bool on_satisfy() {
        EquationSystem sys;
        auto params = parameters();
        sys.add_parameters(params);
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
        throw std::runtime_error("unhandled option");
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

class Length : public ValueConstraint {
public:
    std::shared_ptr<Entity> entity;

    Length(const std::shared_ptr<Entity>& e) :
        ValueConstraint(CONSTRAINT_TYPE::Length),
        entity(e)
    {
        entities.push_back(e.get());
        satisfy();
    }

    std::vector<ExprPtr> equations() {
         return std::vector<ExprPtr>({entity->length() - value->expr()});
    }
};

// public class AngleConstraint : ValueConstraint {

//     bool supplementary_;
//     public bool supplementary {
//         get {
//             return supplementary_;
//         }
//         set {
//             if(value == supplementary_) return;
//             supplementary_ = value;
//             if(HasEntitiesOfType(IEntityType.Arc, 1)) {
//                 this.value.value = 2.0 * Math.PI - this.value.value;
//             } else {
//                 this.value.value = -(Math.Sign(this.value.value) * Math.PI - this.value.value);
//             }
//             sketch.MarkDirtySketch(topo:true);
//         }
//     }

//     public AngleConstraint(Sketch sk, IEntity[] points) : base(sk) {
//         foreach(var p in points) {
//             AddEntity(p);
//         }
//         Satisfy();
//     }

//     public AngleConstraint(Sketch sk, IEntity arc) : base(sk) {
//         AddEntity(arc);
//         value.value = Math.PI / 4;
//         Satisfy();
//     }

//     public AngleConstraint(Sketch sk, IEntity l0, IEntity l1) : base(sk) {
//         AddEntity(l0);
//         AddEntity(l1);
//         Satisfy();
//     }

//     public override IEnumerable<Exp> equations {
//         get {
//             var p = GetPointsExp(sketch.plane);
//             ExpVector d0 = p[0] - p[1];
//             ExpVector d1 = p[3] - p[2];
//             bool angle360 = HasEntitiesOfType(IEntityType.Arc, 1);
//             Exp angle = sketch.is3d ? ConstraintExp.angle3d(d0, d1) : ConstraintExp.angle2d(d0, d1, angle360);
//             yield return angle - value;
//         }
//     }

//     Vector3[] GetPointsInPlane(IPlane plane) {
//         return GetPointsExp(plane).Select(pe => pe.Eval()).ToArray();
//     }

//     Vector3[] GetPoints() {
//         return GetPointsInPlane(sketch.plane);
//     }

//     ExpVector[] GetPointsExp(IPlane plane) {
//         var p = new ExpVector[4];
//         if(HasEntitiesOfType(IEntityType.Point, 4)) {
//             for(int i = 0; i < 4; i++) {
//                 p[i] = GetEntityOfType(IEntityType.Point, i).GetPointAtInPlane(0, plane);
//             }
//             if(supplementary) {
//                 SystemExt.Swap(ref p[2], ref p[3]);
//             }
//         } else 
//         if(HasEntitiesOfType(IEntityType.Line, 2)) {
//             var l0 = GetEntityOfType(IEntityType.Line, 0);
//             p[0] = l0.GetPointAtInPlane(0, plane);
//             p[1] = l0.GetPointAtInPlane(1, plane);
//             var l1 = GetEntityOfType(IEntityType.Line, 1);
//             p[2] = l1.GetPointAtInPlane(0, plane);
//             p[3] = l1.GetPointAtInPlane(1, plane);
//             if(supplementary) {
//                 SystemExt.Swap(ref p[2], ref p[3]);
//             }
//         } else 
//         if(HasEntitiesOfType(IEntityType.Arc, 1)) {
//             var arc = GetEntityOfType(IEntityType.Arc, 0);
//             p[0] = arc.GetPointAtInPlane(0, plane);
//             p[1] = arc.GetPointAtInPlane(2, plane);
//             p[2] = arc.GetPointAtInPlane(2, plane);
//             p[3] = arc.GetPointAtInPlane(1, plane);
//             if(supplementary) {
//                 SystemExt.Swap(ref p[0], ref p[3]);
//                 SystemExt.Swap(ref p[1], ref p[2]);
//             }
//         }
//         return p;
//     }
//     protected override Matrix4x4 OnGetBasis() {
//         var pos = GetPoints();
//         var p = pos[1];
//         double angle = Math.Abs(GetValue());
//         Vector3 z = Vector3.zero;
//         if(Math.Abs(Math.Abs(angle) - 180.0) < EPSILON) {
//             p = pos[1];
//             if(sketch.plane != null) z = -sketch.plane.n;
//         } else
//         if(GeomUtils.isLinesCrossed(pos[0], pos[1], pos[2], pos[3], ref p, Mathf.Epsilon)) {
//             z = Vector3.Cross(pos[0] - pos[1], pos[3] - pos[2]).normalized;
//         }
//         if(z.magnitude < Mathf.Epsilon) z = new Vector3(0.0f, 0.0f, 1.0f);
        
//         var y = Quaternion.AngleAxis((float)angle / 2f, z) * (pos[0] - pos[1]).normalized;
//         var x = Vector3.Cross(y, z).normalized;
//         var result = UnityExt.Basis(x, y, z, p);
//         return getPlane().GetTransform() * result;
//     }
// }



using ConstraintPtr = std::shared_ptr<Constraint>;
using EntityPtr = std::shared_ptr<Entity>;

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