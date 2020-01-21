#include <iostream>
#include "expression.hpp"
#include "entity.hpp"
#include "constraint.hpp"


int main()
{
    auto a = expr(123);
    auto b = expr(323);
    auto p = param("test", 1.);
    auto c = (a + sin(b) * sin(p->expr()));
    std::cout << (a + sin(b) * sin(p->expr()))->to_string() << std::endl;

    std::cout << c->d(p)->to_string() << std::endl;
    std::cout << "Hello world." << std::endl;

    auto p1 = std::make_shared<PointE>(param("p1_x", 3), param("p1_y", 1), param("p1_z", 1));
    auto p2 = std::make_shared<PointE>(param("p2_x", 4), param("p2_y", 2), param("p2_z", 1));
    auto p3 = std::make_shared<PointE>(param("p3_x", 10.5), param("p3_y", 1.5), param("p3_z", 1));

    auto l = std::make_shared<LineE>(*p1, *p2);

    std::cout << l->to_string() << std::endl;
    std::cout << p3->to_string() << std::endl;

    Sketch s;
    s.add_entity(p1);
    s.add_entity(l);

    std::cout << "Adding Point On" << std::endl;
    auto ccc = std::make_shared<PointOnConstraint>(p3, l);
    s.add_constraint(ccc);
    s.update();
    s.sys.solve();
    std::cout << "Adding length" << std::endl;
    auto lC = std::make_shared<LengthConstraint>(l, 15);
    s.add_constraint(lC);
    s.update();
    s.sys.solve();

    std::cout << "Adding HV Constraint" << std::endl;
    auto HC = std::make_shared<HVConstraint>(l, OX);
    s.add_constraint(HC);
    s.update();
    s.sys.solve();

    std::cout << s.sys.solve() << std::endl;

    int rank;
    s.sys.test_rank(rank);

    std::cout << "RANK: " << rank << std::endl;

    for (auto& el : s.entities)
    {
        std::cout << el->to_string() << std::endl;
    }


    return 0;
}