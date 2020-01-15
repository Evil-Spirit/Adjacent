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

    PointE point1(param("p1_x", 3), param("p1_y", 2), param("p1_z", 1));
    PointE point2(param("p2_x", 4), param("p2_y", 2), param("p2_z", 1));
    PointE point3(param("p3_x", 4), param("p3_y", 2), param("p3_z", 1));

    LineE line(point1, point2);

    std::cout << line.to_string() << std::endl;
    std::cout << point3.to_string() << std::endl;

    auto ccc = PointOnConstraint(&point3, &line);

    return 0;
}