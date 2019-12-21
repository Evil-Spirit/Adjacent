#include <iostream>
#include "expression.hpp"

int main ()
{
	auto a = expr(123);
	auto b = expr(323);
	auto p = Param<double>("test", 1.);
	auto c = (a + sin(b) * sin(p.expr()));
	std::cout << (a + sin(b) * sin(p.expr()))->to_string() << std::endl;

	std::cout << c->d(p)->to_string() << std::endl;
	std::cout << "Hello world." << std::endl;
	return 0;
}