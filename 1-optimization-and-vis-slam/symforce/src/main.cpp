#include <symforce/opt/values.h>
#include <symforce/opt/optimizer.h>
#include <iostream>

int main() {
    sym::Values<double> values;
    values.Set('x', 10.0);

    std::cout << "SymForce C++ Optimizer is linked and ready!" << std::endl;
    std::cout << "Stored initial guess for 'x': " << values.At<double>('x') << std::endl;


    return 0;
}