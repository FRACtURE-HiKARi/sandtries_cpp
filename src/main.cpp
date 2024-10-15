#include <iostream>
#include "types.h"

int main(){
    float a = 0.2f;
    float b = 0.4f;
    Mat2 M = Calculations::angle2RotMat(a);
    Mat2 N = Calculations::angle2RotMat(b);
    Mat2 P = Calculations::angle2RotMat(a+b);
    std::cout << M << M*N << '\n' << P << std::endl;
    return 0;
}