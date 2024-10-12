#include <iostream>
#include "types.h"

int main(){
    Vec2 v1 = {1, 4};
    Vec2 v2 = {2, 3};
    std::cout << v1+v2 << '\n' << v1-v2 << std::endl;
    return 0;
}