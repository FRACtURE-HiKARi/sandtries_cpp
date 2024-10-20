#include <iostream>
#include "sandtries.h"
#include <windows.h>

// TODO: collision handling
// TODO: Test GJK on Ball Collider
// TODO: optimize support func & BroadPhase

int main(){
    initgraph(320, 640);
    PhysicsEngine e(10);
    TestObj o(75);
    TestStatic s(100, 40);
    e.addRigid(o);
    e.addStatic(s);
    o.setPos({250, 320});
    s.setPos({130, 480});
    float dt = 1.f/30;
    while (true) {
        o.addTorque(500);
        cleardevice();
        e.updateObjects(dt);
        o.show();
        s.show();
        //std::cout << o.RigidBody::getPosition();
        Sleep((unsigned)(dt * 1000));
    }
    return 0;
}