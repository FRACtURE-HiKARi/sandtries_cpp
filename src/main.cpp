#include <iostream>
#include "sandtries.h"
#include <windows.h>
#include <sstream>

// TODO: collision handling
// TODO: optimize support func & BroadPhase: kd-trees or uniform tiles

int main(){
    initgraph(320, 640);
    PhysicsEngine e(10);
    TestObj o(75);
    TestStatic s(75);
    e.addRigid(o);
    e.addStatic(s);
    o.setPos({250, 320});
    s.setPos({130, 480});
    float dt = 1.f/30;
    while (true) {
        o.addTorque(500);
        long t_start = clock();
        cleardevice();
        e.updateObjects(dt);
        long t_end = clock();
        o.show();
        s.show();
        RECT r = {50, 10, 300, 30};
        std::stringstream ss;
        ss << "Delay: " << t_end - t_start << "ms";
        drawtext(_T(ss.str().c_str()), &r, DT_LEFT);
        //std::cout << o.RigidBody::getPosition();
        Sleep((unsigned)(dt * 1000));
    }
    return 0;
}