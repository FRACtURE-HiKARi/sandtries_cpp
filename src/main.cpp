#include <iostream>
#include "sandtries.h"
#include <windows.h>
#include <conio.h>

// TODO: collision handling
// TODO: handle EPA only on polygon, and write another one for circle
// TODO: optimize support func & BroadPhase: kd-trees or uniform split
// TODO:? multi-thread when updating objects / refreshing screen

int main(){
    Renderer renderer(320, 640);
    PhysicsEngine e(100);
    e.setRenderer(renderer);
    int radius = 40;
    TestObj o((float)radius);
    //TestStatic o(radius, radius);
    TestStatic s(150, 100);
    e.addRigid(o);
    //e.addStatic(o);
    e.addStatic(s);
    o.setRotation(Calculations::angle2RotMat(m_pi/6));
    o.setPos({220, 320});
    s.setPos({130, 480});
    Scene scene;
    scene.addVisible(s);
    scene.addVisible(o);
    float dt = 1.f/60;
    BeginBatchDraw();
    bool dragging = false;
    int offsetX, offsetY;
    Vec3 pos;
    while (true) {
        //o.addTorque(500);
        long t_start = clock();
        cleardevice();
        if (dragging)
            pos = o.global_centroid();
        e.updateObjects(dt);
        if (dragging)
            o.setPosition(pos.squeeze());
        long t_end = clock();
        // capture mouse
        ExMessage msg{};
        if (peekmessage(&msg, EM_MOUSE)) {
            Vec3 p = o.global_centroid();
            Vec3 m = {(float)msg.x, (float)msg.y, 0};
            switch (msg.message) {
                case WM_LBUTTONDOWN:
                    if ((p - m).abs() <= (float)radius) {
                        dragging = true;
                        offsetX = msg.x - (int)p.x;
                        offsetY = msg.y - (int)p.y;
                    }
                    break;
                case WM_LBUTTONUP:
                    dragging = false;
                    break;
                default:
                    if (dragging){
                        int x = msg.x - offsetX;
                        int y = msg.y - offsetY;
                        o.setPos({(float)x, (float)y});
                    }
                    break;
            }
        }
        renderer.addDebugInfo("Delay", t_end - t_start);
        renderer.render(scene);
        //std::cout << o.RigidBody::getPosition();
        Sleep((unsigned)(dt * 1000));
    }
    return 0;
}

/*
int main() {
    // 初始化图形窗口
    initgraph(640, 480);
    int w = 240, h = 180;
    int x = 200, y = 200;
    bool dragging = false;
    int offsetX, offsetY;
    BeginBatchDraw();
    while (true) {
        cleardevice();
        fillrectangle(x, y, x+w, y+h);
        FlushBatchDraw();
        ExMessage msg{};
        if (peekmessage(&msg, EM_MOUSE)) {
            switch (msg.message) {
                case WM_LBUTTONDOWN:
                    dragging = true;
                    offsetX = msg.x - x;
                    offsetY = msg.y - y;
                    break;
                case WM_MOUSEMOVE:
                    if (dragging){
                        x = msg.x - offsetX;
                        y = msg.y - offsetY;
                    }
                    break;
                case WM_LBUTTONUP:
                    dragging = false;
            }
        }
    }
    return 0;

}
 */