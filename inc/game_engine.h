#pragma once

/*
 * Defines the game engine, renderer, scene, events, ...
 */

#include "physics_engine.h"
#include "utils.h"
#include <graphics.h>

typedef std::list<Node*> ObjectList;
typedef struct Color {
    byte r;
    byte g;
    byte b;
} Color;

class Scene {
protected:
    ObjectList objects;
    ObjectList visible;
public:
    ObjectList& getObjects() {return objects; }
    ObjectList& getVisible() {return visible; }
    void addObject(Node &object) {objects.push_back(&object);}
    void addVisible(Visible &v) {visible.push_back(&v);}
};

class Renderer {
    int width, height;
    DWORD* imageBuffer;
    HWND hwnd;
public:
    Renderer(int window_width, int window_height);
    void render(Scene &scene);
    static void drawText(const std::string& str, RECT* rect, unsigned format);
    void drawText(const std::string& str, long left, long top);
};

static Renderer renderer(320, 640);
