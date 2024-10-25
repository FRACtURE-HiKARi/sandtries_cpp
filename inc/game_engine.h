#pragma once

/*
 * Defines the game engine, renderer, scene, events, ...
 */

#include "physics_engine.h"
#include "utils.h"
#include <graphics.h>
#include <sstream>

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
    int db_left = 50, db_top = 10, db_top_offset = 0;
    std::stringstream debug_strings;
public:
    Renderer(int window_width, int window_height);
    void render(Scene &scene);
    static void drawText(const std::string& str, RECT* rect, unsigned format);
    void drawText(const std::string& str, long left, long top);
    void drawDebugInfo();
    void addDebugInfo(const std::string& str);
    template <typename T>
    void addDebugInfo(const std::string &name, const T& value) {
        debug_strings << name << ": " << value << '\n';
    }
};
