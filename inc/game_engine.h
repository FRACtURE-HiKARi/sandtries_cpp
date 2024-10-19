#pragma once

/*
 * Defines the game engine, renderer, scene, events, ...
 */

#include "physics_engine.h"
#include "utils.h"
#include <graphics.h>

class Scene {
protected:
    std::vector <Node> objects;
public:
    std::vector <Node> getObjects();
    void addObject(Node& object);
};

class Renderer {

public:
    Renderer(int window_width, int window_height);
    void render(const Scene& scene);
};
