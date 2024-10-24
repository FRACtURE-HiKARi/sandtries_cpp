#include "game_engine.h"

Renderer::Renderer(int window_width, int window_height) {
    width = window_width;
    height = window_height;
    hwnd = initgraph(width, height);
    imageBuffer = GetImageBuffer();
}

void Renderer::render(Scene &s) {
    for (auto v: s.getVisible())
        ((Visible*)v)->show();
    FlushBatchDraw();
}

void Renderer::drawText(const std::string& str, RECT *rect, unsigned format) {
    drawtext(str.c_str(), rect, format);
}

void Renderer::drawText(const std::string& str, long left, long top) {
    RECT rect = {left, top, width-1, height-1};
    drawText(str, &rect, DT_TOP | DT_BOTTOM);
}
