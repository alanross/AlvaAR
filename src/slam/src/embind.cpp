#include <emscripten/bind.h>
#include <emscripten/val.h>
#include <emscripten/emscripten.h>

#include "./system.hpp"

using namespace emscripten;

EMSCRIPTEN_BINDINGS(Module)
{
    class_<System>("System")
        .constructor()
        .function("configure", &System::configure)
        .function("reset", &System::reset)
        .function("findCameraPoseWithIMU", &System::findCameraPoseWithIMU, allow_raw_pointers())
        .function("findCameraPose", &System::findCameraPose, allow_raw_pointers())
        .function("findPlane", &System::findPlane)
        .function("getFramePoints", &System::getFramePoints);
}