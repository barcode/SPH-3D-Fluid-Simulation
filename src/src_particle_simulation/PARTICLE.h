#pragma once

#ifndef NO_OPENGL
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif
#endif

#include "VEC3.h"
#include <vector>

class PARTICLE
{
public:
    static bool isSurfaceVisible;
    static bool showArrows;

    PARTICLE(const PARTICLE&) = default;
    PARTICLE(PARTICLE&&) = default;
    PARTICLE& operator=(const PARTICLE&) = default;
    PARTICLE& operator=(PARTICLE&&) = default;
    //static unsigned int count;
    PARTICLE();
    PARTICLE(const VEC3D& position);
    PARTICLE(const VEC3D& position, const VEC3D& velocity);
    //~PARTICLE();
#ifndef NO_OPENGL
    // draw to OGL
    void draw();
#endif
    // clear all previous accumulated forces

    // accumulate forces


    // accessors
    VEC3D& position()
    {
        return _position;
    }
    const VEC3D& position() const
    {
        return _position;
    }
    VEC3D& velocity()
    {
        return _velocity;
    }
    VEC3D& acceleration()
    {
        return _acceleration;
    }
    double& density()
    {
        return _density;
    }
    double& pressure()
    {
        return _pressure;
    }
    bool& flag()
    {
        return _flag;
    }
    int& id()
    {
        return _id;
    }
    int id() const
    {
        return _id;
    }
    VEC3D normal;

    void clearParameters();

    //static unsigned int count;

private:
    VEC3D _position;
    VEC3D _velocity;
    VEC3D _acceleration;
    double _density;
    double _pressure;
    bool _flag;
    int _id = 0;
#ifndef NO_OPENGL
    GLUquadricObj* myQuadric = nullptr;
#endif
};
