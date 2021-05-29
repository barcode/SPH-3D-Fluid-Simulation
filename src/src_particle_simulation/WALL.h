#pragma once

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include "PARTICLE.h"
#include "VEC3.h"

class WALL
{
public:
    WALL(const VEC3D& normal, const VEC3D& point);

    // accessors
    const VEC3D& normal() const
    {
        return _normal;
    }
    const VEC3D& point() const
    {
        return _point;
    }

private:
    VEC3D _normal;
    VEC3D _point;
};
