#pragma once

#include "PARTICLE.h"
#include "VEC3.h"

class WALL
{
public:
    WALL(const VEC3D& normal, const VEC3D& point) :
        _normal(normal), _point(point)
    {
        // just in case, normalize the normal vector
        _normal.normalize();
    }

    // accessors
    const VEC3D& normal() const
    {
        return _normal;
    }
    const VEC3D& point() const
    {
        return _point;
    }

    std::pair<double, VEC3D> collide(const PARTICLE& p, double part_r) const
    {
        return {(point() - p.position()).dot(normal()) + part_r, normal()};
    }

private:
    VEC3D _normal;
    VEC3D _point;
};
