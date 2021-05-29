#pragma once

#include "PARTICLE.h"
#include "VEC3.h"

class CYLINDRICAL_WALL
{
public:
    CYLINDRICAL_WALL(const VEC3D& center, const VEC3D& up, double r, bool contain) :
        _center(center), _up(up), _r(r), _contain(contain)
    {
        if (!r > 0)
        {
            throw std::invalid_argument{"CYLINDRICAL_WALL: r hast to be >0! r = " + std::to_string(r)};
        }
        // just in case, normalize the normal vector
        _up.normalize();
    }

    // accessors
    const VEC3D& center() const
    {
        return _center;
    }
    const VEC3D& up() const
    {
        return _up;
    }
    double r() const
    {
        return _r;
    }
    bool contain() const
    {
        return _contain;
    }

    std::pair<double, VEC3D> collide(const PARTICLE& p, double part_r) const
    {
        using RT = std::pair<double, VEC3D>;
        const auto relp = p.position()-center();
        const auto relu = relp.dot(up()) * up();
        const auto rels = relp - relu;
        const double sidedist = rels.magnitude();


        if (contain())
        {
            //the particle has to be inside
            return (sidedist + part_r <= r()) ?
                   RT{0, {0, 0, 0}} :
                   RT{sidedist + part_r - r(), -rels.normal()};
        }
        //the particle has to be outside
        return (sidedist - part_r >= r()) ?
               RT{0, {0, 0, 0}} :
               RT{r() - sidedist - part_r, rels.normal()};
    }

private:
    VEC3D _center;
    VEC3D _up;
    double _r;
    bool _contain;
};
