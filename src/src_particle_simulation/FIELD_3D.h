#pragma once

#include <cstdlib>
#include <iostream>
#include <vector>

#include "assert.h"
#include "PARTICLE.h"

class FIELD_3D
{
    typedef std::vector<PARTICLE> particleVector;

public:
    FIELD_3D();
    FIELD_3D(int xRes, int yRes, int zRes);
    virtual ~FIELD_3D() = default;

    inline particleVector& operator()(int x, int y, int z)
    {
        return _data.at(x + y * _xRes + z * _xRes * _yRes);
    }

    // accessors
    int xRes() const
    {
        return _xRes;
    }
    int yRes() const
    {
        return _yRes;
    }
    int zRes() const
    {
        return _zRes;
    }
    int cellCount() const
    {
        return _cellCount;
    }
    std::vector<particleVector>& data()
    {
        return _data;
    }
    const std::vector<particleVector>& data() const
    {
        return _data;
    }

private:

    int _xRes;
    int _yRes;
    int _zRes;
    int _cellCount;

    std::vector<particleVector> _data;
};
