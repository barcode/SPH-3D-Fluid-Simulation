#pragma once

//////////////////////////////////////////////////////////////////////
// This code is based on code from the (excellent) libgfx library by
// Michael Garland:
//
// http://mgarland.org/software/libgfx.html
//////////////////////////////////////////////////////////////////////

#include <iostream>
#include <cmath>
#include <array>

template<class FT>
class VEC3
{
public:
    // Standard constructors
    VEC3(FT x = 0, FT y = 0, FT z = 0)
    {
        _element.at(0) = x;
        _element.at(1) = y;
        _element.at(2) = z;
    }

    // Copy constructors & assignment operators
    VEC3(const VEC3& v)
    {
        *this = v;
    }
    VEC3& operator=(FT s)
    {
        _element.at(0) = _element.at(1) = _element.at(2) = s;
        return *this;
    }

    // Access methods
    const FT* data() const
    {
        return _element.data();
    }
    FT& at(int i)
    {
        return _element.at(i);
    }

    VEC3& operator+=(const VEC3& v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }
    VEC3& operator-=(const VEC3& v)
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }
    VEC3& operator/=(const VEC3& v)
    {
        x /= v.x;
        y /= v.y;
        z /= v.z;
        return *this;
    }
    VEC3& operator/=(const FT b)
    {
        x /= b;
        y /= b;
        z /= b;
        return *this;
    }
    VEC3& operator*=(const FT b)
    {
        x *= b;
        y *= b;
        z *= b;
        return *this;
    }
    VEC3 operator+(const VEC3& b) const
    {
        return VEC3(x + b.x, y + b.y, z + b.z);
    }
    VEC3 operator-(const VEC3& b) const
    {
        return VEC3(x - b.x, y - b.y, z - b.z);
    }
    VEC3 operator*(const FT b) const
    {
        return VEC3(x * b, y * b, z * b);
    }
    VEC3 operator/(const FT b) const
    {
        return VEC3(x / b, y / b, z / b);
    }

    // this computes the cross product
    VEC3 operator^(const VEC3& v) const
    {
        return VEC3(y * v.z - v.y * z, -x * v.z + v.x * z, x * v.y - v.x * y);
    }

    // these are *element-by-element* multiplies, not dot products
    VEC3 operator*(const VEC3& v) const
    {
        return VEC3(x * v.x, y * v.y, z * v.z);
    }
    VEC3& operator*=(const VEC3& v)
    {
        x *= v.x;
        y *= v.y;
        z *= v.z;
        return *this;
    }

    // *this one* does the dot product
    FT dot(const VEC3& b) const
    {
        return x * b.x + y * b.y + z * b.z;
    }
    void clear()
    {
        _element.at(0) = 0;
        _element.at(1) = 0;
        _element.at(2) = 0;
    }

    FT magnitude()
    {
        return sqrt(_element.at(0) * _element.at(0) + _element.at(1) * _element.at(1) + _element.at(2) * _element.at(2));
    }
    VEC3 normalize()
    {
        FT l = _element.at(0) * _element.at(0) +
               _element.at(1) * _element.at(1) +
               _element.at(2) * _element.at(2);
        if (l != 1.0 && l != 0.0)
        {
            FT inv = 1.0 / sqrt(l);
            _element.at(0) *= inv;
            _element.at(1) *= inv;
            _element.at(2) *= inv;
        }
        return *this;
    }
    VEC3 normal()
    {
        VEC3 a = *this;
        a.normalize();
        return a;
    }

    FT maxVal()
    {
        return x > y && x > z ? x : y > z ? y : z;
    }

    // the data
    union
    {
        struct
        {
            FT x, y, z;
        };
        struct
        {
            FT r, g, b;
        };
        std::array<FT, 3> _element;
    };
};

template<class FT>
inline std::istream& operator>>(std::istream& in, VEC3<FT>& v)
{
    return in >> v.at(0) >> v.at(1) >> v.at(2);
}

template<class FT>
inline std::ostream& operator<<(std::ostream& out, VEC3<FT>& v)
{
    return out << v.at(0) << " " << v.at(1) << " " << v.at(2);
}

template<class FT>
inline VEC3<FT> operator*(const FT a, const VEC3<FT>& b)
{
    return VEC3<FT>(b.x * a, b.y * a, b.z * a);
}

using VEC3F = VEC3<float>;
using VEC3D = VEC3<double>;
