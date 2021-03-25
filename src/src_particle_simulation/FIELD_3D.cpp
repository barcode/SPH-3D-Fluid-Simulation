#include "FIELD_3D.h"

FIELD_3D::FIELD_3D() :
    FIELD_3D(0, 0, 0)
{}

FIELD_3D::FIELD_3D(int xRes, int yRes, int zRes) :
    _xRes(xRes), _yRes(yRes), _zRes(zRes), _cellCount(xRes * yRes * zRes), _data(_xRes * _yRes * _zRes)
{}
