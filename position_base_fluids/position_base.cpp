//
// Created by 北冥咸鱼 on 2022/8/21.
//

#include "position_base.hpp"

PositionBase::PositionBase(std::vector<Object *> & objects, const Bound3D &bound, const float &r, const float &h,
                           const float &p0, const float &g0)
                           : objects_(objects), bound3D_(bound), h_(h), p0_(p0), g0_(g0), m0_(r * r * p0_) {}

