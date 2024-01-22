/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <vector>

#include "reference_point.h"
#include <nav_msgs/msg/path.hpp>

namespace l5player {
namespace reference_line {

struct AnchorPoint {
    ReferencePoint path_point;
    double lateral_bound = 0.0;
    double longitudinal_bound = 0.0;
    // enforce smoother to strictly follow this reference point
    bool enforced = false;
};

class ReferenceLineSmoother {
   public:
    ReferenceLineSmoother(){};
    virtual ~ReferenceLineSmoother() = default;

    /**
     * Smoothing constraints
     */
    virtual void SetAnchorPoints(const std::vector<AnchorPoint>& achor_points) = 0;

    /**
     * Smooth a given reference line
     */
    virtual bool Smooth(nav_msgs::msg::Path& referenceline) = 0;

   protected:
};

}    // namespace reference_line
}    // namespace l5player
