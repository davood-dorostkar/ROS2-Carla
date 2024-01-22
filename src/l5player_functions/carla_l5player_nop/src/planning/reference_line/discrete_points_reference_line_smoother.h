/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <utility>
#include <vector>

#include "reference_line_smoother.h"

namespace l5player {
namespace reference_line {

class DiscretePointsReferenceLineSmoother : public ReferenceLineSmoother {
   public:
    enum class SmoothingMethod { COS_THETA_SMOOTHING = 1, FEM_POS_DEVIATION_SMOOTHING = 2, UNDERLINE = 4 };

   public:
    explicit DiscretePointsReferenceLineSmoother() {
        smoothing_method_ = SmoothingMethod::FEM_POS_DEVIATION_SMOOTHING;
    };

    virtual ~DiscretePointsReferenceLineSmoother() = default;

    bool Smooth(nav_msgs::msg::Path& referenceline) override;

    void SetAnchorPoints(const std::vector<AnchorPoint>&) override;

   private:
    bool CosThetaSmooth(const std::vector<std::pair<double, double>>& raw_point2d, const std::vector<double>& bounds,
                        std::vector<std::pair<double, double>>* ptr_smoothed_point2d);

    bool FemPosSmooth(const std::vector<std::pair<double, double>>& raw_point2d, const std::vector<double>& bounds,
                      std::vector<std::pair<double, double>>* ptr_smoothed_point2d);

    void NormalizePoints(std::vector<std::pair<double, double>>* xy_points);

    void DeNormalizePoints(std::vector<std::pair<double, double>>* xy_points);

    bool GenerateRefPointProfile(const Eigen::MatrixXd& raw_reference_line,
                                 const std::vector<std::pair<double, double>>& xy_points,
                                 std::vector<ReferencePoint>* reference_points);

    std::vector<AnchorPoint> anchor_points_;

    double zero_x_ = 0.0;

    double zero_y_ = 0.0;

    SmoothingMethod smoothing_method_;
};

}    // namespace reference_line
}    // namespace l5player
